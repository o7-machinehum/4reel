// 4reel stereo visual odometry prototype. Camera frames must be synchronized;
// libviso2 requires rectified 8-bit grayscale images and measured calibration.
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "viso_stereo.h"

namespace {

volatile sig_atomic_t stopped = 0;
void stop_handler(int) { stopped = 1; }

void fail(const std::string &message) { throw std::runtime_error(message); }

void xioctl(int fd, unsigned long request, void *arg, const char *operation) {
  int result;
  do { result = ioctl(fd, request, arg); } while (result < 0 && errno == EINTR);
  if (result < 0) fail(std::string(operation) + ": " + strerror(errno));
}

uint32_t read_le32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 |
         uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

uint16_t read_le16(const uint8_t *p) {
  return uint16_t(p[0]) | uint16_t(p[1]) << 8;
}

struct Options {
  std::string left = "/dev/video0";
  std::string right = "/dev/video11";
  std::string capture_dir;
  std::string left_map;
  std::string right_map;
  std::string calib;
  uint32_t width = 1280;
  uint32_t height = 800;
  uint32_t format = V4L2_PIX_FMT_Y10;
  uint32_t sync_ms = 10;
  uint32_t frames = 0;
};

uint32_t number(const char *text, const char *name) {
  char *end = NULL;
  errno = 0;
  unsigned long value = strtoul(text, &end, 10);
  if (errno || !text[0] || *end || value > UINT32_MAX)
    fail(std::string("invalid ") + name + ": " + text);
  return uint32_t(value);
}

void usage() {
  fprintf(stderr,
      "Usage: 4reel-vo [--left /dev/video0] [--right /dev/video11] [--width 1280] [--height 800]\n"
      "                 [--format y10|grey] [--sync-ms 10] [--frames N]\n"
      "                 --capture-dir DIR\n"
      "   or: 4reel-vo [camera options] --left-map L.map --right-map R.map --calib vo.calib\n"
      "Capture mode writes paired PGM files. VO mode prints camera-to-world poses.\n"
      "Neither mode can establish physical exposure synchronization from timestamps alone.\n");
}

Options parse_options(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help") { usage(); exit(0); }
    if (i + 1 >= argc) fail("missing value after " + a);
    const char *v = argv[++i];
    if (a == "--left") o.left = v;
    else if (a == "--right") o.right = v;
    else if (a == "--width") o.width = number(v, "width");
    else if (a == "--height") o.height = number(v, "height");
    else if (a == "--sync-ms") o.sync_ms = number(v, "sync-ms");
    else if (a == "--frames") o.frames = number(v, "frames");
    else if (a == "--capture-dir") o.capture_dir = v;
    else if (a == "--left-map") o.left_map = v;
    else if (a == "--right-map") o.right_map = v;
    else if (a == "--calib") o.calib = v;
    else if (a == "--format") {
      if (!strcmp(v, "y10")) o.format = V4L2_PIX_FMT_Y10;
      else if (!strcmp(v, "grey")) o.format = V4L2_PIX_FMT_GREY;
      else fail("format must be y10 or grey");
    } else fail("unknown option: " + a);
  }
  if (!o.width || !o.height || o.width > 4096 || o.height > 4096)
    fail("invalid camera dimensions");
  if (o.format == V4L2_PIX_FMT_Y10 && o.width % 4)
    fail("packed Y10 width must be divisible by four");
  if (o.sync_ms > 100) fail("sync-ms must be at most 100");
  if (o.left == o.right) fail("left and right camera devices must differ");
  if (o.capture_dir.empty() &&
      (o.left_map.empty() || o.right_map.empty() || o.calib.empty()))
    fail("VO mode requires both rectification maps and calibration");
  if (!o.capture_dir.empty() &&
      (!o.left_map.empty() || !o.right_map.empty() || !o.calib.empty()))
    fail("capture mode cannot also run VO");
  return o;
}

struct Buffer {
  void *data = NULL;
  size_t length = 0;
};

struct Frame {
  unsigned index = 0;
  const uint8_t *data = NULL;
  size_t bytes = 0;
  int64_t time_us = 0;
};

uint8_t raw_pixel(uint32_t format, bool packed10, uint32_t stride, const uint8_t *data,
                  uint16_t x, uint16_t y) {
  const uint8_t *row = data + size_t(y) * stride;
  if (format != V4L2_PIX_FMT_Y10) return row[x];
  // This CIF's compact Y10 is four consecutive little-endian 10-bit pixels
  // in five bytes; discard the two low bits of each pixel.
  if (packed10) {
    const uint8_t *group = row + (x / 4) * 5;
    switch (x % 4) {
      case 0: return (group[0] >> 2) | ((group[1] & 3) << 6);
      case 1: return (group[1] >> 4) | ((group[2] & 15) << 4);
      case 2: return (group[2] >> 6) | ((group[3] & 63) << 2);
      default: return group[4];
    }
  }
  const uint8_t *word = row + size_t(x) * 2;
  return read_le16(word) >> 2;
}

void self_test() {
  const uint8_t raw10[5] = {0x40, 0x00, 0x02, 0x0C, 0x40};
  for (uint16_t x = 0; x < 4; ++x)
    if (raw_pixel(V4L2_PIX_FMT_Y10, true, 5, raw10, x, 0) != (x + 1) * 0x10)
      fail("RAW10 conversion self-test failed");
  const uint8_t y10[2] = {0xFF, 0x03};
  if (raw_pixel(V4L2_PIX_FMT_Y10, false, 2, y10, 0, 0) != 255)
    fail("unpacked Y10 conversion self-test failed");
  const uint8_t le[4] = {0x78, 0x56, 0x34, 0x12};
  if (read_le16(le) != 0x5678 || read_le32(le) != 0x12345678)
    fail("rectification map byte-order self-test failed");
  puts("4reel-vo self-test OK");
}

class Camera {
 public:
  Camera(const std::string &path, const Options &o) : path_(path), format_(o.format) {
    fd_ = open(path.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) fail(path + ": " + strerror(errno));

    v4l2_capability cap = {};
    xioctl(fd_, VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP");
    uint32_t caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ?
                    cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_STREAMING)) fail(path + ": no streaming support");
    type_ = caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE ?
            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!(caps & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_CAPTURE)))
      fail(path + ": not a capture node");

    v4l2_format fmt = {};
    fmt.type = type_;
    if (multiplanar()) {
      fmt.fmt.pix_mp.width = o.width;
      fmt.fmt.pix_mp.height = o.height;
      fmt.fmt.pix_mp.pixelformat = format_;
      fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
      fmt.fmt.pix_mp.num_planes = 1;
    } else {
      fmt.fmt.pix.width = o.width;
      fmt.fmt.pix.height = o.height;
      fmt.fmt.pix.pixelformat = format_;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }
    xioctl(fd_, VIDIOC_S_FMT, &fmt, "VIDIOC_S_FMT");
    width_ = multiplanar() ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
    height_ = multiplanar() ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    uint32_t got_format = multiplanar() ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
    stride_ = multiplanar() ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline :
                              fmt.fmt.pix.bytesperline;
    if (width_ != o.width || height_ != o.height || got_format != format_)
      fail(path + ": driver changed requested size or pixel format");
    if (multiplanar() && fmt.fmt.pix_mp.num_planes != 1)
      fail(path + ": only one-plane capture is supported");
    packed10_ = format_ == V4L2_PIX_FMT_Y10 && stride_ < width_ * 2;
    size_t row_bytes = format_ == V4L2_PIX_FMT_Y10 ?
                       (packed10_ ? width_ * 5 / 4 : width_ * 2) : width_;
    if (stride_ < row_bytes) fail(path + ": invalid bytes-per-line");
    fprintf(stderr, "%s: %ux%u, stride %u, %s\n", path.c_str(), width_,
            height_, stride_, format_ == V4L2_PIX_FMT_GREY ? "GREY" :
            (packed10_ ? "packed Y10" : "unpacked Y10"));

    v4l2_requestbuffers req = {};
    req.count = 2;
    req.type = type_;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(fd_, VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");
    if (req.count < 2) fail(path + ": needs two capture buffers");
    buffers_.resize(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
      v4l2_buffer buf = {};
      v4l2_plane plane = {};
      buf.type = type_;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (multiplanar()) { buf.m.planes = &plane; buf.length = 1; }
      xioctl(fd_, VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");
      size_t length = multiplanar() ? plane.length : buf.length;
      off_t offset = multiplanar() ? plane.m.mem_offset : buf.m.offset;
      void *data = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, offset);
      if (data == MAP_FAILED) fail(path + ": mmap: " + strerror(errno));
      buffers_[i].data = data;
      buffers_[i].length = length;
      queue(i);
    }
    int type = type_;
    xioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  }

  ~Camera() {
    if (streaming_) { int type = type_; ioctl(fd_, VIDIOC_STREAMOFF, &type); }
    for (size_t i = 0; i < buffers_.size(); ++i)
      if (buffers_[i].data) munmap(buffers_[i].data, buffers_[i].length);
    if (fd_ >= 0) close(fd_);
  }

  Camera(const Camera &) = delete;
  Camera &operator=(const Camera &) = delete;

  bool dequeue(Frame &frame) {
    v4l2_buffer buf = {};
    v4l2_plane plane = {};
    buf.type = type_;
    buf.memory = V4L2_MEMORY_MMAP;
    if (multiplanar()) { buf.m.planes = &plane; buf.length = 1; }
    int result;
    do { result = ioctl(fd_, VIDIOC_DQBUF, &buf); } while (result < 0 && errno == EINTR);
    if (result < 0 && errno == EAGAIN) return false;
    if (result < 0) fail(path_ + ": VIDIOC_DQBUF: " + strerror(errno));
    if (buf.index >= buffers_.size()) fail(path_ + ": bad buffer index");
    if (!(buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC))
      fail(path_ + ": timestamps are not monotonic; cannot pair frames safely");
    frame.index = buf.index;
    size_t offset = multiplanar() ? plane.data_offset : 0;
    size_t used = multiplanar() ? plane.bytesused : buf.bytesused;
    if (!used) used = buffers_[buf.index].length;
    if (offset > buffers_[buf.index].length || used < offset ||
        used > buffers_[buf.index].length)
      fail(path_ + ": invalid captured plane bounds");
    frame.data = static_cast<const uint8_t *>(buffers_[buf.index].data) + offset;
    frame.bytes = used - offset;
    frame.time_us = int64_t(buf.timestamp.tv_sec) * 1000000 + buf.timestamp.tv_usec;
    size_t payload = format_ == V4L2_PIX_FMT_Y10 ?
                     (packed10_ ? width_ * 5 / 4 : width_ * 2) : width_;
    if (frame.bytes < size_t(height_ - 1) * stride_ + payload)
      fail(path_ + ": short frame");
    return true;
  }

  void queue(unsigned index) {
    v4l2_buffer buf = {};
    v4l2_plane plane = {};
    buf.type = type_;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (multiplanar()) { buf.m.planes = &plane; buf.length = 1; }
    xioctl(fd_, VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
  }

  uint8_t pixel(const Frame &frame, uint16_t x, uint16_t y) const {
    return raw_pixel(format_, packed10_, stride_, frame.data, x, y);
  }
  int fd() const { return fd_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

 private:
  bool multiplanar() const { return type_ == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; }
  std::string path_;
  int fd_ = -1;
  int type_ = 0;
  uint32_t format_, width_ = 0, height_ = 0, stride_ = 0;
  bool packed10_ = false;
  bool streaming_ = false;
  std::vector<Buffer> buffers_;
};

struct Point { uint16_t x, y; };
struct Map {
  uint32_t width, height, source_width, source_height;
  std::vector<Point> points;
};

struct PoseState {
  Matrix pose = Matrix::eye(4);
  unsigned segment = 0;
  bool had_motion = false;
  bool tracking_lost = false;
};

Map read_map(const std::string &path) {
  std::ifstream file(path.c_str(), std::ios::binary);
  if (!file) fail("cannot open map: " + path);
  uint8_t header[24];
  file.read(reinterpret_cast<char *>(header), sizeof(header));
  if (!file || memcmp(header, "4RMP", 4) || read_le32(header + 4) != 1)
    fail("invalid rectification map header: " + path);
  Map map;
  map.width = read_le32(header + 8);
  map.height = read_le32(header + 12);
  map.source_width = read_le32(header + 16);
  map.source_height = read_le32(header + 20);
  if (!map.width || !map.height || uint64_t(map.width) * map.height > 1048576 ||
      !map.source_width || !map.source_height ||
      map.source_width > 65535 || map.source_height > 65535)
    fail("invalid rectification map dimensions: " + path);
  const size_t count = size_t(map.width) * map.height;
  std::vector<uint8_t> bytes(count * 4);
  file.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
  if (!file || file.peek() != EOF) fail("invalid rectification map payload: " + path);
  map.points.resize(count);
  for (size_t i = 0; i < count; ++i) {
    map.points[i] = {read_le16(&bytes[i * 4]), read_le16(&bytes[i * 4 + 2])};
    const Point &p = map.points[i];
    if (!((p.x == 65535 && p.y == 65535) ||
          (p.x < map.source_width && p.y < map.source_height)))
      fail("out-of-range coordinate in map: " + path);
  }
  return map;
}

void rectify(const Camera &camera, const Frame &frame, const Map &map,
             std::vector<uint8_t> &output) {
  for (size_t i = 0; i < map.points.size(); ++i) {
    const Point &p = map.points[i];
    output[i] = p.x == 65535 ? 0 : camera.pixel(frame, p.x, p.y);
  }
}

void make_dir(const std::string &path) {
  if (mkdir(path.c_str(), 0755) && errno != EEXIST)
    fail("mkdir " + path + ": " + strerror(errno));
  struct stat st;
  if (stat(path.c_str(), &st) || !S_ISDIR(st.st_mode)) fail(path + " is not a directory");
}

std::string image_path(const std::string &dir, const char *side, unsigned index) {
  char name[32];
  snprintf(name, sizeof(name), "%06u.pgm", index);
  return dir + "/" + side + "/" + name;
}

unsigned next_index(const std::string &dir) {
  for (unsigned i = 0; i < 1000000; ++i) {
    struct stat left, right;
    bool has_left = !stat(image_path(dir, "left", i).c_str(), &left);
    bool has_right = !stat(image_path(dir, "right", i).c_str(), &right);
    if (has_left != has_right) fail("incomplete existing pair in " + dir);
    if (!has_left) return i;
  }
  fail("capture directory is full");
  return 0;
}

void save_pgm(const std::string &path, const Camera &camera, const Frame &frame) {
  FILE *file = fopen(path.c_str(), "wx");
  if (!file) fail("cannot create " + path + ": " + strerror(errno));
  if (fprintf(file, "P5\n%u %u\n255\n", camera.width(), camera.height()) < 0) {
    fclose(file); fail("cannot write PGM header: " + path);
  }
  std::vector<uint8_t> row(camera.width());
  for (uint32_t y = 0; y < camera.height(); ++y) {
    for (uint32_t x = 0; x < camera.width(); ++x)
      row[x] = camera.pixel(frame, x, y);
    if (fwrite(row.data(), 1, row.size(), file) != row.size()) {
      fclose(file); fail("cannot write PGM pixels: " + path);
    }
  }
  if (fclose(file)) fail("cannot close PGM: " + path);
}

void process_pair(const Camera &left, const Frame &lf,
                  const Camera &right, const Frame &rf,
                  const Map *left_map, const Map *right_map,
                  VisualOdometryStereo *vo, PoseState *state,
                  const std::string &dir, unsigned index) {
  if (!dir.empty()) {
    save_pgm(image_path(dir, "left", index), left, lf);
    save_pgm(image_path(dir, "right", index), right, rf);
    std::string times_path = dir + "/times.csv";
    FILE *times = fopen(times_path.c_str(), "a");
    if (!times) fail("cannot append " + times_path);
    fprintf(times, "%u,%lld,%lld\n", index,
            static_cast<long long>(lf.time_us), static_cast<long long>(rf.time_us));
    if (fclose(times)) fail("cannot close " + times_path);
    fprintf(stderr, "saved pair %u; dt=%lld us\n", index,
            static_cast<long long>(rf.time_us - lf.time_us));
    return;
  }
  std::vector<uint8_t> l(left_map->points.size()), r(right_map->points.size());
  rectify(left, lf, *left_map, l);
  rectify(right, rf, *right_map, r);
  int32_t dims[3] = {int32_t(left_map->width), int32_t(left_map->height),
                     int32_t(left_map->width)};
  bool valid = vo->process(l.data(), r.data(), dims);
  if (valid) {
    Matrix motion = vo->getMotion();
    if (!motion.inv()) valid = false;
    else {
      state->pose = state->pose * motion;
      state->had_motion = true;
      state->tracking_lost = false;
    }
  }
  if (!valid && state->had_motion && !state->tracking_lost) {
    ++state->segment;
    state->pose.eye();
    state->tracking_lost = true;
  }
  printf("%u,%u,%lld,%lld,%d,%d,%s", index, state->segment,
         static_cast<long long>(lf.time_us),
         static_cast<long long>(rf.time_us - lf.time_us), vo->getNumberOfMatches(),
         vo->getNumberOfInliers(), valid ? "ok" :
         (state->had_motion ? "lost" : "initializing"));
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 4; ++col) printf(",%.9g", state->pose.val[row][col]);
  putchar('\n');
  fflush(stdout);
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && !strcmp(argv[1], "--self-test")) {
      self_test();
      return 0;
    }
    Options o = parse_options(argc, argv);
    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);

    Map lm, rm;
    VisualOdometryStereo *vo = NULL;
    PoseState pose;
    if (o.capture_dir.empty()) {
      lm = read_map(o.left_map);
      rm = read_map(o.right_map);
      if (lm.width != rm.width || lm.height != rm.height ||
          lm.source_width != o.width || rm.source_width != o.width ||
          lm.source_height != o.height || rm.source_height != o.height)
        fail("rectification maps do not match cameras or each other");
      std::ifstream calib(o.calib.c_str());
      double f, cu, cv, baseline;
      if (!(calib >> f >> cu >> cv >> baseline) || !std::isfinite(f) ||
          !std::isfinite(cu) || !std::isfinite(cv) || !std::isfinite(baseline) ||
          f <= 0 || baseline <= 0)
        fail("calib must contain measured: f cu cv baseline_m");
      VisualOdometryStereo::parameters params;
      params.calib.f = f;
      params.calib.cu = cu;
      params.calib.cv = cv;
      params.base = baseline;
      params.match.half_resolution = 0;  // maps are already downsampled
      params.match.multi_stage = 0;
      params.bucket.max_features = 2;
      vo = new VisualOdometryStereo(params);
      puts("frame,segment,left_timestamp_us,right_minus_left_us,matches,inliers,status,"
           "r00,r01,r02,tx,r10,r11,r12,ty,r20,r21,r22,tz");
    } else {
      make_dir(o.capture_dir);
      make_dir(o.capture_dir + "/left");
      make_dir(o.capture_dir + "/right");
    }

    Camera left(o.left, o), right(o.right, o);
    unsigned index = o.capture_dir.empty() ? 0 : next_index(o.capture_dir);
    unsigned processed = 0, dropped = 0;
    Frame lf, rf;
    bool have_left = false, have_right = false;
    while (!stopped && (!o.frames || processed < o.frames)) {
      pollfd fds[2] = {{left.fd(), short(have_left ? 0 : POLLIN), 0},
                       {right.fd(), short(have_right ? 0 : POLLIN), 0}};
      int ready = poll(fds, 2, 2000);
      if (ready < 0 && errno == EINTR) continue;
      if (ready < 0) fail(std::string("poll: ") + strerror(errno));
      if (!ready) { fprintf(stderr, "waiting for camera frames\n"); continue; }
      if ((fds[0].revents | fds[1].revents) & (POLLERR | POLLHUP | POLLNVAL))
        fail("camera stream error");
      if (!have_left && (fds[0].revents & POLLIN)) have_left = left.dequeue(lf);
      if (!have_right && (fds[1].revents & POLLIN)) have_right = right.dequeue(rf);
      if (!have_left || !have_right) continue;

      int64_t delta = rf.time_us - lf.time_us;
      if (llabs(delta) > int64_t(o.sync_ms) * 1000) {
        if (delta > 0) { left.queue(lf.index); have_left = false; }
        else { right.queue(rf.index); have_right = false; }
        if (++dropped % 100 == 1)
          fprintf(stderr, "discarded %u unpaired frames (latest dt=%lld us)\n",
                  dropped, static_cast<long long>(delta));
        continue;
      }
      process_pair(left, lf, right, rf, o.capture_dir.empty() ? &lm : NULL,
                   o.capture_dir.empty() ? &rm : NULL, vo, &pose, o.capture_dir, index);
      left.queue(lf.index);
      right.queue(rf.index);
      have_left = have_right = false;
      ++index;
      ++processed;
    }
    delete vo;
    fprintf(stderr, "processed %u pairs, discarded %u unpaired frames\n",
            processed, dropped);
    return 0;
  } catch (const std::exception &error) {
    fprintf(stderr, "4reel-vo: %s\n", error.what());
    usage();
    return 1;
  }
}
