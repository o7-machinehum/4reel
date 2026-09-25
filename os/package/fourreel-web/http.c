#include "http.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define HEADER_LIMIT 4096
#define BODY_LIMIT 2048

static int send_all(int fd, const void *data, size_t length)
{
    const char *bytes = data;
    while (length) {
        ssize_t sent = send(fd, bytes, length, MSG_NOSIGNAL);
        if (sent < 0 && errno == EINTR)
            continue;
        if (sent <= 0)
            return -1;
        bytes += sent;
        length -= (size_t)sent;
    }
    return 0;
}

static const char *reason_phrase(int status)
{
    switch (status) {
    case 200: return "OK";
    case 201: return "Created";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 409: return "Conflict";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default: return "Response";
    }
}

static void send_response(int fd, const HttpResponse *response)
{
    char header[512];
    const char *type = response->content_type ? response->content_type : "text/plain";
    size_t body_len = response->body ? response->body_len : 0;
    int length = snprintf(header, sizeof(header),
                          "HTTP/1.1 %d %s\r\n"
                          "Content-Type: %s\r\n"
                          "Content-Length: %zu\r\n"
                          "Cache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n",
                          response->status, reason_phrase(response->status), type,
                          body_len);
    if (length < 0 || (size_t)length >= sizeof(header))
        return;
    if (send_all(fd, header, (size_t)length) == 0 && body_len)
        send_all(fd, response->body, body_len);
}

static void send_error(int fd, int status)
{
    const char *message = reason_phrase(status);
    HttpResponse response = {
        .status = status,
        .content_type = "text/plain; charset=utf-8",
        .body = message,
        .body_len = strlen(message),
    };
    send_response(fd, &response);
}

static int parse_content_length(const char *value, size_t *length)
{
    size_t result = 0;

    while (*value == ' ' || *value == '\t')
        value++;
    if (*value < '0' || *value > '9')
        return 400;
    while (*value >= '0' && *value <= '9') {
        size_t digit = (size_t)(*value++ - '0');
        if (result > (BODY_LIMIT - digit) / 10)
            return 413;
        result = result * 10 + digit;
    }
    while (*value == ' ' || *value == '\t')
        value++;
    if (*value)
        return 400;
    *length = result;
    return 0;
}

static int parse_header(char *header, HttpRequest *request, size_t *content_length)
{
    char *line_end = strstr(header, "\r\n");
    char *first_space;
    char *second_space;
    char *cursor;
    int saw_length = 0;
    int saw_control = 0;

    if (!line_end)
        return 400;
    *line_end = '\0';
    first_space = strchr(header, ' ');
    if (!first_space)
        return 400;
    second_space = strchr(first_space + 1, ' ');
    if (!second_space || strchr(second_space + 1, ' '))
        return 400;
    *first_space = '\0';
    *second_space = '\0';
    if (strcmp(header, "GET") && strcmp(header, "POST"))
        return 405;
    if (strcmp(second_space + 1, "HTTP/1.0") &&
        strcmp(second_space + 1, "HTTP/1.1"))
        return 400;
    if (first_space[1] != '/')
        return 400;

    char *query = strchr(first_space + 1, '?');
    size_t path_len = query ? (size_t)(query - (first_space + 1))
                            : strlen(first_space + 1);
    if (path_len >= sizeof(request->path))
        return 413;
    memcpy(request->method, header, strlen(header) + 1);
    memcpy(request->path, first_space + 1, path_len);
    request->path[path_len] = '\0';

    cursor = line_end + 2;
    while (*cursor) {
        char *next = strstr(cursor, "\r\n");
        char *colon;
        if (!next)
            return 400;
        if (next == cursor)
            break;
        *next = '\0';
        colon = strchr(cursor, ':');
        if (!colon || colon == cursor)
            return 400;
        *colon++ = '\0';
        if (strpbrk(cursor, " \t"))
            return 400;
        if (!strcasecmp(cursor, "Content-Length")) {
            int status;
            if (saw_length++)
                return 400;
            status = parse_content_length(colon, content_length);
            if (status)
                return status;
        } else if (!strcasecmp(cursor, "Transfer-Encoding")) {
            /* Request framing is deliberately limited to Content-Length. */
            return 400;
        } else if (!strcasecmp(cursor, "X-4reel-control")) {
            if (saw_control++)
                return 400;
            while (*colon == ' ' || *colon == '\t')
                colon++;
            if (strcmp(colon, "1"))
                return 400;
            request->control_header = 1;
        }
        cursor = next + 2;
    }
    return 0;
}

static int read_request(int fd, HttpRequest *request)
{
    char input[HEADER_LIMIT];
    char header[HEADER_LIMIT + 1];
    size_t used = 0;
    size_t header_end = 0;
    size_t content_length = 0;
    int status;

    while (!header_end) {
        ssize_t got;
        if (used == sizeof(input))
            return 413;
        got = recv(fd, input + used, sizeof(input) - used, 0);
        if (got < 0 && errno == EINTR)
            continue;
        if (got <= 0)
            return 400;
        used += (size_t)got;
        for (size_t i = 3; i < used; i++) {
            if (!memcmp(input + i - 3, "\r\n\r\n", 4)) {
                header_end = i + 1;
                break;
            }
        }
    }

    if (memchr(input, '\0', header_end))
        return 400;
    memcpy(header, input, header_end);
    header[header_end] = '\0';
    status = parse_header(header, request, &content_length);
    if (status)
        return status;

    request->body_len = content_length;
    size_t prefetched = used - header_end;
    size_t copied = prefetched < content_length ? prefetched : content_length;
    memcpy(request->body, input + header_end, copied);
    while (copied < content_length) {
        ssize_t got = recv(fd, request->body + copied, content_length - copied, 0);
        if (got < 0 && errno == EINTR)
            continue;
        if (got <= 0)
            return 400;
        copied += (size_t)got;
    }
    request->body[content_length] = '\0';
    return 0;
}

int http_listen(const char *ipv4_address, unsigned port)
{
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons((uint16_t)port),
    };
    int listener;
    int reuse = 1;

    if (!ipv4_address || port > 65535 ||
        inet_pton(AF_INET, ipv4_address, &address.sin_addr) != 1) {
        errno = EINVAL;
        return -1;
    }
    listener = socket(AF_INET, SOCK_STREAM, 0);
    if (listener < 0)
        return -1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(listener, 8) < 0) {
        int error = errno;
        close(listener);
        errno = error;
        return -1;
    }
    return listener;
}

void http_handle_client(int listener,
                        void (*handler)(const HttpRequest *, HttpResponse *, void *),
                        void *context)
{
    int client = accept(listener, NULL, NULL);
    struct timeval timeout = {.tv_sec = 1};
    HttpRequest request = {0};
    HttpResponse response = {
        .status = 500,
        .content_type = "text/plain; charset=utf-8",
        .body = "Internal Server Error",
        .body_len = sizeof("Internal Server Error") - 1,
    };
    int status;

    if (client < 0)
        return;
    if (setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0 ||
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
        close(client);
        return;
    }
    status = read_request(client, &request);
    if (status)
        send_error(client, status);
    else {
        if (handler)
            handler(&request, &response, context);
        send_response(client, &response);
    }
    close(client);
}
