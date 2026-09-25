#ifndef FOURREEL_HTTP_H
#define FOURREEL_HTTP_H

#include <stddef.h>

typedef struct HttpRequest {
    char method[8];
    char path[256];
    char body[2049];
    size_t body_len;
    int control_header;
} HttpRequest;

typedef struct HttpResponse {
    int status;
    const char *content_type;
    const void *body;
    size_t body_len;
} HttpResponse;

/* The caller chooses the address to expose; this function has no default bind. */
int http_listen(const char *ipv4_address, unsigned port);

/* Accept and handle one client, then close its connection. */
void http_handle_client(int listener,
                        void (*handler)(const HttpRequest *, HttpResponse *, void *),
                        void *context);

#endif
