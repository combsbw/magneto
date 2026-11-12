#pragma once
#include <stddef.h>
#include <stdint.h>

// The name on the certificate. mDNS is HOST_NAME ("magneto") plus ".local".
#define TLS_HOST "magneto.local"

// After LittleFS is up. Creates the certificate authority and server certificate on the board
// the first time, and signs a new server certificate when this build's dates or layout differ.
// Returns immediately; the work runs on its own task so the web server can start first.
void tlsPrepare();

// True once that work has finished. Call tlsStart() from loop() when it does.
bool tlsCertReady();

// Listens on 443 for wss://<board>/ws (home Wi-Fi or the board's own Wi-Fi). Call after server.begin().
// True when it is answering.
bool tlsStart();

// Requests the socket answers itself instead of repeating them to port 80 (the status poll, wake).
//   json: GET (post = 0) or POST (post = 1) of uri; fn fills buf with a JSON reply and returns its length (or < 0 for an error).
// The table must stay valid for the life of the program.
struct TlsJson { const char* uri; uint8_t post; int (*fn)(char* buf, size_t cap); };
void tlsRoutes(const TlsJson* json, size_t nJson);

// For the board's own page: whether the secure server is listening, and how many sockets it has accepted or refused (other site) since boot.
// A phone that cannot trust the certificate never gets as far as being counted.
bool tlsRunning();
void tlsStats(uint32_t* opened, uint32_t* refused);

// Frees the TLS server before an outbound HTTPS download (online update).
void tlsStop();
