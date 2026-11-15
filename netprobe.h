// netprobe.h - which operating-system connectivity probe a request is, if any.
//
// A phone checks a network for internet the moment it joins. On the soft AP every name resolves to
// the board, so those checks land on our web server. Answering them with the page (a redirect) is
// what tells the phone "captive portal, no internet", and iOS then leaves the network within
// seconds when it has cellular to fall back on. Answering each one the way a working network would
// keeps the phone connected.
//
// Pure C++ with no Arduino headers, so test/host_test.cpp checks it on a PC.
#pragma once
#include <string.h>

namespace mg {

enum ProbeKind {
  PROBE_NONE = 0,       // not a probe: send it to the page
  PROBE_APPLE,          // 200 + the exact Success document
  PROBE_204,            // 204 with an empty body
  PROBE_NCSI_TXT,       // 200 + "Microsoft NCSI"
  PROBE_CONNECTTEST     // 200 + "Microsoft Connect Test"
};

inline const char* probeBody(ProbeKind k) {
  switch (k) {
    case PROBE_APPLE: return "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    case PROBE_NCSI_TXT: return "Microsoft NCSI";
    case PROBE_CONNECTTEST: return "Microsoft Connect Test";
    default: return "";
  }
}

inline const char* probeType(ProbeKind k) { return k == PROBE_APPLE ? "text/html" : "text/plain"; }
inline int probeCode(ProbeKind k) { return k == PROBE_204 ? 204 : 200; }

inline bool startsWith(const char* s, const char* pfx) { return strncmp(s, pfx, strlen(pfx)) == 0; }
inline bool has(const char* s, const char* needle) { return strstr(s, needle) != nullptr; }

// uri and host must already be lower-cased. uri is the path only (no query).
inline ProbeKind probeKind(const char* uri, const char* host) {
  if (!uri || !host) return PROBE_NONE;
  // Apple (iOS, iPadOS, macOS): anything but the exact Success body counts as a portal.
  if (startsWith(uri, "/hotspot-detect") || startsWith(uri, "/library/test/success") ||
      has(host, "captive.apple.com") || has(host, "apple.com")) return PROBE_APPLE;
  // Android / Chrome OS.
  if (startsWith(uri, "/generate_204") || startsWith(uri, "/gen_204") ||
      has(host, "connectivitycheck") || has(host, "gstatic.com")) return PROBE_204;
  // Windows. The legacy path goes first: its host also contains "msftncsi", and the two replies are
  // not interchangeable - each expects its own exact body.
  if (startsWith(uri, "/ncsi.txt")) return PROBE_NCSI_TXT;
  if (startsWith(uri, "/connecttest.txt") || has(host, "msftconnecttest") || has(host, "msftncsi"))
    return PROBE_CONNECTTEST;
  return PROBE_NONE;
}

}  // namespace mg
