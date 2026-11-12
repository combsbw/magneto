// ghupdate.h - small pure helpers for the "update from GitHub" feature. No Arduino headers, so they are tested on a PC.
#pragma once
#include <stddef.h>
#include <string.h>

namespace mg {

#define GH_REPO_DEFAULT "combsbw/magneto"
#define GH_TAG "latest"
#define GH_ASSET "magneto.ota.bin"

// Why an online update failed. The number is also flashed on the Set LED and stored in flash for the page to show.
enum UpdErr { UE_NONE = 0, UE_WIFI = 1, UE_TIME = 2, UE_LOOKUP = 3, UE_DOWNLOAD = 4, UE_FLASH = 5 };

inline bool ghNameChar(char c, bool dot) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || (dot && c == '.');
}

// "owner/name": letters, digits, - _ . and exactly one slash.
inline bool ghRepoOk(const char* r) {
  size_t n = strlen(r), slash = 0, a = 0, b = 0;
  if (n < 3 || n > 140) return false;
  for (size_t i = 0; i < n; i++) {
    if (r[i] == '/') { slash++; continue; }
    if (!ghNameChar(r[i], true)) return false;
    if (slash == 0) a++; else b++;
  }
  return slash == 1 && a > 0 && b > 0;
}

// Personal access tokens are letters, digits, - and _ (ghp_..., github_pat_...). Empty = no token (public repo).
inline bool ghTokenOk(const char* t) {
  size_t n = strlen(t);
  if (n > 255) return false;
  for (size_t i = 0; i < n; i++) if (!ghNameChar(t[i], false)) return false;
  return true;
}

// Host part of "https://host[:port]/path" into host (cap bytes); "" when it is not an https URL.
inline void ghUrlHost(const char* url, char* host, size_t cap) {
  host[0] = 0;
  if (strncmp(url, "https://", 8) != 0) return;
  const char* s = url + 8;
  size_t n = 0;
  while (s[n] && s[n] != '/' && s[n] != ':' && s[n] != '?' && n + 1 < cap) { host[n] = s[n]; n++; }
  host[n] = 0;
}

// The token is only ever sent to GitHub's API host, never to wherever a download redirects to.
inline bool ghIsApiHost(const char* host) { return strcmp(host, "api.github.com") == 0; }

// In the release JSON from GET /repos/{repo}/releases/tags/{tag}, find the asset called `name` and copy its API URL
// (https://api.github.com/repos/.../releases/assets/ID). An asset object lists "url" before "name".
inline bool ghFindAssetUrl(const char* json, size_t n, const char* name, char* out, size_t cap) {
  size_t nl = strlen(name);
  for (size_t i = 0; i + 8 < n; i++) {
    if (memcmp(json + i, "\"name\"", 6) != 0) continue;
    size_t j = i + 6;
    while (j < n && (json[j] == ' ' || json[j] == ':')) j++;
    if (j + nl + 2 > n || json[j] != '"' || memcmp(json + j + 1, name, nl) != 0 || json[j + 1 + nl] != '"') continue;
    // walk back to the nearest "url" key before this name
    for (size_t k = i; k >= 6; k--) {
      if (memcmp(json + k - 6, "\"url\":", 6) != 0 && !(k >= 7 && memcmp(json + k - 7, "\"url\": ", 7) == 0)) continue;
      size_t p = k;
      while (p < n && (json[p] == ' ' || json[p] == ':')) p++;
      if (p >= n || json[p] != '"') return false;
      p++;
      size_t o = 0;
      while (p < n && json[p] != '"' && o + 1 < cap) out[o++] = json[p++];
      out[o] = 0;
      return strstr(out, "/releases/assets/") != nullptr && strncmp(out, "https://api.github.com/", 23) == 0;
    }
    return false;
  }
  return false;
}

}  // namespace mg
