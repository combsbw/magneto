// json.h - a tiny streaming JSON reader (no allocation, no dependencies).
//
// jsonWalk() parses a JSON document and calls fn() for every number, boolean and string it finds, passing the dotted
// path of the value ("profiles.load.colors.0", "pins.timer.r"). Objects and arrays are walked, null is skipped.
// Returns false if the document is not valid JSON. Plain C++, so it builds on a PC too.
#pragma once
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

namespace mg {

// type: 'n' number, 'b' boolean (num = 0 or 1), 's' string (str)
typedef void (*JsonLeaf)(void* ctx, const char* path, char type, double num, const char* str);

struct JsonWalker {
  const char* s;
  size_t n, i;
  char path[96];
  int plen;
  JsonLeaf fn;
  void* ctx;
};

inline void jwSkip(JsonWalker& w) {
  while (w.i < w.n && (w.s[w.i] == ' ' || w.s[w.i] == '\t' || w.s[w.i] == '\n' || w.s[w.i] == '\r')) w.i++;
}

// Reads a quoted string into out (truncated to cap-1 characters). \uXXXX becomes '?'.
inline bool jwString(JsonWalker& w, char* out, size_t cap) {
  if (w.i >= w.n || w.s[w.i] != '"') return false;
  w.i++;
  size_t o = 0;
  while (w.i < w.n) {
    char ch = w.s[w.i++];
    if (ch == '"') { out[o] = 0; return true; }
    if (ch == '\\') {
      if (w.i >= w.n) return false;
      char e = w.s[w.i++];
      if (e == 'u') { if (w.i + 4 > w.n) return false; w.i += 4; ch = '?'; }
      else if (e == 'n') ch = '\n';
      else if (e == 't') ch = '\t';
      else if (e == 'r') ch = '\r';
      else if (e == 'b') ch = '\b';
      else if (e == 'f') ch = '\f';
      else ch = e;                       // \" \\ \/
    }
    if (o + 1 < cap) out[o++] = ch;
  }
  return false;
}

inline bool jwPush(JsonWalker& w, const char* key) {
  size_t k = strlen(key);
  if ((size_t)w.plen + k + 2 >= sizeof w.path) return false;
  if (w.plen > 0) w.path[w.plen++] = '.';
  memcpy(w.path + w.plen, key, k);
  w.plen += (int)k;
  w.path[w.plen] = 0;
  return true;
}

inline bool jwValue(JsonWalker& w, int depth) {
  if (depth > 8) return false;
  jwSkip(w);
  if (w.i >= w.n) return false;
  char ch = w.s[w.i];

  if (ch == '{') {
    w.i++;
    jwSkip(w);
    if (w.i < w.n && w.s[w.i] == '}') { w.i++; return true; }
    for (;;) {
      jwSkip(w);
      char key[28];
      if (!jwString(w, key, sizeof key)) return false;
      jwSkip(w);
      if (w.i >= w.n || w.s[w.i] != ':') return false;
      w.i++;
      int save = w.plen;
      if (!jwPush(w, key)) return false;
      if (!jwValue(w, depth + 1)) return false;
      w.plen = save; w.path[save] = 0;
      jwSkip(w);
      if (w.i >= w.n) return false;
      if (w.s[w.i] == ',') { w.i++; continue; }
      if (w.s[w.i] == '}') { w.i++; return true; }
      return false;
    }
  }

  if (ch == '[') {
    w.i++;
    jwSkip(w);
    if (w.i < w.n && w.s[w.i] == ']') { w.i++; return true; }
    for (int idx = 0;; idx++) {
      char key[8];
      key[0] = 0;
      int v = idx, d = 0;
      char tmp[8];
      do { tmp[d++] = (char)('0' + v % 10); v /= 10; } while (v > 0 && d < 6);
      for (int k = 0; k < d; k++) key[k] = tmp[d - 1 - k];
      key[d] = 0;
      int save = w.plen;
      if (!jwPush(w, key)) return false;
      if (!jwValue(w, depth + 1)) return false;
      w.plen = save; w.path[save] = 0;
      jwSkip(w);
      if (w.i >= w.n) return false;
      if (w.s[w.i] == ',') { w.i++; continue; }
      if (w.s[w.i] == ']') { w.i++; return true; }
      return false;
    }
  }

  if (ch == '"') {
    char sv[40];
    if (!jwString(w, sv, sizeof sv)) return false;
    w.fn(w.ctx, w.path, 's', 0, sv);
    return true;
  }

  if (w.n - w.i >= 4 && memcmp(w.s + w.i, "true", 4) == 0) { w.i += 4; w.fn(w.ctx, w.path, 'b', 1, ""); return true; }
  if (w.n - w.i >= 5 && memcmp(w.s + w.i, "false", 5) == 0) { w.i += 5; w.fn(w.ctx, w.path, 'b', 0, ""); return true; }
  if (w.n - w.i >= 4 && memcmp(w.s + w.i, "null", 4) == 0) { w.i += 4; return true; }

  // number
  char buf[32];
  size_t b = 0;
  while (w.i < w.n && b < sizeof buf - 1) {
    char c = w.s[w.i];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') { buf[b++] = c; w.i++; }
    else break;
  }
  if (b == 0) return false;
  buf[b] = 0;
  char* endp = nullptr;
  double v = strtod(buf, &endp);
  if (endp == buf || *endp != 0) return false;
  w.fn(w.ctx, w.path, 'n', v, "");
  return true;
}

inline bool jsonWalk(const char* s, size_t n, JsonLeaf fn, void* ctx) {
  JsonWalker w;
  w.s = s; w.n = n; w.i = 0; w.plen = 0; w.path[0] = 0; w.fn = fn; w.ctx = ctx;
  if (!jwValue(w, 0)) return false;
  jwSkip(w);
  return w.i == w.n;
}

}  // namespace mg
