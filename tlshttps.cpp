/*
 * Secure WebSocket for the Magneto page.
 *
 * The board generates its own certificate authority and a short-lived server certificate
 * the first time it boots, and keeps them in LittleFS so a firmware update does not
 * replace the authority the phone already trusts. The server certificate names both
 * magneto.local (home Wi-Fi) and 192.168.4.1 (the board's own Wi-Fi).
 *
 * Port 443 serves one thing: wss://<board>/ws. The installed web app (GitHub Pages, https) cannot
 * use plain http or ws, so each request it sends over the socket is repeated to the existing
 * handler on port 80 and the answer comes back on the same socket. The page itself is NOT
 * served over https; the board's own copy is plain http (/ on its Wi-Fi, /app anywhere).
 */
#include "tlshttps.h"
#include "certwindow.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <esp_https_server.h>
#include <esp_system.h>
#include <string.h>
#include <strings.h>

#include "mbedtls/ecp.h"
#include "mbedtls/oid.h"
#include "mbedtls/pk.h"
#include "mbedtls/x509_crt.h"

static const char* CA_SUBJ = "CN=Magneto,O=Magneto";
static const char* SRV_SUBJ = "CN=magneto.local,O=Magneto";
static const unsigned char EKU_SERVER[] = {
  0x30, 0x0a, 0x06, 0x08, 0x2b, 0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x01
};

static httpd_handle_t https = nullptr;

// The board's own Wi-Fi address (WiFi.softAP default). The server certificate lists it so wss://192.168.4.1/ws is trusted.
static const unsigned char AP_IP4[4] = {192, 168, 4, 1};
// Written next to the certificate; a different value (new dates, or a new layout of the certificate) makes the board sign a new one.
#define CERT_LAYOUT "-ip"

static int rng(void*, unsigned char* out, size_t len) {
  esp_fill_random(out, len);
  return 0;
}

static bool writeFile(const char* path, const char* data) {
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  size_t n = strlen(data);
  size_t w = f.write((const uint8_t*)data, n);
  f.close();
  return w == n;
}

static bool readFile(const char* path, char* buf, size_t cap) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  size_t n = f.read((uint8_t*)buf, cap - 1);
  f.close();
  if (n == 0 || n >= cap - 1) return false;
  buf[n] = 0;
  return true;
}

static bool pairOk(const char* crtPath, const char* keyPath) {
  char crtPem[2048], keyPem[2048];
  if (!readFile(crtPath, crtPem, sizeof crtPem) || !readFile(keyPath, keyPem, sizeof keyPem)) return false;
  mbedtls_x509_crt crt;
  mbedtls_pk_context key;
  mbedtls_x509_crt_init(&crt);
  mbedtls_pk_init(&key);
  bool ok = mbedtls_x509_crt_parse(&crt, (const unsigned char*)crtPem, strlen(crtPem) + 1) == 0
         && mbedtls_pk_parse_key(&key, (const unsigned char*)keyPem, strlen(keyPem) + 1, nullptr, 0) == 0
         && mbedtls_pk_check_pair(&crt.pk, &key) == 0;
  mbedtls_x509_crt_free(&crt);
  mbedtls_pk_free(&key);
  return ok;
}

// SubjectAltName: the DNS name, and (when ip is not null) one IPv4 address given as four numbers.
static int sanBytes(unsigned char* out, size_t cap, const char* dns, const unsigned char* ip) {
  size_t n = strlen(dns);
  size_t total = n + 2 + (ip ? 6 : 0);
  if (n < 1 || n > 100 || cap < total + 2) return -1;
  out[0] = 0x30;
  out[1] = (unsigned char)total;
  out[2] = 0x82;
  out[3] = (unsigned char)n;
  memcpy(out + 4, dns, n);
  if (ip) {
    out[4 + n] = 0x87;
    out[5 + n] = 4;
    memcpy(out + 6 + n, ip, 4);
  }
  return (int)(total + 2);
}

static bool makeCert(mbedtls_pk_context* subjectKey, mbedtls_pk_context* issuerKey,
                     const char* subject, const char* issuer, int isCa,
                     const char* notBefore, const char* notAfter,
                     const char* dns, const unsigned char* ip, char* pemOut, size_t pemCap) {
  mbedtls_mpi serial;
  mbedtls_x509write_cert crt;
  mbedtls_mpi_init(&serial);
  mbedtls_x509write_crt_init(&crt);
  unsigned char raw[16];
  rng(nullptr, raw, sizeof raw);
  raw[0] &= 0x7f;
  if (raw[0] == 0) raw[0] = 1;
  bool ok = false;
  unsigned char san[128];
  int sanLen = 0;
  if (mbedtls_mpi_read_binary(&serial, raw, sizeof raw) != 0) goto done;
  mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
  mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
  mbedtls_x509write_crt_set_subject_key(&crt, subjectKey);
  mbedtls_x509write_crt_set_issuer_key(&crt, issuerKey);
  if (mbedtls_x509write_crt_set_subject_name(&crt, subject) != 0) goto done;
  if (mbedtls_x509write_crt_set_issuer_name(&crt, issuer) != 0) goto done;
  if (mbedtls_x509write_crt_set_serial(&crt, &serial) != 0) goto done;
  if (mbedtls_x509write_crt_set_validity(&crt, notBefore, notAfter) != 0) goto done;
  if (mbedtls_x509write_crt_set_basic_constraints(&crt, isCa, -1) != 0) goto done;
  if (mbedtls_x509write_crt_set_key_usage(&crt, isCa
        ? (MBEDTLS_X509_KU_DIGITAL_SIGNATURE | MBEDTLS_X509_KU_KEY_CERT_SIGN | MBEDTLS_X509_KU_CRL_SIGN)
        : MBEDTLS_X509_KU_DIGITAL_SIGNATURE) != 0) goto done;
  if (mbedtls_x509write_crt_set_subject_key_identifier(&crt) != 0) goto done;
  if (mbedtls_x509write_crt_set_authority_key_identifier(&crt) != 0) goto done;
  if (!isCa) {
    sanLen = sanBytes(san, sizeof san, dns, ip);
    if (sanLen < 0) goto done;
    if (mbedtls_x509write_crt_set_extension(&crt, MBEDTLS_OID_SUBJECT_ALT_NAME,
          MBEDTLS_OID_SIZE(MBEDTLS_OID_SUBJECT_ALT_NAME), 0, san, (size_t)sanLen) != 0) goto done;
    if (mbedtls_x509write_crt_set_extension(&crt, MBEDTLS_OID_EXTENDED_KEY_USAGE,
          MBEDTLS_OID_SIZE(MBEDTLS_OID_EXTENDED_KEY_USAGE), 0, EKU_SERVER, sizeof EKU_SERVER) != 0) goto done;
  }
  if (mbedtls_x509write_crt_pem(&crt, (unsigned char*)pemOut, pemCap, rng, nullptr) != 0) goto done;
  ok = true;
done:
  mbedtls_x509write_crt_free(&crt);
  mbedtls_mpi_free(&serial);
  return ok;
}

static bool makeKey(mbedtls_pk_context* key, char* pemOut, size_t pemCap) {
  mbedtls_pk_init(key);
  if (mbedtls_pk_setup(key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) != 0) return false;
  if (mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(*key), rng, nullptr) != 0) return false;
  return mbedtls_pk_write_key_pem(key, (unsigned char*)pemOut, pemCap) == 0;
}

static bool loadKey(mbedtls_pk_context* key, const char* path, char* pem, size_t cap) {
  mbedtls_pk_init(key);
  if (!readFile(path, pem, cap)) return false;
  return mbedtls_pk_parse_key(key, (const unsigned char*)pem, strlen(pem) + 1, nullptr, 0) == 0;
}

static void wipe(char* p, size_t n) { if (p) memset(p, 0, n); }

static bool issueServer(mbedtls_pk_context* caKey) {
  mbedtls_pk_context srv;
  char* srvPem = (char*)malloc(2048);
  char* keyPem = (char*)malloc(2048);
  bool ok = false;
  bool keyed = false;
  if (!srvPem || !keyPem) goto done;
  keyed = true;
  if (!makeKey(&srv, keyPem, 2048)) { Serial.println("cert: server key failed"); goto done; }
  if (!makeCert(&srv, caKey, SRV_SUBJ, CA_SUBJ, 0, CERT_NOT_BEFORE, CERT_NOT_AFTER, TLS_HOST, AP_IP4, srvPem, 2048)) {
    Serial.println("cert: server certificate failed");
    goto done;
  }
  if (!writeFile("/srv.key.tmp", keyPem) || !writeFile("/srv.crt.tmp", srvPem) || !writeFile("/srv.dates.tmp", CERT_NOT_AFTER CERT_LAYOUT)) goto done;
  LittleFS.remove("/srv.key");
  LittleFS.remove("/srv.crt");
  LittleFS.remove("/srv.dates");
  ok = LittleFS.rename("/srv.key.tmp", "/srv.key") && LittleFS.rename("/srv.crt.tmp", "/srv.crt") && LittleFS.rename("/srv.dates.tmp", "/srv.dates");
done:
  wipe(keyPem, keyPem ? 2048 : 0);
  free(srvPem);
  free(keyPem);
  if (keyed) mbedtls_pk_free(&srv);
  return ok;
}

static bool issueCa() {
  mbedtls_pk_context ca;
  char* crtPem = (char*)malloc(2048);
  char* keyPem = (char*)malloc(2048);
  bool ok = false;
  bool keyed = false;
  if (!crtPem || !keyPem) goto done;
  keyed = true;
  if (!makeKey(&ca, keyPem, 2048)) { Serial.println("cert: authority key failed"); goto done; }
  if (!makeCert(&ca, &ca, CA_SUBJ, CA_SUBJ, 1, "20200101000000", "20400101000000", nullptr, nullptr, crtPem, 2048)) {
    Serial.println("cert: authority certificate failed");
    goto done;
  }
  if (!writeFile("/ca.key.tmp", keyPem) || !writeFile("/ca.crt.tmp", crtPem)) goto done;
  LittleFS.remove("/ca.key");
  LittleFS.remove("/ca.crt");
  ok = LittleFS.rename("/ca.key.tmp", "/ca.key") && LittleFS.rename("/ca.crt.tmp", "/ca.crt");
done:
  wipe(keyPem, keyPem ? 2048 : 0);
  free(crtPem);
  free(keyPem);
  if (keyed) mbedtls_pk_free(&ca);
  return ok;
}

static void tlsPrepareBody() {
  if (!pairOk("/ca.crt", "/ca.key")) {
    Serial.println("cert: creating a new authority");
    if (!issueCa()) { Serial.println("cert: authority was not stored"); return; }
    LittleFS.remove("/srv.crt");
    LittleFS.remove("/srv.key");
    LittleFS.remove("/srv.dates");
  }
  char dates[32];
  bool datesOk = readFile("/srv.dates", dates, sizeof dates) && strcmp(dates, CERT_NOT_AFTER CERT_LAYOUT) == 0;
  if (pairOk("/srv.crt", "/srv.key") && datesOk) {
    Serial.println("cert: already on the board");
    return;
  }
  mbedtls_pk_context ca;
  char caPem[2048];
  Serial.println("cert: signing the server certificate");
  if (!loadKey(&ca, "/ca.key", caPem, sizeof caPem) || !issueServer(&ca)) Serial.println("cert: server certificate was not stored");
  else Serial.println("cert: ready");
  wipe(caPem, sizeof caPem);
  mbedtls_pk_free(&ca);
}

static volatile bool certReady = false;
static volatile bool certBusy = false;

static void certTask(void*) {
  tlsPrepareBody();
  certReady = true;
  certBusy = false;
  vTaskDelete(nullptr);
}

// Starts the work and returns at once. Signing a fresh authority takes tens of seconds on a cold
// board, and blocking setup() through it would leave the page unreachable for that whole time -
// the web server has to be answering first. The caller starts the secure socket from loop() once
// tlsCertReady() turns true.
void tlsPrepare() {
  if (certBusy || certReady) return;
  certBusy = true;
  if (xTaskCreate(certTask, "cert", 24576, nullptr, 1, nullptr) != pdPASS) {
    certBusy = false;
    Serial.println("cert: skipped, not enough memory");
  }
}

bool tlsCertReady() { return certReady; }

// ---- wss://<board>/ws: each text message is one request to the page's own API ------------------
//
//   browser -> board   "<id> <GET|POST> /api/...[ f]\n<body>"    (id: digits, chosen by the page; " f" = form-encoded body, else plain text)
//   board -> browser   "<id> <status>\n<body>"                    (the same id; body as the API returns it)
//
// The status and body are exactly what the plain http API returns, so the page uses one set of
// calls whichever way it reaches the board. Only /api/ paths pass; firmware upload (/update) does not.

#define WS_MAX_MSG 40000      // largest request or answer; a full set of saved timers is about 32 kB

static uint32_t gOpened = 0, gRefused = 0;
static const TlsJson* gJson = nullptr;
static size_t gNJson = 0;

void tlsRoutes(const TlsJson* json, size_t nJson) { gJson = json; gNJson = nJson; }

// Pages allowed to open the socket. A browser always sends Origin; programs that send none are let in.
static bool originOk(httpd_req_t* req) {
  char o[96];
  if (httpd_req_get_hdr_value_len(req, "Origin") == 0) return true;
  if (httpd_req_get_hdr_value_str(req, "Origin", o, sizeof o) != ESP_OK) return false;
  static const char* const ok[] = {"https://combsbw.github.io", "https://" TLS_HOST, "https://192.168.4.1", "http://" TLS_HOST, "http://192.168.4.1"};   /* the last two: the board's own page testing the link */
  for (const char* a : ok) if (strcmp(o, a) == 0) return true;
  return false;
}

// Our own address on whichever network the board is on (it is either on the home Wi-Fi or running its own).
static IPAddress selfIp() {
  return WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP();
}

static bool pathUsable(const char* p) {
  if (strncmp(p, "/api/", 5) != 0) return false;
  for (const char* c = p; *c; c++) if ((unsigned char)*c <= 0x20 || (unsigned char)*c >= 0x7f) return false;
  return strlen(p) < 200;
}

static bool uriIs(const char* uri, const char* want) {
  size_t n = strlen(want);
  return strncmp(uri, want, n) == 0 && (uri[n] == 0 || uri[n] == '?');
}

// Repeats the request to the page on port 80. On success *out is a malloc'd buffer holding "<id> <status>\n<body>".
static bool relay(const char* id, const char* method, const char* path, bool form, const uint8_t* body, size_t blen, uint8_t** out, size_t* outLen) {
  *out = nullptr;
  WiFiClient client;
  client.setTimeout(15000);
  if (!client.connect(selfIp(), 80)) return false;
  char head[400];
  int hlen = snprintf(head, sizeof head, "%s %s HTTP/1.1\r\nHost: " TLS_HOST "\r\nConnection: close\r\nContent-Type: %s\r\nContent-Length: %u\r\n\r\n",
                      method, path, form ? "application/x-www-form-urlencoded" : "text/plain", (unsigned)blen);
  if (hlen <= 0 || hlen >= (int)sizeof head) { client.stop(); return false; }
  if (client.write((const uint8_t*)head, hlen) != (size_t)hlen) { client.stop(); return false; }
  size_t off = 0;
  while (off < blen) {
    size_t chunk = blen - off > 1024 ? 1024 : blen - off;
    int w = client.write(body + off, chunk);
    if (w <= 0) { client.stop(); return false; }
    off += (size_t)w;
  }
  client.flush();

  size_t cap = 2048;                     /* grows as the answer arrives; most are a few hundred bytes */
  uint8_t* buf = (uint8_t*)malloc(cap);
  if (!buf) { client.stop(); return false; }
  size_t n = 0;
  uint32_t deadline = millis() + 15000;
  while ((int32_t)(millis() - deadline) < 0) {
    int a = client.available();
    if (a <= 0) {
      if (!client.connected()) break;
      delay(2);
      continue;
    }
    if (n + 1 + (size_t)a > cap) {
      size_t want = cap;
      while (n + 1 + (size_t)a > want) want *= 2;
      if (want > WS_MAX_MSG + 1024) want = WS_MAX_MSG + 1024;
      if (n + 1 >= want) { free(buf); client.stop(); return false; }   /* bigger than any answer we send */
      uint8_t* bigger = (uint8_t*)realloc(buf, want);
      if (!bigger) { free(buf); client.stop(); return false; }
      buf = bigger;
      cap = want;
    }
    int r = client.read(buf + n, cap - 1 - n);
    if (r <= 0) break;
    n += (size_t)r;
    deadline = millis() + 15000;
  }
  client.stop();
  buf[n] = 0;
  if (n < 12 || memcmp(buf, "HTTP/1.", 7) != 0) { free(buf); return false; }
  int status = atoi((const char*)buf + 9);
  const char* split = strstr((const char*)buf, "\r\n\r\n");
  if (status < 100 || status > 599 || !split) { free(buf); return false; }
  const char* b = split + 4;
  size_t bl = n - (size_t)(b - (const char*)buf);
  /* a body shorter than its Content-Length means the answer was cut off */
  for (const char* h = (const char*)buf; h < split; h++) {
    if (h[0] == '\r' && h[1] == '\n' && strncasecmp(h + 2, "content-length:", 15) == 0) {
      long want = strtol(h + 17, nullptr, 10);
      if (want >= 0 && (size_t)want != bl) {
        if ((size_t)want > bl) { free(buf); return false; }
        bl = (size_t)want;
      }
      break;
    }
  }
  size_t idLen = strlen(id);
  uint8_t* msg = (uint8_t*)malloc(idLen + 8 + bl);
  if (!msg) { free(buf); return false; }
  int p = snprintf((char*)msg, idLen + 8, "%s %d\n", id, status);
  memcpy(msg + p, b, bl);
  free(buf);
  *out = msg;
  *outLen = (size_t)p + bl;
  return true;
}

static esp_err_t sendText(httpd_req_t* req, const char* s, size_t n) {
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  f.payload = (uint8_t*)s;
  f.len = n;
  return httpd_ws_send_frame(req, &f);
}

static esp_err_t sendError(httpd_req_t* req, const char* id, int status, const char* why) {
  char m[96];
  int n = snprintf(m, sizeof m, "%s %d\n{\"ok\":false,\"err\":\"%s\"}", id, status, why);
  return sendText(req, m, (size_t)n);
}

static esp_err_t wsReq(httpd_req_t* req) {
  if (req->method == HTTP_GET) {                 /* the upgrade has just been accepted */
    if (!originOk(req)) {
      gRefused++;
      Serial.println("wss: refused a page from another site");
      httpd_sess_trigger_close(req->handle, httpd_req_to_sockfd(req));
      return ESP_FAIL;
    }
    gOpened++;
    return ESP_OK;
  }
  httpd_ws_frame_t f = {};
  f.type = HTTPD_WS_TYPE_TEXT;
  esp_err_t r = httpd_ws_recv_frame(req, &f, 0);
  if (r != ESP_OK) return r;
  if (f.len == 0) return ESP_OK;
  if (f.len > WS_MAX_MSG || !f.final || f.type != HTTPD_WS_TYPE_TEXT) {
    if (f.len <= WS_MAX_MSG) {      /* read and drop a frame we do not use, so the stream stays in step */
      uint8_t* junk = (uint8_t*)malloc(f.len + 1);
      if (junk) { f.payload = junk; httpd_ws_recv_frame(req, &f, f.len); free(junk); }
      return ESP_OK;
    }
    return ESP_FAIL;                              /* too big: close the socket */
  }
  uint8_t* msg = (uint8_t*)malloc(f.len + 1);
  if (!msg) return ESP_FAIL;
  f.payload = msg;
  r = httpd_ws_recv_frame(req, &f, f.len);
  if (r != ESP_OK) { free(msg); return r; }
  msg[f.len] = 0;

  /* "<id> <METHOD> <path>\n<body>" */
  char* line = (char*)msg;
  char* nl = strchr(line, '\n');
  const uint8_t* body = (const uint8_t*)(nl ? nl + 1 : line + f.len);
  size_t blen = (size_t)(msg + f.len - body);
  if (nl) *nl = 0;
  char* sp1 = strchr(line, ' ');
  char* sp2 = sp1 ? strchr(sp1 + 1, ' ') : nullptr;
  char id[12];
  id[0] = 0;
  if (sp1 && (size_t)(sp1 - line) > 0 && (size_t)(sp1 - line) < sizeof id) { memcpy(id, line, (size_t)(sp1 - line)); id[sp1 - line] = 0; }
  bool idOk = id[0] != 0;
  for (const char* c = id; *c; c++) if (*c < '0' || *c > '9') idOk = false;
  if (!idOk) { free(msg); return ESP_OK; }        /* cannot answer a request that has no id */
  if (!sp2) { free(msg); return sendError(req, id, 400, "bad request"); }
  *sp2 = 0;
  const char* method = sp1 + 1;
  char* path = sp2 + 1;
  char* sp3 = strchr(path, ' ');                  /* optional " f": the body is form-encoded; otherwise it is plain text (JSON) */
  bool form = false;
  if (sp3) { *sp3 = 0; form = sp3[1] == 'f'; }
  bool post = strcmp(method, "POST") == 0;
  if ((!post && strcmp(method, "GET") != 0) || !pathUsable(path)) { free(msg); return sendError(req, id, 400, "bad request"); }

  for (size_t i = 0; i < gNJson; i++) {          /* the status poll and wake are answered here, without a trip through port 80 */
    const TlsJson& j = gJson[i];
    if (j.post != (post ? 1 : 0) || !uriIs(path, j.uri)) continue;
    char out[460];
    int hn = snprintf(out, sizeof out, "%s 200\n", id);
    int n = j.fn(out + hn, sizeof out - (size_t)hn);
    free(msg);
    if (n < 0) return sendError(req, id, 500, "state");
    return sendText(req, out, (size_t)(hn + n));
  }

  uint8_t* answer = nullptr;
  size_t alen = 0;
  bool got = relay(id, method, path, form, body, blen, &answer, &alen);
  free(msg);
  if (!got) return sendError(req, id, 502, "board did not answer");
  r = sendText(req, (const char*)answer, alen);
  free(answer);
  return r;
}

// https://<board>/ in a browser: a short note, handy for checking that the certificate is trusted.
static esp_err_t noteReq(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  static const char page[] = "<!doctype html><meta name=viewport content=\"width=device-width\"><title>Magneto</title>"
    "<body style=\"font:16px system-ui;padding:24px\"><h3>Magneto</h3><p>The secure link is up and this phone trusts it.</p>"
    "<p>Open the Magneto app and choose Connect. The device's own page: <a href=\"http://" TLS_HOST "/app\">/app</a></p>";
  return httpd_resp_send(req, page, sizeof page - 1);
}

bool tlsStart() {
  if (https) return true;
  char* crt = (char*)malloc(4096);
  char* key = (char*)malloc(2048);
  char* ca = (char*)malloc(2048);
  bool started = false;
  if (!crt || !key || !ca) goto done;
  if (!readFile("/srv.crt", crt, 3000) || !readFile("/srv.key", key, 2048) || !readFile("/ca.crt", ca, 2048)) {
    Serial.println("wss: no certificate");
    goto done;
  }
  if (strlen(crt) + strlen(ca) + 2 >= 4096) goto done;
  strcat(crt, ca);   // leaf, then the authority, so the phone can build the chain

  {
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.httpd.max_open_sockets = 3;       /* each TLS connection holds about 40 kB of RAM (16 kB in, 16 kB out) */
    conf.httpd.max_uri_handlers = 4;
    conf.httpd.stack_size = 20480;
    conf.httpd.ctrl_port = 32769;
    conf.httpd.recv_wait_timeout = 5;      /* one silent connection holds up every other until this runs out */
    conf.httpd.send_wait_timeout = 5;
    conf.httpd.lru_purge_enable = true;    /* a phone that vanished without closing its socket gets its place taken */
    conf.cacert_pem = (const uint8_t*)crt;
    conf.cacert_len = strlen(crt) + 1;
    conf.prvtkey_pem = (const uint8_t*)key;
    conf.prvtkey_len = strlen(key) + 1;

    httpd_handle_t handle = nullptr;
    if (httpd_ssl_start(&handle, &conf) != ESP_OK) {
      Serial.println("wss: server did not start");
      goto done;
    }
    httpd_uri_t ws = {};
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = wsReq;
    ws.is_websocket = true;
    httpd_uri_t note = {};
    note.uri = "/";
    note.method = HTTP_GET;
    note.handler = noteReq;
    if (httpd_register_uri_handler(handle, &ws) != ESP_OK || httpd_register_uri_handler(handle, &note) != ESP_OK) {
      httpd_ssl_stop(handle);
      Serial.println("wss: route failed");
      goto done;
    }
    https = handle;
    started = true;
    Serial.println("wss: listening on 443");
  }
done:
  wipe(key, key ? 2048 : 0);
  free(crt);
  free(key);
  free(ca);
  return started;
}

bool tlsRunning() { return https != nullptr; }
void tlsStats(uint32_t* opened, uint32_t* refused) { *opened = gOpened; *refused = gRefused; }

void tlsStop() {
  if (!https) return;
  httpd_ssl_stop(https);
  https = nullptr;
}
