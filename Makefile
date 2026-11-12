# Magneto - Seeed XIAO ESP32-S3, built with arduino-cli
#
#   make ports      list serial ports / boards
#   make deps       install the esp32 core 2.0.17 (ArduinoOTA, WebServer, Update, DNSServer, ... ship with it)
#   make build      compile
#   make flash      compile and upload over USB
#   make ota        upload over Wi-Fi with ArduinoOTA (join the "Magneto" network first)
#
# Extras: make test (host tests of the engine), make ui (rebuild webui.h), make monitor, make ota-http, make clean
#
# Override on the command line, e.g.  make flash PORT=/dev/cu.usbmodem1101

SKETCH     := magneto
CORE       := esp32:esp32
CORE_VER   := 2.0.17
# Seeed XIAO ESP32-S3: 8 MB flash, 8 MB PSRAM, USB CDC console. The default 8 MB partition table
# already has two app slots, which OTA needs.
FQBN       ?= esp32:esp32:XIAO_ESP32S3:PSRAM=opi
ESP32_URL  := https://espressif.github.io/arduino-esp32/package_esp32_index.json

CLI        ?= arduino-cli
BUILD      := build
BIN        := $(BUILD)/$(SKETCH).ino.bin

# USB port: first ESP32-S3 that arduino-cli sees (Linux /dev/ttyACM*, macOS /dev/cu.usbmodem*, Windows COMn).
# If the board does not show up, hold BOOT while plugging it in, then run make ports again.
PORT       ?= $(shell $(CLI) board list 2>/dev/null | awk 'tolower($$0) ~ /esp32s3|esp32-s3|s3 dev/ {print $$1; exit}')

# Wi-Fi update: the soft AP is 192.168.4.1. On home Wi-Fi, pass the device address (OTA_HOST=magneto.local).
# ArduinoOTA listens on 3232 (no password).
OTA_HOST   ?= 192.168.4.1
OTA_PORT   ?= 3232
DATA_DIR   := $(shell $(CLI) config get directories.data 2>/dev/null)
ARDUINO15  := $(if $(DATA_DIR),$(DATA_DIR),$(HOME)/.arduino15)
ESPOTA     ?= $(firstword $(wildcard $(ARDUINO15)/packages/esp32/hardware/esp32/$(CORE_VER)/tools/espota.py))

.DEFAULT_GOAL := help
.PHONY: help ports deps ui pages build flash ota ota-http monitor test clean

help:
	@echo "make ports   list serial ports"
	@echo "make deps    install esp32 core $(CORE_VER)"
	@echo "make build   compile ($(FQBN))"
	@echo "make flash   build and upload over USB (PORT=$(PORT))"
	@echo "make ota     upload over Wi-Fi to $(OTA_HOST):$(OTA_PORT)  (Magneto, or the home network if the device joined it)"
	@echo "also: test, ui, pages, monitor, ota-http, clean"

ports:
	$(CLI) board list

deps:
	$(CLI) core update-index --additional-urls $(ESP32_URL)
	$(CLI) core install $(CORE)@$(CORE_VER) --additional-urls $(ESP32_URL)
	@echo "Core $(CORE_VER) installed. OTA libraries (ArduinoOTA, Update, WebServer, DNSServer, ESPmDNS, Preferences) are part of it."
	@command -v python3 >/dev/null || echo "Note: 'make ota' and 'make ui' need python3."

# The web page is compiled into the sketch as a gzipped array.
webui.h: ui/app.html tools/embed_ui.py
	python3 tools/embed_ui.py

# GitHub Pages site (docs/ on main). Relative paths, so it works at /<repo>/.
docs/index.html: ui/app.html icons/apple-touch-icon.png icons/icon-192.png tools/build_pages.py
	python3 tools/build_pages.py

pages: docs/index.html

ui: webui.h pages

# Server-certificate dates. Regenerated every build; the authority on the board stays put.
certwindow.h: tools/cert_window.py
	python3 tools/cert_window.py

build: webui.h certwindow.h
	$(CLI) compile --fqbn $(FQBN) --output-dir $(BUILD) .

flash: build
	@test -n "$(PORT)" || { echo "No ESP32-S3 port found. Plug the board in (hold BOOT while plugging if needed), run 'make ports', then 'make flash PORT=<port>'."; exit 1; }
	$(CLI) upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD) .

ota: build
	@test -n "$(ESPOTA)" || { echo "espota.py not found under $(ARDUINO15). Run 'make deps' first (or pass ESPOTA=/path/to/espota.py)."; exit 1; }
	python3 $(ESPOTA) -i $(OTA_HOST) -p $(OTA_PORT) -f $(BIN) -r

# Same update through the web page's /update endpoint (no espota needed).
ota-http: build
	curl -fS --progress-bar -F "firmware=@$(BIN)" http://$(OTA_HOST)/update && echo && echo "Updated, the board is rebooting."

monitor:
	@test -n "$(PORT)" || { echo "No port found. Run 'make ports'."; exit 1; }
	$(CLI) monitor --port $(PORT) --config baudrate=115200

# Host tests: the treatment engine and the settings parser, built with the PC's compiler.
test:
	g++ -std=c++11 -Wall -Wextra -O1 -I. test/host_test.cpp -o test/host_test
	test/host_test state

clean:
	rm -rf $(BUILD) test/host_test
