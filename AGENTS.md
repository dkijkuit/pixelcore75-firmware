# AGENTS.md

Firmware for the PixelCore75 ESP32-S3 LED matrix panel (Huidu WF2-compatible hardware). Despite the parent folder being named `java/`, this is a C++ PlatformIO project — no Java, no Maven/Gradle.

## Commands

PlatformIO is installed via the VSCode extension; `pio` is NOT on PATH. Use the full path:

```sh
~/.platformio/penv/bin/pio run                         # build (debug env, default)
~/.platformio/penv/bin/pio run -e pixelcore75_release  # build release
~/.platformio/penv/bin/pio run -t upload               # flash to connected board
~/.platformio/penv/bin/pio device monitor              # serial monitor (115200, from platformio.ini)
```

No tests, lint, or formatter are configured. `pio run` is the only verification step.

## Structure

- `src/main.cpp` — entire firmware (setup/loop, WiFi, MQTT, BLE provisioning, display)
- `include/images.h` — bitmap assets; `include/image_utils.h` — draw helpers + `ScreenImage` enum
- `platformio.ini` — two envs: `pixelcore75_debug` (default) and `pixelcore75_release`
- `lib/` and `test/` are empty PlatformIO boilerplate; don't add anything there by accident

## Gotchas

- Board config lives in `platformio.ini`; pin mappings are `#define`s at the top of `src/main.cpp` (board = wifiduino32s3, 8MB flash, custom `partitions_pixelcore75.csv` — app 3MB + LittleFS ~4.9MB + coredump, using the full chip; the stock `huge_app.csv` only addressed the first 4MB — USB CDC on boot — serial goes over native USB, not UART).
- The panel is driven on the X2 port pins; the X2 E-pin is unknown (`-1`), so 1/32-scan panels don't work on X2.
- Runtime config (WiFi SSID/password, MQTT server/port, brightness) persists in NVS via `Preferences` (namespace `cryptoticker`) and is provisioned over BLE GATT when no WiFi credentials are stored, or when the test button (GPIO 17) is held at boot.
- MQTT (base topic = client ID `PXCORE75-<MAC>`): base-topic payloads must be exactly 4096 bytes (64×32 RGB565, little-endian) — anything else on the base topic (including the server's zero-length retained-clear while an animation is displaying) is silently ignored, and a static frame stops any playing animation (but keeps the slot cache files). On boot/connect the retained base frame (if any) is drawn; if none was retained the last played slot animation auto-resumes — the server clears the retained frame during animation slots for exactly that reason.
- Animations: send 14 bytes to `<base>/anim/start` — `"ANIM"` magic (u32 LE) + frameCount (u16, 2–200) + delayMs (u16, ≥10) + uploadId (u32 LE) + flags (u8; bit0 = stage-only) + slot (u8, 0–31) — then frameCount messages of 4100 bytes to `<base>/anim/frame` — `"ANIF"` magic (u32 LE) + frameIdx (u16, must be sequential from 0) + 4096-byte frame. Uploads stage into `/anim_up.bin` (the file handle stays open for the whole upload — per-frame open/close plus LittleFS alloc/erase stalls starve the animation loop; a new anim/start aborts an in-flight upload cleanly) and land in the persistent slot file `/a<slot>.bin` on the last frame. Without stage-only, the last frame also plays the slot immediately; with stage-only the panel waits for a 9-byte `"ANIP"` + slot + uploadId message on `<base>/anim/play` (a play arriving mid-upload is remembered and applied on completion; a play for any slot just plays that slot's file, which is how the server's content cache skips uploads). Static frames and reboots keep slot files; a corrupt file is deleted on read failure. Space ladder on upload: replace target slot file → idle slot files → playing animation last; eviction of idle slots is invisible to the server, which self-heals via play-ack timeout + re-upload. Slot files auto-resume after reconnect. Chunked per-frame messages exist because PubSubClient's buffer is uint16-capped (~64KB), so a whole animation can't fit in one message. On staging completion AND on playback start, the panel publishes the 9-byte `"ANIL"` + slot + echoed uploadId to `<base>/anim/loaded` (from `loop()`, not the MQTT callback); the server matches the echoed uploadId so stale acks are ignored.
- `.pio/`, `.vscode/.browse.c_cpp.db*`, and `.vscode/ipch/` are generated artifacts — never edit or commit them.
