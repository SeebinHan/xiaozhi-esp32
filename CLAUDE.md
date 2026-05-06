# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

This repository is the ESP-IDF C/C++ firmware for the Topsky ESP32-S3 AI voice companion robot. It handles board hardware, audio capture/playback, Opus framing, display/animation, sensors, servos, OTA, and device-side protocol logic.

The active custom board target is `Topsky Robot`, registered as `BOARD_TYPE_TOPSKY_ROBOT` with build name `topsky-robot`.

## Common commands

ESP-IDF 5.4+ is expected.

```bash
idf.py set-target esp32s3
idf.py menuconfig        # Xiaozhi Assistant -> Board Type -> Topsky Robot
idf.py build
idf.py flash monitor
```

For board release packaging, project docs use:

```bash
python scripts/release.py topsky-robot
```

Useful references:

- `main/Kconfig.projbuild` — board selection, OTA URL, language, asset options.
- `main/boards/topsky-robot/config.json` — Topsky target and sdkconfig append values.
- `docs/custom-board.md` — board-porting and build metadata flow.
- `docs/websocket.md` — device-side WebSocket protocol.

## High-level architecture

Firmware entry and coordination live in `main/`:

- `main.cc` enters the singleton `Application`.
- `application.cc` coordinates initialization, FreeRTOS event handling, audio queues, state transitions, wake/listen/speak behavior, OTA, presence/touch flows, and protocol lifecycle.
- `device_state_machine.*` defines legal transitions between device states.
- `audio/` owns codecs, audio service, wake word/VAD callbacks, Opus packet flow, and audio debugging.
- `protocols/protocol.*`, `protocols/websocket_protocol.*`, and `protocols/mqtt_protocol.*` implement transport abstractions.
- `boards/common/` contains reusable board/network/peripheral abstractions.
- `boards/topsky-robot/` contains the Topsky-specific board, camera/cat-eye display, head gimbal, tail servo, presence sensor, and touch sensor logic.
- `companion/` coordinates higher-level presence and touch behaviors across sensors, audio state, and robot actions.

`main/CMakeLists.txt` includes Topsky-specific sources such as `boards/topsky-robot/cat_eye_display.cc`, `head_gimbal.cc`, `tail_servo.cc`, presence/touch sensors, and companion coordinators.

## WebSocket protocol

The device/server protocol uses:

- WebSocket path `/xiaozhi/v1/`.
- Headers: `Authorization`, `Protocol-Version`, `Device-Id`, `Client-Id`.
- JSON control messages: `hello`, `listen`, `abort`, `tts`, `stt`, `llm`, `mcp`, IoT/tool messages.
- Binary audio: Opus frames. Protocol v1 sends raw Opus; v2 adds a 16-byte timestamp/length header; v3 adds a 4-byte lightweight header.
- Typical audio parameters: device hello advertises Opus, 16 kHz, mono, 60 ms frames; server hello commonly returns 24 kHz playback params.

Firmware implementation is in `main/protocols/websocket_protocol.cc`; protocol documentation is in `docs/websocket.md`.

## Project-specific notes

- Avoid treating `managed_components/` as first-party code unless the task specifically concerns an ESP-IDF managed component.
- Verify board-specific changes against `BOARD_TYPE_TOPSKY_ROBOT`, `main/boards/topsky-robot/`, and `main/boards/topsky-robot/config.json`.
- If changing protocol behavior, check both `main/protocols/websocket_protocol.cc` and `docs/websocket.md`.
