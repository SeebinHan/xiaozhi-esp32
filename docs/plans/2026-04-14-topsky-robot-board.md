# Topsky Robot Board Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a new `topsky-robot` board to `xiaozhi-esp32` that supports manual Wi‑Fi/ML307 network switching, OV2640 camera input, bread-compact-wifi-lcd style simplex audio, and headless operation with `NoDisplay`.

**Architecture:** Reuse the existing dual-network architecture by basing the board on `DualNetworkBoard`, but make headless operation explicit by returning a `NoDisplay` instance instead of a real LCD. Keep the change local to the board layer plus the minimal shared fix in `DualNetworkBoard` needed to avoid dereferencing a missing display during network switch/startup UI updates. Reuse the existing camera wiring pattern from `zhengchen-cam-ml307` and the existing simplex audio path from `bread-compact-wifi-lcd`.

**Tech Stack:** ESP-IDF C++, existing board framework (`Board`, `WifiBoard`, `Ml307Board`, `DualNetworkBoard`), `Esp32Camera`, `NoAudioCodecSimplex`, CMake board selection via `CONFIG_BOARD_TYPE_*`.

---

### Task 1: Verify the shared headless seam and board registration points

**Files:**
- Inspect: `resources/project/xiaozhi-esp32/main/boards/common/dual_network_board.cc`
- Inspect: `resources/project/xiaozhi-esp32/main/application.cc`
- Modify: `resources/project/xiaozhi-esp32/main/CMakeLists.txt`

**Step 1: Confirm the display dereference points**

Read and confirm these existing headless-sensitive calls:
- `main/boards/common/dual_network_board.cc:45-52`
- `main/boards/common/dual_network_board.cc:64-70`
- `main/application.cc:65-68`
- `main/application.cc:102-153`

Expected conclusion: headless boards must never return `nullptr` from `GetDisplay()`; they must return `NoDisplay`.

**Step 2: Add the new board type mapping to CMake**

Modify `resources/project/xiaozhi-esp32/main/CMakeLists.txt` by adding a new branch in the board selection chain:

```cmake
elseif(CONFIG_BOARD_TYPE_TOPSKY_ROBOT)
    set(BOARD_TYPE "topsky-robot")
```

Place it near the other custom board mappings, not inside an unrelated manufacturer-specific branch.

**Step 3: Run a targeted grep sanity check**

Run: `grep`/search for `CONFIG_BOARD_TYPE_TOPSKY_ROBOT` in `main/CMakeLists.txt`
Expected: exactly one new branch exists.

**Step 4: Commit**

```bash
git add resources/project/xiaozhi-esp32/main/CMakeLists.txt
git commit -m "feat: register topsky robot board type"
```

### Task 2: Make `DualNetworkBoard` safe for headless boards

**Files:**
- Modify: `resources/project/xiaozhi-esp32/main/boards/common/dual_network_board.cc:44-56`
- Modify: `resources/project/xiaozhi-esp32/main/boards/common/dual_network_board.cc:63-71`
- Reference: `resources/project/xiaozhi-esp32/main/display/display.h`

**Step 1: Write the minimal code change**

Wrap the display usage in `SwitchNetworkType()` so notifications are only sent when a display object exists:

```cpp
void DualNetworkBoard::SwitchNetworkType() {
    auto display = GetDisplay();
    if (network_type_ == NetworkType::WIFI) {
        SaveNetworkTypeToSettings(NetworkType::ML307);
        if (display) {
            display->ShowNotification(Lang::Strings::SWITCH_TO_4G_NETWORK);
        }
    } else {
        SaveNetworkTypeToSettings(NetworkType::WIFI);
        if (display) {
            display->ShowNotification(Lang::Strings::SWITCH_TO_WIFI_NETWORK);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    auto& app = Application::GetInstance();
    app.Reboot();
}
```

Do the same in `StartNetwork()`:

```cpp
void DualNetworkBoard::StartNetwork() {
    auto display = Board::GetInstance().GetDisplay();

    if (display) {
        if (network_type_ == NetworkType::WIFI) {
            display->SetStatus(Lang::Strings::CONNECTING);
        } else {
            display->SetStatus(Lang::Strings::DETECTING_MODULE);
        }
    }
    current_board_->StartNetwork();
}
```

Even though the new board will return `NoDisplay`, keep these checks because they harden the shared seam with minimal risk.

**Step 2: Verify no unrelated behavior changed**

Read the edited function bodies and confirm:
- saved network type behavior is unchanged
- reboot behavior is unchanged
- current board start logic is unchanged

**Step 3: Commit**

```bash
git add resources/project/xiaozhi-esp32/main/boards/common/dual_network_board.cc
git commit -m "fix: allow dual network boards without lcd"
```

### Task 3: Create the new board config files

**Files:**
- Create: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.h`
- Create: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.json`

**Step 1: Create `config.h` with exact first-pass pin definitions**

```cpp
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

#define CAMERA_PIN_PWDN  GPIO_NUM_NC
#define CAMERA_PIN_RESET GPIO_NUM_NC
#define CAMERA_PIN_XCLK  GPIO_NUM_17
#define CAMERA_PIN_SIOD  GPIO_NUM_38
#define CAMERA_PIN_SIOC  GPIO_NUM_39
#define CAMERA_PIN_D7    GPIO_NUM_42
#define CAMERA_PIN_D6    GPIO_NUM_46
#define CAMERA_PIN_D5    GPIO_NUM_45
#define CAMERA_PIN_D4    GPIO_NUM_20
#define CAMERA_PIN_D3    GPIO_NUM_19
#define CAMERA_PIN_D2    GPIO_NUM_9
#define CAMERA_PIN_D1    GPIO_NUM_18
#define CAMERA_PIN_D0    GPIO_NUM_48
#define CAMERA_PIN_VSYNC GPIO_NUM_40
#define CAMERA_PIN_HREF  GPIO_NUM_21
#define CAMERA_PIN_PCLK  GPIO_NUM_41
#define XCLK_FREQ_HZ     24000000

#define ML307_TX_PIN GPIO_NUM_11
#define ML307_RX_PIN GPIO_NUM_12
#define ML307_DTR_PIN GPIO_NUM_NC

#endif // _BOARD_CONFIG_H_
```

Notes for execution:
- `GPIO17` is chosen for `CAMERA_PIN_XCLK` because the pin table marks it unused; verify this against the hardware document before finalizing.
- Keep optional/unknown control pins as `GPIO_NUM_NC` unless the hardware document proves otherwise.

**Step 2: Create `config.json`**

```json
{
  "target": "esp32s3",
  "builds": [
    {
      "name": "topsky-robot",
      "sdkconfig_append": [
        "CONFIG_USE_DEVICE_AEC=y",
        "CONFIG_LWIP_TCPIP_RECVMBOX_SIZE=48",
        "CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32",
        "CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=n",
        "CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1=y"
      ]
    }
  ]
}
```

Do not copy LCD-related sdkconfig options into this headless board.

**Step 3: Verify file names and board name alignment**

Confirm all three match exactly:
- folder: `topsky-robot`
- build name: `topsky-robot`
- CMake `BOARD_TYPE`: `topsky-robot`

**Step 4: Commit**

```bash
git add resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.h resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.json
git commit -m "feat: add topsky robot board configuration"
```

### Task 4: Implement the headless dual-network board class

**Files:**
- Create: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/topsky_robot_board.cc`
- Reference: `resources/project/xiaozhi-esp32/main/boards/zhengchen-cam-ml307/zhengchen_cam_board_ml307.cc`
- Reference: `resources/project/xiaozhi-esp32/main/boards/bread-compact-wifi-lcd/compact_wifi_board_lcd.cc`
- Reference: `resources/project/xiaozhi-esp32/main/display/display.h`

**Step 1: Write the initial board class**

Create this first-pass implementation:

```cpp
#include "dual_network_board.h"
#include "audio/codecs/no_audio_codec.h"
#include "application.h"
#include "config.h"
#include "display/display.h"
#include "esp32_camera.h"
#include "led/single_led.h"

#include <esp_log.h>

#define TAG "TopskyRobotBoard"

class TopskyRobotBoard : public DualNetworkBoard {
private:
    Esp32Camera* camera_ = nullptr;

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = I2C_NUM_1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

public:
    TopskyRobotBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN) {
        InitializeCamera();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual Display* GetDisplay() override {
        static NoDisplay display;
        return &display;
    }

    virtual Led* GetLed() override {
        if (BUILTIN_LED_GPIO != GPIO_NUM_NC) {
            static SingleLed led(BUILTIN_LED_GPIO);
            return &led;
        }
        return DualNetworkBoard::GetLed();
    }
};

DECLARE_BOARD(TopskyRobotBoard);
```

**Step 2: Verify the includes and symbols compile conceptually**

Check that:
- `NoAudioCodecSimplex` comes from `audio/codecs/no_audio_codec.h`
- `NoDisplay` comes from `display/display.h`
- `Esp32Camera` comes from `esp32_camera.h`
- `DualNetworkBoard::GetLed()` is a valid fallback through `Board`

If `I2C_NUM_1` or LEDC symbols need explicit includes, add only the missing headers.

**Step 3: Keep the board intentionally minimal**

Do not add:
- display setup
- button behavior
- PCA9685 setup
- LD2410B setup
- pressure sensor logic
- custom Wi‑Fi/4G switch UI logic

Those belong in later tasks after the board boots and the shared seam is proven.

**Step 4: Commit**

```bash
git add resources/project/xiaozhi-esp32/main/boards/topsky-robot/topsky_robot_board.cc
git commit -m "feat: add initial topsky robot board"
```

### Task 5: Verify the new board is wired into the build metadata

**Files:**
- Inspect: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.json`
- Inspect: `resources/project/xiaozhi-esp32/main/CMakeLists.txt`
- Optional inspect: `resources/project/xiaozhi-esp32/docs/custom-board.md`

**Step 1: Confirm board naming consistency**

Verify the following literal strings all match `topsky-robot`:
- folder name
- `config.json` build name
- CMake `BOARD_TYPE`

**Step 2: Confirm headless design choice**

Verify `topsky_robot_board.cc` returns `NoDisplay` and does not initialize any LCD/backlight code.

**Step 3: If build tooling is available, run targeted board packaging**

Run:

```bash
cd resources/project/xiaozhi-esp32 && python scripts/release.py topsky-robot
```

Expected:
- board config is discovered
- sdkconfig generation includes `BOARD_TYPE=topsky-robot`
- compile may fail on unresolved hardware-specific details; if it does, fix only the concrete compile errors

If build tooling is not available in the environment, explicitly record that the user will run build verification.

**Step 4: Commit**

```bash
git add resources/project/xiaozhi-esp32/main/CMakeLists.txt \
        resources/project/xiaozhi-esp32/main/boards/common/dual_network_board.cc \
        resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.h \
        resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.json \
        resources/project/xiaozhi-esp32/main/boards/topsky-robot/topsky_robot_board.cc
git commit -m "feat: add headless topsky robot dual-network board"
```

### Task 6: Post-build validation and next-scope notes

**Files:**
- Inspect: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/topsky_robot_board.cc`
- Inspect: `resources/project/xiaozhi-esp32/main/boards/topsky-robot/config.h`

**Step 1: Validate against current agreed scope**

Confirm the first version includes only:
- manual Wi‑Fi/4G switching through `DualNetworkBoard`
- camera initialization
- simplex audio initialization
- headless `NoDisplay`

Confirm it intentionally excludes:
- display support
- button behavior
- PCA9685
- LD2410B
- ADC pressure input

**Step 2: Record explicit follow-up items for later, not now**

Future follow-up tasks after the first build succeeds:
1. verify and correct `CAMERA_PIN_XCLK`
2. add board-specific manual switch trigger if no boot-button flow exists
3. add PCA9685 initialization if servo behavior is required
4. add LD2410B integration if presence detection is required
5. add ADC pressure sensor handling if squeeze input is required

**Step 3: Do not expand scope during implementation**

If the code compiles and the board boots, stop. Do not add “nice to have” board features in the same pass.
