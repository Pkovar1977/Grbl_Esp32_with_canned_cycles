## Copilot / AI agent instructions for Grbl_Esp32

This file contains concise, repository-specific guidance to help an AI coding agent be productive.

- **Project purpose:** Firmware for ESP32-based GRBL (motion controller) with an optional WebUI.
- **Primary source:** Grbl_Esp32 folder; entry: Grbl_Esp32/Grbl_Esp32.ino and core sources under Grbl_Esp32/src.

- **Build system:** PlatformIO. Key config: platformio.ini (default_envs = release). Typical commands:
  - Build release: `platformio run -e release`
  - Build debug: `platformio run -e debug`
  - On Windows, helper scripts: build-all.ps1 and build.bat in embedded/ for packaged flows.

- **Important files to read first:**
  - platformio.ini — board, framework, build_flags, src_filter, library deps.
  - Grbl_Esp32/src/Config.h and Defaults.h — device and compile-time configuration.
  - Grbl_Esp32/src/Planner.cpp, MotionControl.cpp, Stepper.cpp — motion stack.
  - Grbl_Esp32/src/Pins.cpp and CPUMap.h — hardware abstraction.

- **Machine-specific configuration:** Machine definitions live in Grbl_Esp32/Machines/*.h. The active machine header is selected via -DMACHINE_FILENAME=... build flag (see commented example in platformio.ini). To add or change a machine, add a header there and update the build flag.

- **Web UI and embedded assets:** embedded/www contains the WebUI assets (tool.html, script.js). The firmware serves these; check embedded/header.txt / footer.txt and embedded/gulpfile.js for build-time packaging.

- **Libraries & dependencies:** libraries/ contains vendored libs. platformio.ini declares external deps (e.g., TMCStepper, SSD1306 driver). Keep lib_dir = libraries in mind when adding libs.

- **Coding patterns & conventions:**
  - C++ style: header .h + implementation .cpp pairs in Grbl_Esp32/src.
  - Global/static hardware abstractions (Pins, Machine) are used instead of dependency injection.
  - State and settings persist via Settings.cpp/SettingsDefinitions.cpp and Preferences/EEPROM abstractions.
  - Follow CodingStyle.md when modifying formatting or public APIs.

- **Where to change behavior safely:** Prefer hooks and modules (e.g., Custom/ examples) rather than editing core timing code in Planner.cpp or Stepper.cpp. Small UI or feature additions should live in Custom/ or separate files and included by build flags.

- **Debugging tips:**
  - Serial monitor speed: see platformio.ini monitor_speed (115200) and monitor_flags (esp32_exception_decoder).
  - Use build_type = debug environment in platformio.ini to enable symbols and assertions.
  - There are helper docs: VisualStudio.md for native debugging and debug.ini for config.

- **Tests & examples:** Look under Grbl_Esp32/tests and libraries/*/tests for unit and integration tests. Repo also contains example machine configs in Grbl_Esp32/Custom.

- **When modifying build flags:** Update platformio.ini or the helper scripts. Keep common flags in the [common] section to avoid duplication.

- **Specific code patterns to search for when making changes:**
  - MACHINE_FILENAME (selects machine header)
  - Settings.read() / Settings.write() (persistence flow)
  - serial, Protocol and Report files for communication parsing/formatting

- **What NOT to change lightly:** Low-level timing/planner/stepper logic (Planner.cpp, Stepper.cpp, MotionControl.cpp) — these are sensitive to small changes and affect motion safety.

- **Contact / feedback:** If anything is unclear or you want more detail on a specific area (build/test flow, adding machines, or WebUI integration), tell me which part and I will expand or iterate.
