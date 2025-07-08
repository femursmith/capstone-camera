# ESP32 WebRTC Camera & Audio Recorder

This project implements a WebRTC-enabled camera and audio recorder on the ESP32-S3 platform. It supports live video streaming, audio capture, SD card recording, playback, and event-triggered uploads (e.g., via PIR sensor or sound detection).

## Features
- **Live Video Streaming**: Streams camera frames over WebRTC to a remote peer.
- **Audio Capture**: Captures audio using I2S PDM microphone and supports sound event detection.
- **SD Card Recording**: Records video and audio to SD card in AVI format.
- **Playback**: Supports playback of recorded files, including seeking by date/time.
- **Event System**: Triggers image uploads on PIR or sound events.
- **Time Synchronization**: Uses SNTP to synchronize system time.
- **WiFi Management**: Handles WiFi connection and reconnection.

## Directory Structure
- `main/` - Main application source files (camera, recorder, playback, events, WiFi manager, etc.)
- `managed_components/` - External and third-party components (WebRTC, camera driver, audio codec, etc.)
- `build/` - Build output directory

## Getting Started
### Prerequisites
- ESP32-S3 development board
- ESP-IDF v5.2.2 or compatible
- SD card (for recording/playback)
- I2S PDM microphone (for audio features)

### Build & Flash
1. Configure ESP-IDF environment:
   ```sh
   . $IDF_PATH/export.sh
   ```
2. Build the project:
   ```sh
   idf.py build
   ```
3. Flash to your ESP32-S3:
   ```sh
   idf.py -p <PORT> flash
   ```

### Configuration
- Edit `sdkconfig` or use `idf.py menuconfig` to adjust camera pins, WiFi credentials, and other settings as needed.
- SD card must be formatted and inserted before boot.

### Usage
- On boot, the device connects to WiFi, initializes the camera and audio, and synchronizes time.
- Video and audio are streamed via WebRTC when a peer connects.
- Recordings are saved to SD card and can be played back remotely.
- Event triggers (PIR, sound) cause image uploads.

## Main Components
- `app_main.c` - Application entry point, system/task initialization
- `camera.c` - Camera configuration and capture
- `recorder.cpp` - Recording logic (video/audio to SD card)
- `playback.c` - Playback and index management
- `events.c` - Event detection and image upload
- `wifimanager.c` - WiFi connection management

## Customization
- Adjust camera and audio settings in `camera.c` and `app_main.c`.
- Modify event logic in `events.c` for custom triggers.
- Extend playback/indexing in `playback.c` as needed.

## Troubleshooting
- Check serial logs for errors (e.g., SD card mount, camera init, WiFi).
- Ensure all hardware connections (camera, SD, microphone) are correct.
- Use `idf.py monitor` for runtime debugging.

## License
This project is for educational and prototyping use. See individual component licenses for third-party code.

---
