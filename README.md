# IoT Motion Sensor with Notification

This Python script is designed for an IoT motion sensor system that detects motion using an MPU6050 accelerometer and sends notifications to a smartphone when motion is detected. The system is controlled remotely via Google Assistant and IFTTT.

## Requirements

- ESP32 microcontroller
- MPU6050 accelerometer
- Neopixel RGB LED
- Wi-Fi network
- IFTTT account for notification
- ThingSpeak account for remote control

## Installation

1. Connect the components to the ESP32 board.
2. Update the Wi-Fi SSID and password in the `connect_wifi()` function.
3. Update the ThingSpeak API key in the `check_activate_status()` function.
4. Update the IFTTT webhook URL in the `send_noti()` function.
5. Upload the script to the ESP32 board.

## Usage

1. Power on the ESP32 board.
2. The system will connect to Wi-Fi and periodically check for motion.
3. When motion is detected, a notification will be sent via IFTTT to the configured smartphone.
4. The system can be remotely activated or deactivated via Google Assistant commands, which update the status on ThingSpeak.

## Functionality

- The script initializes the MPU6050 accelerometer and sets up periodic timers for motion detection, notification sending, and remote activation checking.
- When motion is detected, the RGB LED will turn on and a notification will be sent to the smartphone.
- The system periodically checks for remote activation commands on ThingSpeak.

## Contributing

Contributions are welcome! If you find any bugs or have suggestions for improvement, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License.
