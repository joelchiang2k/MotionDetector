Demo video: https://youtu.be/x5O_w8GbAbU

MPU6050 is connected via I2C pin 14,22
Every 0.5s, checks for motion detection
Every 2s, if motion is detected during last interval -> send noti to phone
Every 30s, check activation status on thinkspeak server