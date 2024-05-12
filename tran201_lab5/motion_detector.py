from machine import SoftI2C, Pin, Timer
import network, esp32, neopixel, ujson
from time import sleep
import urequests as request

MPU_ADDR = 0x68 # I2C address of MPU
ACC_X = 0x3B # Address of value storing acceleration in X
ACC_Y = 0x3D # Address of value storing acceleration in Y
ACC_Z = 0x3F # Address of value storing acceleration in Z

# Threshold to trigger motion_detected in X,Y,Z direction
# Note: these offset are scale to G, for example, 0.1 means if the change is 0.1G m/s^2, motion will be detected
THRESHOLD_X = 0.1
THRESHOLD_Y = 0.1
THRESHOLD_Z = 0.1

GREEN = (0, 10, 0) #NEO LED COLOR
OFF = (0,0,0)

# Interval for periodic timers
MOTION_DECTED_INTERVAL = 500 # 0.5s
NOTI_SENDING_INTERVAL = 2000 # 2s
ACTIVE_CHECKING_INTERVAL = 30000 # 30s

# Global flags
active_status = False # Motion sensor activated via Google Asistant?
motion_detected = False # Motion detected?

def connect_wifi():
    login_ssid = "Apt9"
    login_pwd = "hoihoicailon"
    station_interface = network.WLAN(network.STA_IF)
    station_interface.active(True)
    station_interface.scan() 
    if not station_interface.isconnected():
        station_interface.connect(login_ssid, login_pwd)
        while not station_interface.isconnected():
            pass
    print("Connected to", login_ssid)
    print("IP Address:", station_interface.ifconfig()[0])

def mpu_init():
    # Make sure MPU is connected    
    while True:
        if MPU_ADDR == i2c.scan()[0]:
            break
        print("Cannot find MPU!!")
    # Wake up the MPU6050 since it starts in sleep mode
    i2c.start()
    i2c.writeto(MPU_ADDR, bytearray([107,0]))
    i2c.stop()
    print('Done init MPU6050.')

def bytes_to_int(data):
    # Little endian, check signing first
    if not data[0] & 0x80:
        return data[0] << 8 | data[1]
    # else 2's complement
    return -(((data[0] ^ 0xFF) << 8) | (data[1] ^ 0xFF) + 1)
    
def read_acceleration():
    i2c.start()
    acc_x = i2c.readfrom_mem(MPU_ADDR, ACC_X, 2)
    acc_y = i2c.readfrom_mem(MPU_ADDR, ACC_Y, 2)
    acc_z = i2c.readfrom_mem(MPU_ADDR, ACC_Z, 2)
    i2c.stop()

    # Calibrate, default to be 2G: 16384 according to mpu datasheet
    acc_x = bytes_to_int(acc_x) / 16384
    acc_y = bytes_to_int(acc_y) / 16384
    acc_z = bytes_to_int(acc_z) / 16384

    return acc_x, acc_y, acc_z

def detect_motion():
    global active_status, motion_detected, OFFSET_X, OFFSET_Y, OFFSET_Z

    if not active_status:
        # Motion sensor is not activated, just skip
        return
    
    if motion_detected:
        # Already detected motion in this checking interval, just skip
        return
    
    X, Y, Z = read_acceleration()
    if (abs(OFFSET_X-X) > THRESHOLD_X) or (abs(OFFSET_Y-Y) > THRESHOLD_Y) or (abs(OFFSET_Z-Z) > THRESHOLD_Z):
        motion_detected = True
        red_led.value(1)
        print("Motion detected")

def send_noti():
    global active_status, motion_detected, OFFSET_X, OFFSET_Y, OFFSET_Z
    
    if not active_status:
        # Motion sensor is not activated, just skip
        return
    
    if not motion_detected:
        # No motion detected in this interval, just skip
        return
    
    motion_detected = False # Reset so we can detect motion in the next interval
    OFFSET_X, OFFSET_Y, OFFSET_Z = read_acceleration() # Update offset

    # Trigger IFTTT applet 3 and send a notification to phone
    x = request.get("https://maker.ifttt.com/trigger/sensor_moved/with/key/ccZeAwPIbVtPN9mYk9uUzR")
    print("send noti to phone!")
    x.close()
    
def check_activate_status():
    global active_status, motion_detected, OFFSET_X, OFFSET_Y, OFFSET_Z
    # global fake_active

    msg = request.get('https://api.thingspeak.com/channels/2363554/feeds.json?api_key=5FQKE2N7CS4YBLQ6&results=2')

    if msg.json()['feeds'][-1]['field1'] == '1':
    # if fake_active:
        neo_pin[0] = GREEN
        neo_pin.write()
        OFFSET_X, OFFSET_Y, OFFSET_Z = read_acceleration() # Update offset
        active_status = True
        motion_detected = False
    # else:
    elif msg.json()['feeds'][-1]['field1'] == '0':
        red_led.value(0)
        neo_pin[0] = OFF
        neo_pin.write()
        active_status = False
        motion_detected = False

##############################################################
##############################################################
connect_wifi()

red_led = Pin(13, Pin.OUT, Pin.PULL_UP)
power_pin = Pin(2, Pin.OUT).value(1)
neo_pin = neopixel.NeoPixel(Pin(0), 1)
neo_pin[0] = OFF
neo_pin.write()

i2c = SoftI2C(scl=Pin(14), sda=Pin(22))
mpu_init()

# Read the initial value to set the OFFSET for this session
print("Begin calibration")
OFFSET_X, OFFSET_Y, OFFSET_Z = read_acceleration() #remember to reset these OFFSET when re-activated by remote commands
print(f"Done calibration\nX: {OFFSET_X*9.8}, Y: {OFFSET_Y*9.8}, Z:{OFFSET_Z*9.8}")

timer0 = Timer(0)
timer1 = Timer(1)
timer2 = Timer(2)

timer0.init(period=MOTION_DECTED_INTERVAL, mode=Timer.PERIODIC, callback=lambda t: detect_motion())
timer1.init(period=NOTI_SENDING_INTERVAL, mode=Timer.PERIODIC, callback=lambda t: send_noti())
timer2.init(period=ACTIVE_CHECKING_INTERVAL, mode=Timer.PERIODIC, callback=lambda t:check_activate_status())

# Below are for testing only, using fake_active instead of fetching data from thingspeak server
# fake_active = False
# while True:
#     sleep(1)
#     i = input("Active?")
#     if i == '1':
#         fake_active = True
#     else:
#         fake_active = False

