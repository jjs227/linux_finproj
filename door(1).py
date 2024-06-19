import RPi.GPIO as GPIO
import time
import board
import neopixel
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from gpiozero import Button
from gpiozero import PWMOutputDevice

button_pin = 24
buzzer_pin = 12
music_on = False

button = Button(button_pin, pull_up=False)
factory = PiGPIOFactory()
servo = Servo(17, pin_factory=factory)

TRIG = 13
ECHO = 19
LED_PIN = board.D10
NUM_PIXELS = 4
BUTTON = 24

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(buzzer_pin, GPIO.OUT)

pixels = neopixel.NeoPixel(LED_PIN, NUM_PIXELS, brightness=0.2, auto_write=False, pixel_order=neopixel.GRB)
pixels.fill((255, 255, 255))
pixels.show()

tones = [659, 622, 659, 622, 659, 494, 587, 523, 440]
music = [0, 1, 0, 1, 0, 5, 6, 7, 8]
term = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.8]

def button_pressed():
    global music_on
    music_on = not music_on
    print(f"Button Pressed! Music_On = {music_on}")

button.when_pressed = button_pressed

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

def open_door():
    servo.min()
    pixels.fill((0, 255, 0))
    pixels.show()
    time.sleep(3)
    servo.max()
    pixels.fill((255, 255, 255))
    pixels.show()

def stop_mode():
    pwm_device = PWMOutputDevice(pin=buzzer_pin, frequency=100, initial_value=0.5)
    servo.min()
    pixels.fill((255, 0, 0))
    pixels.show()

    while music_on:
        print("Music on !!")
        for i in range(len(music)):
            if not music_on:
                print("Music off !!")
                break
            pwm_device.frequency = tones[music[i]]
            pwm_device.value = 0.5
            sleep(term[i])
            pwm_device.value = 0

    pwm_device.off()
    pixels.fill((255, 255, 255))
    pixels.show()
    servo.max()

try:
    while True:
        if music_on:
            stop_mode()
        else:
            distance = get_distance()
            print(f"Distance: {distance} cm")
            if distance <= 10:
                open_door()
            else:
                pixels.fill((255, 255, 255))
                pixels.show()
        time.sleep(0.5)

except KeyboardInterrupt:
    print("stop")
    pixels.fill((0, 0, 0))
    pixels.show()
    servo.max()

