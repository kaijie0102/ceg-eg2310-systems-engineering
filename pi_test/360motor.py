import time
import RPi.GPIO as GPIO

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose an appropriate pwm channel to be used to control the servo
"""
This spins the motor 360degrees. Setting it to 80 will stop the motor
"""

servo_pin = 17

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as it's starting position
p.start(7.5)

try:
        while True:
                angle = int(input("Angle:"))
                if angle<0 or angle>180:
                        print("Please input an angle between 0 and 180.")
                duty_cycle = 2.5 + (angle/180.00) * 10.0
                p.ChangeDutyCycle(duty_cycle)
                time.sleep(1) #delay 1 second

except KeyboardInterrupt:
        p.stop()        
        GPIO.cleanup()
