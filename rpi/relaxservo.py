import RPi.GPIO as GPIO
import time

# Set up GPIO:
servo_pins = {i: pin for i, pin in enumerate(range(2, 19))}  # Adjust this based on your GPIO connections
GPIO.setmode(GPIO.BCM)
for pin in servo_pins.values():
    GPIO.setup(pin, GPIO.OUT)

# Set up the servo motors:
pwms = {pin: GPIO.PWM(pin, 50) for pin in servo_pins.values()}  # 50 Hz (20 ms PWM period)
for pwm in pwms.values():
    pwm.start(0)  # Initialization

# Function to relax all servos:
def relax_all_servos():
    for pwm in pwms.values():
        pwm.ChangeDutyCycle(0)  # Stop sending the signal
    print("All servos are now relaxed.")

# Relax all servos and log the action:
try:
    with open("servo_log.txt", "a") as log_file:
        relax_all_servos()
        log_file.write(f"{time.ctime()}: All servos relaxed\n")
except KeyboardInterrupt:
    print("Program exited.")
finally:
    for pwm in pwms.values():
        pwm.stop()
    GPIO.cleanup()
