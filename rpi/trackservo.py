import RPi.GPIO as GPIO
import time

# Set up GPIO:
servo_pin = 0  # Adjust this based on your GPIO connection
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Set up the servo motor:
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms PWM period)
pwm.start(0)  # Initialization

# Function to set servo angle:
def set_servo_angle(angle):
    duty_cycle = angle / 18.0 + 2
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)
    pwm.ChangeDutyCycle(0)  # Stop sending signal to avoid jittering
# Function to relax the servo:
def relax_servo():
    pwm.ChangeDutyCycle(0)  # Stop sending the signal

# Main loop to log movements and allow relaxation:
try:
    with open("servo_log.txt", "a") as log_file:
        while True:
            command = input("Enter servo angle (0 to 180) or 'relax': ")
            if command.lower() == 'relax':
                relax_servo()
                log_file.write(f"{time.ctime()}: Servo relaxed\n")
                print("Servo is now relaxed.")
            else:
                try:
                    angle = float(command)
                    if 0 <= angle <= 180:
                        set_servo_angle(angle)
                        log_file.write(f"{time.ctime()}: Moved to {angle} degrees\n")
                        log_file.flush()  # Ensure data is written to the file
                    else:
                        print("Invalid angle. Please enter a value between 0 and 180.")
                except ValueError:
                    print("Invalid input. Enter a number between 0 to 180 or type 'relax'.")
except KeyboardInterrupt:
    print("Program exited.")
finally:
    pwm.stop()
    GPIO.cleanup()
