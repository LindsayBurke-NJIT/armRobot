import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD) 

# Set pin 12 as an output, and define as servo1 as PWM pin
# You can change '12' to the BOARD number of your chosen GPIO pin
pwm_gpio = 12 
frequence = 50 # Standard frequency for analog servos
GPIO.setup(pwm_gpio, GPIO.OUT)
servo1 = GPIO.PWM(pwm_gpio, frequence) 

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)

# Function to convert angle to duty cycle percentage for the servo
def angle_to_percent(angle):
    if angle > 180 or angle < 0:
        return False
    # These values might need adjustment based on your specific servo
    start_duty = 2.5  # Corresponds to 0 degrees
    end_duty = 12.5   # Corresponds to 180 degrees
    
    # Calculate the ratio of angle to the total range (180 degrees)
    ratio = (end_duty - start_duty) / 180 
    
    # Calculate the duty cycle percentage
    duty_cycle = angle * ratio + start_duty
    return duty_cycle

servo1.ChangeDutyCycle(angle_to_percent(90))
