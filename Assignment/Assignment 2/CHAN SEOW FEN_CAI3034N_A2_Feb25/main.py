# Import the cyberbot library for controlling the robot
from cyberbot import *

# For coin detection
coin_found = False  # Flag to indicate if a coin has been found

# For avoiding the robot stuck in corner
counter = 0         # Counter to track if the robot is stuck in a corner
w_l = 0             # State of the left whisker (0 = pressed, 1 = not pressed)
w_r = 0             # State of the right whisker
o_w_l = 0           # Previous state of the left whisker
o_w_r = 1           # Previous state of the right whisker
"""o_w_r is initialised to 1 to ensure the corner detection logic  works correctly on the first iteration."""

# Function for basic robot movement
# Move the robot forward
def forward():
    bot(18).servo_speed(-25)   # Set left servo speed (adjust for direction)
    bot(19).servo_speed(25)    # Set right servo speed (adjust for direction)

# Move the robot backward
def backwards():
    bot(18).servo_speed(25)    # Set left servo speed for backward movement
    bot(19).servo_speed(-25)   # Set right servo speed for backward movement
    sleep(750)                 # Move backward for 750 milliseconds

# Turn the robot right
def right():
    bot(18).servo_speed(25)    # Set left servo speed for right turn
    bot(19).servo_speed(25)    # Set right servo speed for right turn
    sleep(500)                 # Turn right for 500 milliseconds

# Turn the robot left
def left():
    bot(18).servo_speed(-25)   # Set left servo speed for left turn
    bot(19).servo_speed(-25)   # Set right servo speed for left turn
    sleep(500)                 # Turn left for 500 milliseconds

# Stop the robot
def stop():
    bot(18).servo_speed(None)  # Stop left servo
    bot(19).servo_speed(None)  # Stop right servo

# Light source for the phototransistor
bot(3).write_digital(1) # Turn on the LED (P3)

while True:
    # Phototransistor setup
    bot(0).write_digital(1)     # Turn on the phototransistor (P0)
    qt = bot(0).rc_time(1)      # Read the phototransistor value

    if qt < 200:                # Check if the phototransistor detects a coin (low reading indicates a shiny surface which means coin is detected)
        coin_found = True       # Set the coin detection flag to true

    # If the robot has detected the coin
    if coin_found:              
        stop()                       # Immediately stop
        for _ in range(5):           # Blink LED and buzzer 5 times
            bot(22).tone(5000, 500)  # Buzz as coin found (5000 Hz for 500 ms) (P22)
            # Turn on and off LED for blinking to indicate coin found
            bot(20).write_digital(1) # Turn onn the LED (P20)
            sleep(100)               # LED on for 100 milliseconds 
            bot(20).write_digital(0) # Turn off the LED (P20)
            sleep(500)               # Pause for 500 milliseconds between alert cycles
    
    # Read whisker switches for robot navigation
    w_l = bot(7).read_digital() # Left whisker (P7)
    w_r = bot(9).read_digital() # Right whisker (P9)

    # Detect if stuck in a corner
    # If one whisker is pressed and the other is not
    if w_l != w_r:             
        # Check if the whisker states have changed
        if o_w_l != w_l and o_w_r != w_r:
            counter += 1     # Increment counter if the robot is likely in a corner
            # Update previous whisker states 
            o_w_l = w_l      #(Left whisker)
            o_w_r = w_r      #(Right whisker)

            # If stuck in corner too long (counter exceeds 4)
            if counter > 4:
                counter = 1  # Reset counter
               # Force whisker states to avoid immediate re-triggering
                w_l = 1      #(Left whisker)
                w_r = 1      #(Right whisker)
                backwards()  # Back out of the corner
                left()       # Turn away from the corner
                left()       # Extra turn for better escape
        else:
            counter = 1      # Reset counter if not stuck (whisker states haven't changed)
        
    # Whisker-based obstacle avoidance
    # Both whiskers hit
    if w_l == 0 and w_r == 0: 
        backwards()          # Move the robot backwards using the defined function
        right()              # Make the robot turns right using the defined function
    # Right whisker hit
    elif w_r == 0:
        backwards()          # Move the robot backwards using the defined function
        left()               # Make the robot turns left using the defined function
    # Left whisker hit
    elif w_l == 0: 
        backwards()          # Move the robot backwards using the defined function
        right()              # Make the robot turns right using the defined function
    # No obstacle
    else: 
        forward()            # Move the robot forward