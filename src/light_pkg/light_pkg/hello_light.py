import os
import sys
import time
import board
import neopixel
from tkinter import Tk, Button
import threading

# Function to check if the script is running as root
def is_root():
    return os.geteuid() == 0

# If not running as root, re-execute the script with sudo
if not is_root():
    print("This script requires superuser privileges. Re-executing with sudo...")
    os.execvp('sudo', ['sudo', 'python3'] + sys.argv)

# Initialize NeoPixel strip
pixels = neopixel.NeoPixel(board.D18, 70, brightness=1.0, auto_write=False)

def blink_led(index):
    # Function to blink a specific LED for a few seconds
    for _ in range(6):  # Blink 5 times
        with threading.Lock():  # Use a lock to manage simultaneous access to pixels
            pixels.fill((0, 0, 0))  # Turn all LEDs off
            pixels.show()
        time.sleep(0.2)  # Off for 0.2 seconds
        with threading.Lock():
            if index == 0:
                pixels[0] = (255, 165, 0)  # Orange color
                pixels[1] = (255, 165, 0)
                pixels[66] = (255, 165, 0)
                pixels[65] = (255, 165, 0)

            if index == 1:
                pixels[3] = (255, 165, 0)
                pixels[4] = (255, 165, 0)
                pixels[63] = (255, 165, 0)
                pixels[64] = (255, 165, 0)
            if index == 2:
                pixels[5] = (255, 165, 0)
                pixels[6] = (255, 165, 0)
                pixels[60] = (255, 165, 0)
                pixels[61] = (255, 165, 0)

            if index == 3:
                pixels[8] = (255, 165, 0)
                pixels[9] = (255, 165, 0)
                pixels[58] = (255, 165, 0)
                pixels[59] = (255, 165, 0)
            if index == 4:
                pixels[11] = (255, 165, 0)
                pixels[12] = (255, 165, 0)
                pixels[55] = (255, 165, 0)
                pixels[54] = (255, 165, 0)
            if index == 5:
                pixels[17] = (255, 165, 0)
                pixels[18] = (255, 165, 0)
                pixels[50] = (255, 165, 0)
                pixels[51] = (255, 165, 0)

            pixels.show()  # Update the LED
        time.sleep(0.2)  # On for 0.2 seconds
    with threading.Lock():
        pixels.fill((0, 0, 0))  # Ensure all LEDs are off at the end
        pixels.show()

# Set up Tkinter GUI
root = Tk()
root.title("LED Blink Controller")

# Create a button for each LED
buttons = []
for i in range(6):
    button = Button(root, text=f"Blink LED {i+1}", command=lambda i=i: threading.Thread(target=blink_led, args=(i,)).start())
    button.pack(pady=5)
    buttons.append(button)

root.mainloop()
