import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import standard string message
import threading
import time
import board
import neopixel
from tkinter import Tk, Button

if os.geteuid() != 0:
    print("Script requires root privileges. Re-running with sudo...")
    sudo_command = f'sudo bash -c "cd {os.getcwd()} && . install/setup.bash && ros2 run light_pkg light_node"'
    os.system(sudo_command)
    sys.exit(0)

class LEDController(Node):
    def __init__(self):
        super().__init__('led_gui')
        self.pixels = neopixel.NeoPixel(board.D18, 70, brightness=0.2, auto_write=False)
        self.publisher_ = self.create_publisher(String, 'led_status', 10)  # Create a publisher
        # Start the Tkinter GUI in another thread to keep the ROS node responsive
        threading.Thread(target=self.start_gui, daemon=True).start()



    

    def start_gui(self):
        self.root = Tk()
        self.root.title("LED Blink Controller")
        # Create a button for each LED
        for i in range(6):
            button = Button(self.root, text=f"Blink LED {i+1}", command=lambda i=i: threading.Thread(target=self.blink_led, args=(i,)).start())
            button.pack(pady=5)
        
        # Add buttons for non-urgent and urgent errors
        non_urgent_button = Button(self.root, text="Non-Urgent Error", command=lambda: threading.Thread(target=self.set_error, args=('non-urgent',)).start())
        non_urgent_button.pack(pady=5)

        urgent_button = Button(self.root, text="Urgent Error", command=lambda: threading.Thread(target=self.set_error, args=('urgent',)).start())
        urgent_button.pack(pady=5)

        self.root.mainloop()

    def blink_led(self, index):
        # Function to blink a specific LED for a few seconds
        for _ in range(6):  # Blink 6 times
            with threading.Lock():  # Use a lock to manage simultaneous access to pixels
                self.pixels.fill((0, 0, 0))  # Turn all LEDs off
                self.pixels.show()
            time.sleep(0.2)  # Off for 0.2 seconds
            with threading.Lock():
                if index == 0:
                    self.pixels[0] = (255, 165, 0)  # Orange color
                    self.pixels[1] = (255, 165, 0) 
                    self.pixels[66]= (255, 165, 0)
                    self.pixels[65]= (255, 165, 0)
                    
                elif index == 1:
                    self.pixels[3] = (255, 165, 0) 
                    self.pixels[4] = (255, 165, 0)
                    self.pixels[63]= (255, 165, 0)
                    self.pixels[64]= (255, 165, 0) 
                elif index == 2:
                    self.pixels[5]= (255, 165, 0)
                    self.pixels[6]= (255, 165, 0)
                    self.pixels[60]= (255, 165, 0)
                    self.pixels[61]= (255, 165, 0)

                elif index == 3:
                    self.pixels[8]= (255, 165, 0)
                    self.pixels[9]= (255, 165, 0)
                    self.pixels[58]= (255, 165, 0)
                    self.pixels[59]= (255, 165, 0)
                elif index == 4:
                    self.pixels[11] = (255, 165, 0)
                    self.pixels[12] = (255, 165, 0)
                    self.pixels[55]= (255, 165, 0)
                    self.pixels[54]= (255, 165, 0)
                elif index == 5:
                    self.pixels[17] = (255, 165, 0)
                    self.pixels[18] = (255, 165, 0)
                    self.pixels[50]= (255, 165, 0)
                    self.pixels[51]= (255, 165, 0)

                self.pixels.show()  # Update the LED
            time.sleep(0.2)  # On for 0.2 seconds
        with threading.Lock():
            self.pixels.fill((0, 0, 0))  # Ensure all LEDs are off at the end
            self.pixels.show()
        self.publish_status(f"Blink LED {index+1}")

    def set_error(self, error_type):
        # Function to blink all LEDs with red for non-urgent and blue for urgent error
        color = (255, 0, 0) if error_type == 'non-urgent' else (0, 0, 255)
        for _ in range(6):  # Blink 6 times
            with threading.Lock():  # Use a lock to manage simultaneous access to pixels
                self.pixels.fill(color)  # Turn all LEDs to the specified color
                self.pixels.show()
            time.sleep(0.2)  # On for 0.2 seconds
            with threading.Lock():
                self.pixels.fill((0, 0, 0))  # Turn all LEDs off
                self.pixels.show()
            time.sleep(0.2)  # Off for 0.2 seconds
        with threading.Lock():
            self.pixels.fill((0, 0, 0))  # Ensure all LEDs are off at the end
            self.pixels.show()
        self.publish_status(f"Set {error_type} error")

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {message}")

def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDController()
    try:
        rclpy.spin(led_controller)
    except KeyboardInterrupt:
        pass
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
