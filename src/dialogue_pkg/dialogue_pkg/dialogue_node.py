import rclpy
from rclpy.node import Node
import tkinter as tk
from threading import Thread
import pygame

class DialogueNode(Node):
    def __init__(self):
        super().__init__('dialogue_node')

        # Initialize pygame for audio playback
        pygame.init()

        # Start the GUI in a separate thread
        self.gui_thread = Thread(target=self.start_gui, daemon=True)
        self.gui_thread.start()

    def start_gui(self):
        self.root = tk.Tk()
        self.root.title("Dialogue Interface")

        # Create label and entry for text input
        tk.Label(self.root, text="Enter or click a drawer number (1-6):").pack(padx=20, pady=10)
        self.entry = tk.Entry(self.root)
        self.entry.pack(padx=20, pady=10)
        self.entry.bind("<Return>", self.on_enter)

        # Create buttons for direct selection
        button_frame = tk.Frame(self.root)
        button_frame.pack(padx=20, pady=10)
        for number in range(1, 7):
            button = tk.Button(button_frame, text=str(number), 
                               command=lambda num=str(number): self.button_click(num))
            button.pack(side=tk.LEFT, padx=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_enter(self, event):
        self.process_input()

    def button_click(self, number):
        self.process_input(number)

    def process_input(self, number=None):
        if number is None:
            number = self.entry.get()

        drawer_audio_files = {
            '1': "src/dialogue_pkg/dialogue_pkg/drawer_M1.wav",
            '2': "src/dialogue_pkg/dialogue_pkg/drawer_M2.wav",
            '3': "src/dialogue_pkg/dialogue_pkg/drawer_M3.wav",
            '4': "src/dialogue_pkg/dialogue_pkg/drawer_M4.wav",
            '5': "src/dialogue_pkg/dialogue_pkg/drawer_M5.wav",
            '6': "src/dialogue_pkg/dialogue_pkg/drawer_M6.wav",
        }

        audio_file = drawer_audio_files.get(number)
        if audio_file:
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()
        else:
            print("Invalid drawer number")

    def on_close(self):
        pygame.quit()
        self.root.quit()

def main(args=None):
    rclpy.init(args=args)
    dialogue_node = DialogueNode()
    rclpy.spin(dialogue_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
