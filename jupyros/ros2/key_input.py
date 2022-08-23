## Keyboard input for Jupyter Ros 2
"""
Author:         Luigi Dania
Email:          Luigi@dobots.nl
Github:         https://github.com/ldania

Company:        Dobots
Company Repo:   https://github.com/dobots/ 


"""

from ipywidgets import Output
from ipycanvas import Canvas


# Method to receive keyboard input commands and send a String message to a Subscriber
# It is recommended to use a secondary node to translate your String message to an appropriate msg type


class KeyInput:
    
    # Initiate values
    def __init__(self, node, msg_type, topic, key_bindings=None):
        # Set default window size and colors
        self.width = 400
        self.height = 400
        self.color = "#1E3888"
        self.font_color = "#E3DFFF"

        # Initiate values for ipycanvas to
        self.canvas = Canvas() 
        self.canvas.fill_style = self.color 
        self.canvas.fill_rect(0, 0, self.width, self.height)
                     
        self.out = Output()
        self.msg_inst = msg_type()
        self.node = node
        self.__publisher = self.node.create_publisher(msg_type, topic, 10)
        
        self.print_outgoing_msg = True
        self.canvas.text_align = "center"
        self.key_bindings = key_bindings
        self.smallest_size = min(self.width, self.height)
        
    # Method to change the window color
    def set_color(self, color):
        self.color = color
        self.canvas.fill_style = self.color
        self.update()

    # Method to change the window width
    def set_width(self, width):
        self.width = width
        self.smallest_size = min(self.width, self.height)
        self.update()
    
    # Method to change the window height
    def set_height(self, height):
        self.height = height
        self.smallest_size = min(self.width, self.height)
        self.update()

    def print_outgoing(self, var: bool):
        self.print_outgoing_msg = var

    def update(self):
        self.canvas.clear()
        self.canvas.fill_rect(0, 0, self.width, self.height)

    # Method to display the screen and to receive keyboard inputs
    def display_inputs(self):
        self.update()
                  
        @self.out.capture()
        def on_keyboard_event(key, shift_key, ctrl_key, meta_key):
            if key:
                if self.print_outgoing_msg:
                    self.canvas.fill_rect(0, 0, self.width, self.height)
                    if str(key) == "ArrowRight":
                        print_key = "⇒"
                    elif str(key) == "ArrowDown":
                        print_key = "⇓"
                    elif str(key) == "ArrowLeft":
                        print_key = "⇐"
                    elif str(key) == "ArrowUp":
                        print_key = "⇑"
                    else:
                        print_key = key
                    
                    if len(str(print_key)) > 2:
                        factor = 5.5
                    else:
                        factor = 3

                    self.canvas.fill_style = self.font_color
                    self.font_size = self.smallest_size/factor
                    self.canvas.font = "{}px sans-serif".format(self.font_size)
                    self.canvas.fill_text(print_key, self.width/2, self.height/2+self.font_size/3)
                    self.canvas.fill_style = self.color

                self.msg_inst.data = str(key)
                self.__publisher.publish(self.msg_inst)
                
        self.canvas.on_key_down(on_keyboard_event)
        display(self.canvas)
        display(self.out)
