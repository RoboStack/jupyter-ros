## Keyboard input for Jupyter Ros 2
"""
Author:         Luigi Dania
Email:          Luigi@dobots.nl
Github:         https://github.com/ldania

Company:        Dobots
Company Repo:   https://github.com/dobots/ 


"""

import sys
sys.path.append("./../jupyter-ros")
import jupyros.ros2 as jr2
from ipywidgets import Output
from ipycanvas import Canvas
import rclpy


## Method to receive keyboard input commands and send a String message to a Subcriber
## It is recommended to use a secondary node to translate your String message to an appropiate msg type


class key_input:
    
    #Initiate values
    def __init__(self, node, msg_type, topic, key_bindings = None):
        # Set default Window size and colors
        self.width = 400
        self.height = 400
        self.color = "gray"
    
        
        ## Initiate values for Ipycanvas to
        self.canvas = Canvas() 
        self.canvas.fill_style = "gray"
        self.canvas.fill_rect(0, 0, self.width , self.height)
                     
        self.out = Output()
        self.msg_inst = msg_type()
        self.node = node
        self.__publisher = self.node.create_publisher(msg_type, topic, 10)
        
        self.print_outgoing_msg = True
        self.canvas.text_align = "center"
        self.key_bindings = key_bindings
        #Using the Ros2 Jupyros Publisher module, create 
        #self.key_in = jr2.Publisher(node, msg_type, topic)
        
    # Method to change the window color
    def set_color(self, color):
        self.color = color
    
    
    # Method to change the window width
    def set_width(self, width):
        self.width = width
    
    # Method to change the window height
    def set_height (self, height):
        self.height = height
    
    
    def print_outgoing(Var: bool):
        self.print_outgoing_msg = Var

    
        

  

    # method to display the screen and to receive keyboard inputs
    def display(self):
        
                  
        @self.out.capture()
        def on_keyboard_event(key, shift_key, ctrl_key, meta_key):
            if (key):
                if(self.print_outgoing_msg):
                    self.canvas.stroke_text(key,self.width/2, self.height/2)
                    print("Keyboard event:", key)
                self.msg_inst.data = str(key)
                self.__publisher.publish(self.msg_inst)
                
        self.canvas.on_key_down(on_keyboard_event)
        display(self.canvas)
        display(self.out)
        
        
        
    
    
    
     
    
    
    
