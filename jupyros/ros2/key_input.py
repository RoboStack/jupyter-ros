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
from time import time


## Method to receive keyboard input commands and send a String message to a Subcriber
## It is recommended to use a secondary node to translate your String message to an appropiate msg type


class key_input:
    
    #Initiate values
    def __init__(self, node, msg_type, topic, key_bindings = None):
        # Set default Window size and colors
        self.width = 400
        self.height = 400
        self.color = "blue"
    
        
        ## Initiate values for Ipycanvas to
        self.canvas = Canvas() 
        self.canvas.fill_style = self.color 
        self.canvas.fill_rect(0, 0, self.width , self.height)
                     
        self.out = Output()
        self.msg_inst = msg_type()
        self.node = node
        self.__publisher = self.node.create_publisher(msg_type, topic, 10)
        
        self.print_outgoing_msg = True
        self.canvas.text_align = "center"
        
        
        self.key_bindings = key_bindings
        
        self.smallest_size = min(self.width, self.height)
        
        
        
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
    
    
    def print_outgoing(self, Var: bool):
        self.print_outgoing_msg = Var

    def update(self):
        self.canvas.fill_rect(0, 0, self.width , self.height)
        

  

    # method to display the screen and to receive keyboard inputs
    def display(self):
        self.update()
                  
        @self.out.capture()
        def on_keyboard_event(key, shift_key, ctrl_key, meta_key):
            if (key):
                if(self.print_outgoing_msg):
                    self.canvas.fill_rect(0, 0, self.width , self.height)
                    if(str(key) == "ArrowRight"):
                        print_key = "⇒"
                    elif(str(key) == "ArrowDown"):
                        print_key = "⇓"
                    elif(str(key) == "ArrowLeft"):
                        print_key = "⇐"
                    elif(str(key) == "ArrowUp"):
                        print_key = "⇑"
                    else:
                        print_key = key
                    
                    if(len(str(print_key))>2):
                        factor = 5.5
                    else:
                        factor = 3
                    self.canvas.fill_style = "red"  
                    self.font_size = self.smallest_size/factor
                    self.canvas.font = "{}px sans-serif".format(self.font_size)
                    self.canvas.fill_text(print_key,self.width/2, self.height/2+self.font_size/3)
                    self.canvas.fill_style = "blue"

                self.msg_inst.data = str(key)
                self.__publisher.publish(self.msg_inst)
                
        self.canvas.on_key_down(on_keyboard_event)
        display(self.canvas)
        display(self.out)
        
        
        
    
    
    
     
    
    
    
