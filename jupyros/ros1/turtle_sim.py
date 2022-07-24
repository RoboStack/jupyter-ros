import os
import time
import math
import random

import ipycanvas
import ipywidgets

import rospkg


class TurtleSim:
    def __init__(self, width=1600, height=1600, turtle_size=100, background_color="#4556FF"):
        self.turtles = {}
        self.turtle_size = turtle_size
        self.canvas_middle = {"x": width // 2,
                              "y": height // 2,
                              "theta": 0}

        # Three layers for the canvas: 0-background, 1-paths, 2-turtles
        self.canvas = ipycanvas.MultiCanvas(3,
                                            width=width, height=height,
                                            layout={"width": "100%"})

        # Water background
        self.canvas[0].fill_style = background_color
        self.canvas[0].fill_rect(0, 0, width, height)

        # Turtle path width
        self.canvas[1].line_width = 8

        self.last_move_time = time.time()
        self.spawn()

    def spawn(self, name=None, pose=None):

        if (name is None) or (name in self.turtles.keys()):
            name = "turtle" + str(len(self.turtles) + 1)

        self.turtles[name] = self.Turtle(name, self.turtle_size)

        if pose is None:
            # Spawn to middle of canvas
            self.turtles[name].pose = self.canvas_middle
        else:
            self.turtles[name].pose = pose

        with ipycanvas.hold_canvas(self.canvas):
            self.draw_turtle(name)

        print(name + " has spawned.")

    def move_turtles(self, new_poses):
        elapsed_time = time.time() - self.last_move_time

        if elapsed_time > 0.08:  # seconds
            self.last_move_time = time.time()

            with ipycanvas.hold_canvas(self.canvas):
                self.canvas[2].clear()

                for name in self.turtles.keys():
                    # Draw line path
                    self.canvas[1].stroke_style = self.turtles[name].path_color
                    self.canvas[1].stroke_line(self.turtles[name].pose["x"],
                                               self.turtles[name].pose["y"],
                                               new_poses[name]["x"],
                                               new_poses[name]["y"])

                    # Update
                    self.turtles[name].pose = new_poses[name]
                    self.draw_turtle(name)

    def draw_turtle(self, name="turtle1", n=2):
        # Offsets for turtle center and orientation
        x_offset = - self.turtle_size / 2
        y_offset = - self.turtle_size / 2
        theta_offset = self.turtles[name].pose["theta"] - math.radians(90)  # to face right side

        # Transform canvas
        self.canvas[n].save()
        self.canvas[n].translate(self.turtles[name].pose["x"], self.turtles[name].pose["y"])
        self.canvas[n].rotate(-theta_offset)

        self.canvas[n].draw_image(self.turtles[name].canvas,
                                  x_offset, y_offset,
                                  self.turtle_size)

        # Revert transformation
        self.canvas[n].restore()

    class Turtle:
        def __init__(self, name, size=100):
            self.name = name
            self.size = size
            self.canvas = None
            self.randomize()
            self.pose = {"x": 0,
                         "y": 0,
                         "theta": 0}
            self.path_color = '#B3B8FF'  # Light blue

        def randomize(self):
            r = rospkg.RosPack()
            img_path = r.get_path('turtlesim') + '/images/'
            images = os.listdir(img_path)
            turtle_pngs = [img for img in images if ('.png' in img and 'palette' not in img)]
            random_png = turtle_pngs[random.randint(0, len(turtle_pngs) - 1)]
            turtle_img = ipywidgets.Image.from_file(img_path + random_png)
            turtle_canvas = ipycanvas.Canvas(width=self.size, height=self.size)

            with ipycanvas.hold_canvas(turtle_canvas):
                turtle_canvas.draw_image(turtle_img, 0, 0, self.size)

            time.sleep(0.1)  # Drawing time
            self.canvas = turtle_canvas

            return self
