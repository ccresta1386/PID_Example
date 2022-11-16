import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.patches as mpatches
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation

P = 20
I = 0.00001
D = 10

class Robot:

    def __init__(self, x_pos: float, y_pos: float, p: float, i: float, d: float):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.controller = Controller(x_pos, p, i, d)
        self.x_speed = 0
        self.y_speed = 0
        self.x_goal = 0
        self.y_goal = 0
        self.mass = 3
        self.friction = 0.0

    def update(self, dt):
        x_effort = (self.controller.compute_effort(
            self.x_pos, self.x_goal, dt))
        
        y_effort = (self.controller.compute_effort(
            self.y_pos, self.y_goal, dt))
        
        x_acc = x_effort/self.mass
        y_acc = y_effort/self.mass
        # Equtions of motion:
        self.x_pos = self.x_pos + self.x_speed*dt + x_acc*dt**2 - self.friction*dt**2
        self.x_speed = self.x_speed + x_acc*dt - self.friction*dt

        self.y_pos = self.y_pos + self.y_speed*dt + y_acc*dt**2 - self.friction*dt**2
        self.y_speed = self.y_speed + y_acc*dt - self.friction*dt
        
        return self.x_pos, self.y_pos


class Controller:
    def __init__(self, x_pos: float, p: float, i: float, d: float):
        self.x_prev = x_pos
        self.x_pos = x_pos
        self.p = p
        self.i = i
        self.d = d
        self.acc_err = 0
        self.err_prev = 0
        self.t_prev = -0.0001

    def compute_effort(self, x_pos, x_goal, dt):
        err = x_goal-x_pos
        print(f"Error: {err}")
        self.acc_err = self.acc_err + err
        P = err*self.p
        I = self.acc_err*self.i
        D = self.d*(err-self.err_prev)/(dt)
        self.err_prev = err
        print(f"Effort: {P+I+D}")
        return P+I+D


# Script to set up the animation
# Set up the plot for animation
fig, ax = plt.subplots()
x_goal_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03])
y_goal_ax = fig.add_axes([0.1, 0.25, 0.03, 0.65])
button_ax = fig.add_axes([0.35, 0.2, 0.65, 0.1])
fig.subplots_adjust(bottom=0.25,left=0.25)
goal_slider_x = Slider(
    ax=x_goal_ax,
    label='X Goal [m]',
    valmin=-10,
    valmax=10,
    valinit=0,
)
goal_slider_y = Slider(
    ax=y_goal_ax,
    orientation="vertical",
    label='Y Goal [m]',
    valmin=-10,
    valmax=10,
    valinit=0,
)
go_button = Button(button_ax, "Go!",color="r")
dt = 0.01

# Visual parameters
w_body = 5
h_body = 5
d_wheel = 5
# initial position of the cart
x_pos = -10
y_pos = -10

# Make a robot visual
body = mpatches.Rectangle([-w_body/2, -w_body/2], w_body, h_body)
ax.add_patch(body)


# Create an instance of a robot
robot = Robot(x_pos, y_pos, P, I, D)
robot.x_goal = 0
robot.y_goal = 0


def update_goal(val):
    robot.x_goal = goal_slider_x.val
    robot.y_goal = goal_slider_y.val


go_button.on_clicked(func=update_goal)


def init():
    ax.set_aspect('equal')
    return [body]


def update(i):
    x_pos, y_pos = robot.update(dt)
    body.set_transform(Affine2D().translate(x_pos, y_pos) + ax.transData)
    return [body]


ani = FuncAnimation(fig, update, frames=range(int(1000)),
                    init_func=init, interval=dt, blit=True)
ax.set_xlim((-16, 16))
ax.set_ylim((-16, 16))
plt.show()
