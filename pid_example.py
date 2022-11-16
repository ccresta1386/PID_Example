from turtle import color
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as mpatches
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation

P = 10
I = 0.01
D = 6

class Robot:

    def __init__(self, x_pos: float, p: float, i: float, d: float):
        self.x_pos = x_pos
        self.controller = Controller(x_pos, p, i, d)
        self.speed = 0
        self.goal = 0
        self.mass = 3
        self.friction = 0.01

    def update(self, dt):
        self.effort = (self.controller.compute_effort(
            self.x_pos, self.goal, dt))/2-0.1
        acc = self.effort/self.mass
        # Equtions of motion:
        self.x_pos = self.x_pos + self.speed*dt + acc*dt**2 - self.friction*dt**2
        self.speed = self.speed + acc*dt - self.friction*dt
        return self.x_pos


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
goal_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03])
fig.subplots_adjust(bottom=0.25)
goal_slider = Slider(
    ax=goal_ax,
    label='Goal [m]',
    valmin=-10,
    valmax=10,
    valinit=0,
)

dt = 0.01

# Visual parameters
w_body = 12
h_body = 5
d_wheel = 5
# initial position of the cart
x_pos = -10

# Make a robot visual
body = mpatches.Rectangle([-w_body/2, d_wheel/4], w_body, h_body)
l_wheel = mpatches.Circle((-w_body/4, d_wheel/2), d_wheel/2, color="r")
r_wheel = mpatches.Circle((w_body/4, d_wheel/2), d_wheel/2, color="r")
ax.add_patch(body)
ax.add_patch(l_wheel)
ax.add_patch(r_wheel)


# Create an instance of a robot
robot = Robot(x_pos, P, I, D)
robot.goal = 0


def update_goal(val):
    robot.goal = val


goal_slider.on_changed(update_goal)


def init():
    ax.set_aspect('equal')
    return (body, l_wheel, r_wheel)


def update(i):
    x_pos = robot.update(dt)
    body.set_transform(Affine2D().translate(x_pos, 0) + ax.transData)
    l_wheel.set_transform(Affine2D().translate(x_pos, 0) + ax.transData)
    r_wheel.set_transform(Affine2D().translate(x_pos, 0) + ax.transData)
    return (body, l_wheel, r_wheel)


ani = FuncAnimation(fig, update, frames=range(int(1000)),
                    init_func=init, interval=dt, blit=True)
ax.set_xlim((-16, 16))
ax.set_ylim((0, 12))
plt.show()
