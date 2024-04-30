import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.patches as mpatches
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
"""
This is a simple simulation of a 2-Dimensional PID control of an object to show how the P,I, and D parameters
will influence the response of the "Robot" to their variation.

To use run the python script and set the x and y sliders to the desired positions, then click go. Close the 
first plot to show the 'response' plot. This is how the performance of the PID is evaluated.

Try varying P, I, and D and seeing how the initial motion changes. Look at the response plot and try to make it 
look like the critically damped plot here:
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#response-types

In depth explanation: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html

TL;DR: Proportional pushes the system towards the setpoint, derivative looks ahead to say it is moving to fast, and needs to slow down,
and integral says, it haven't gotten to the goal and it's been 10 seconds, maybe I should try harder.

"""

# Change these to mess with the response
P = 50
I = 0.01
D = 30

#### Change below to mess with the simulation

class Robot:

    def __init__(self, x_pos: float, y_pos: float, p: float, i: float, d: float):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.x_controller = Controller(x_pos, p, i, d)
        self.y_controller = Controller(y_pos, p, i, d)
        self.x_speed = 0
        self.y_speed = 0
        self.x_goal = 0
        self.y_goal = 0
        self.mass = 30
        self.max_effort = 50
        self.friction = 0.03

    def update(self, dt):
        effort = self.x_controller.compute_effort(
            self.x_pos, self.x_goal, dt)
        
        x_effort = np.sign(effort)*min(np.abs(effort), self.max_effort)
        
        effort = self.y_controller.compute_effort(
            self.y_pos, self.y_goal, dt)

        y_effort = np.sign(effort)*min(np.abs(effort), self.max_effort)
        
        x_acc = x_effort/self.mass
        y_acc = y_effort/self.mass
        # Equtions of motion:
        self.x_pos = self.x_pos + self.x_speed*dt + x_acc/2*dt**2 - self.friction*dt**2
        self.x_speed = self.x_speed + x_acc*dt - self.friction*dt

        self.y_pos = self.y_pos + self.y_speed*dt + y_acc/2*dt**2 - self.friction*dt**2
        self.y_speed = self.y_speed + y_acc*dt - self.friction*dt
        
        return self.x_pos, self.y_pos


###### Change below to mess with the controller

class Controller:
    def __init__(self, x_position: float, p: float, i: float, d: float):
        self.x_prev = x_position
        self.x_position = x_position
        self.p = p
        self.i = i
        self.d = d
        self.accrued_error = 0
        self.error_previous = 0
        self.windup_limit = 10 # The maximum value of the integral control portion
        self.t_prev = -0.0001

    def compute_effort(self, x_position, x_goal, dt):
        error = x_goal-x_position
        self.accrued_error = self.accrued_error + error
        p_effort = error*self.p
        sign = np.sign(self.accrued_error)
        self.accrued_error = sign*min(np.abs(self.accrued_error), self.windup_limit)
        i_effort = self.accrued_error*self.i
        d_effort = self.d*(error-self.error_previous)/(dt)
        self.error_previous = error
        return p_effort + i_effort + d_effort


# Script to set up the animation
# Set up the plot for animation
fig, ax = plt.subplots()
x_goal_ax = fig.add_axes([0.25, 0.1, 0.65, 0.03])
y_goal_ax = fig.add_axes([0.1, 0.25, 0.03, 0.65])
button_ax = fig.add_axes([0.3, 0.15, 0.375, 0.1])
check_ax = fig.add_axes([0.7, 0.15, 0.2, 0.1])

fig.subplots_adjust(bottom=0.35,left=0.25)
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
follow_cursor_button = Button(check_ax, "Follow Cursor", color="r")

# dt is the change in time, aka timestep
dt = 0.001

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

def cursor_toggle(val):
    if(follow_cursor_button.color == "r"):
        follow_cursor_button.color = "g"
    else:
        follow_cursor_button.color = "r"

def mouse_move(event):
    if follow_cursor_button.color == "g":
        x, y = event.xdata, event.ydata
        if event.inaxes != None and event.inaxes == ax:
            robot.x_goal = x
            robot.y_goal = y

go_button.on_clicked(func=update_goal)
follow_cursor_button.on_clicked(func=cursor_toggle)


def init():
    ax.set_aspect('equal')
    return [body]

x_errors = []
y_errors = []
def update(i):
    x_pos, y_pos = robot.update(dt*10)
    x_errors.append(robot.x_goal-robot.x_pos)
    y_errors.append(robot.y_goal-robot.y_pos)
    body.set_transform(Affine2D().translate(x_pos, y_pos) + ax.transData)
    return [body]


ani = FuncAnimation(fig, update, frames=range(int(1000)),
                    init_func=init, interval=dt*1000, blit=True)
ax.set_xlim((-16, 16))
ax.set_ylim((-16, 16))

plt.connect('motion_notify_event', mouse_move)
plt.show()
plt.figure()
plt.plot(np.array(list(range(0,len(x_errors))))*dt,x_errors)
plt.plot(np.array(list(range(0,len(y_errors))))*dt, y_errors)
plt.legend(["x","y"])
plt.ylabel("Error")
plt.xlabel("Time (s)")
plt.show()