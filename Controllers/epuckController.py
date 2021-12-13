"""epuckController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
import math


# Initlialize Variables
MAX_SPEED = 6.67 # [rad/s]
MAX_SPEED_MS = 0.22 # [m/s]
AXLE_LENGTH = 0.160 # [m]
WHEEL_RADIUS = 0.033 # m
MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

pose_x = 0
pose_y = 0
pose_theta = 0

goalStates = [(0.75, -0.5), (-0.5, -0.5), (-0.5, 0.75), (0.75, 0.75)]
goalThetas = [0, -1.570796, -3.141593, -4.712389]
state = 0

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize Parts
part_names = ("left wheel motor", "right wheel motor")
target_pos = ('inf', 'inf')
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Motor Velocities
vL = 0
vR = 0

# Main loop:
next1 = False
next2 = False
next3 = False
next4 = False
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    vL = MAX_SPEED/2
    vR = MAX_SPEED/2
    if next4:
        exit()
        # heading_error = 0
        # next4 = False
    # goalStates[state][1]
    if pose_x <  2.05 + 0.01 and pose_x > 2.05 - 0.01:
        if pose_y == 0:
            vL = 0
            vR = 0
            state = 1
            if heading_error < .1 and heading_error > -2.49: # only adjust heading once we are fine with position
                if heading_error > 0:
                    vL = 0.1*(MAX_SPEED/2)
                    vR = -vL
                elif heading_error <= 0: 
                    vR = 0.1*(MAX_SPEED/2)
                    vL = -vR
            else:
                next1 = True
    if next1:
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
    if pose_x < 3.27 + 0.01 and pose_x > 3.27 - 0.01:
        if pose_y < 1.63 + 0.01 and pose_y > 1.63 - 0.01:
            vL = 0
            vR = 0
            state = 2
            if heading_error < .1 and heading_error > -4.978: # only adjust heading once we are fine with position
                if heading_error > 0:
                    vL = 0.1*(MAX_SPEED/2)
                    vR = -vL
                elif heading_error <= 0: 
                    vR = 0.1*(MAX_SPEED/2)
                    vL = -vR
            else:
                next2 = True
    if next2:
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
        
    if pose_x < 2.72 + 0.01 and pose_x > 2.72 - 0.01:
        if pose_y < 3.57 + 0.01 and pose_y > 3.57 - 0.01:
            vL = 0
            vR = 0
            state = 3
            if heading_error < .1 and heading_error > -7.467: # only adjust heading once we are fine with position
                if heading_error > 0:
                    vL = 0.1*(MAX_SPEED/2)
                    vR = -vL
                elif heading_error <= 0: 
                    vR = 0.1*(MAX_SPEED/2)
                    vL = -vR
            else:
                next3 = True
    if next3:
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
            
    if pose_x < 0.81 + 0.01 and pose_x > 0.81 - 0.01:
        if pose_y < 4.32 + 0.01 and pose_y > 4.32 - 0.01:
            vL = 0
            vR = 0 
            # state = 4
            if heading_error < -4.98 and heading_error > -8.395: # only adjust heading once we are fine with position
                if heading_error > 0:
                    vL = 0.1*(MAX_SPEED/2)
                    vR = -vL
                elif heading_error <= 0: 
                    vR = 0.1*(MAX_SPEED/2)
                    vL = -vR
            else:
                next4 = True
    if next4:
        next1 = False
        next2 = False
        next3 = False
        next4 = False
        pose_x = 0
        pose_y = 0
        pose_theta = 0   
         
    # Sources of Error
    # bearing_error = math.atan2((goalStates[state][1] - pose_y),(goalStates[state][0] - pose_x)) - pose_theta
    # print("Bearing error is: ", bearing_error)
    heading_error = goalThetas[state] - pose_theta
    print("Heading error is: ", heading_error)
    # print("State", state)
    # Feedback Controller
    # x_r = pos_error*5 
    # theta_r = bearing_error 

                
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH
    # Prints
    print("pose x is: ", pose_x)
    print("pose y is: ", pose_y)
    print("pose theta is: ", pose_theta)
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    # if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    # if pose_theta < -3.14: pose_theta += 6.28

    # TODO: Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    

# Enter here exit cleanup code.
