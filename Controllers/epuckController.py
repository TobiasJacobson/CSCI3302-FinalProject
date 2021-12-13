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
slept1 = False
slept2 = False
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

# Initialize Motor Velocities
vL = 0
vR = 0

# global var to help track wether to move in a straight line post rotation
next1 = False
next2 = False
next3 = False
next4 = False

def sleep(duration):
    global robot
    endtime = robot.getTime() + duration
    
    while robot.step(timestep) != -1 and robot.getTime() < duration:
        pass

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Set as default speed in the event no if's are triggered
    vL = MAX_SPEED/2
    vR = MAX_SPEED/2
    
    # Loop closure, if robot has delivered and returned home
    if next4:
        print("Delivery complete")
        exit()
        
    # First Corner
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
        if not slept1:
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)        
            print("Picking Up Delivery")
            slept1 = True
            sleep(40)
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
        
    # Second Corner 
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
    
    # Third Corner    
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
        if not slept2:
            robot_parts[MOTOR_LEFT].setVelocity(0)
            robot_parts[MOTOR_RIGHT].setVelocity(0)        
            print("Dropping off delivery")
            slept2 = True
            sleep(100)
        vL = MAX_SPEED/2
        vR = MAX_SPEED/2
        
    # 4th corner        
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

    # Sources of Error
    heading_error = goalThetas[state] - pose_theta

    
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/AXLE_LENGTH\
    
    # Test Prints 
    # print("pose x is: ", pose_x)
    # print("pose y is: ", pose_y)
    # print("pose theta is: ", pose_theta)
    
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28

    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
