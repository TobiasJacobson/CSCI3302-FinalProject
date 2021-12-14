from controller import Robot
import time

robot = Robot()

timestep = int(robot.getBasicTimeStep())

def sleep(duration):
    global robot
    end_time = robot.getTime() + duration
    
    while robot.step(timestep) != -1 and robot.getTime() < end_time:
        pass

m1 = robot.getDevice('joint_1')
m2 = robot.getDevice('joint_2')
m3 = robot.getDevice('joint_3')
m4 = robot.getDevice('joint_4')
m5 = robot.getDevice('joint_5')
m6 = robot.getDevice('joint_6')
m7 = robot.getDevice('joint_base_to_jaw_1')
m8 = robot.getDevice('joint_base_to_jaw_2')

# nothing until e-puck gets around
sleep(90)

m1.setPosition(0)
m2.setPosition(0.65)
m3.setPosition(0.1)
m4.setPosition(0)
m5.setPosition(-0.8)
m6.setPosition(0)
m7.setPosition(0.01)
m8.setPosition(0.01)

m1.setVelocity(1)
m2.setVelocity(1)
m3.setVelocity(1)
m4.setVelocity(1)
m5.setVelocity(1)
m6.setVelocity(1)
m7.setVelocity(1)
m8.setVelocity(1)

sleep(2)

m7.setPosition(0)
m8.setPosition(0)

m7.setVelocity(1)
m8.setVelocity(1)

sleep(2)

m1.setPosition(1.6)
m2.setPosition(0.4)
m3.setPosition(0.9)
m4.setPosition(0)
m5.setPosition(-0.3)
m6.setPosition(0)

m1.setVelocity(1)
m2.setVelocity(1)
m3.setVelocity(1)
m4.setVelocity(1)
m5.setVelocity(1)
m6.setVelocity(1)

sleep(2) 

m7.setPosition(0.01)
m8.setPosition(0.01)

m7.setVelocity(1)
m8.setVelocity(1)

sleep(2)

m1.setPosition(0)
m2.setPosition(0)
m3.setPosition(0)
m4.setPosition(0)
m5.setPosition(0)
m6.setPosition(0)
m7.setPosition(0)
m8.setPosition(0)

m1.setVelocity(1)
m2.setVelocity(1)
m3.setVelocity(1)
m4.setVelocity(1)
m5.setVelocity(1)
m6.setVelocity(1)
m7.setVelocity(1)
m8.setVelocity(1)