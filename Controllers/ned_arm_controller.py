from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

def sleep(duration):
    global robot
    endtime = robot.getTime() + duration
    
    while robot.step(timestep) != -1 and robot.getTime() < duration: # end_time?
        pass

# def sleep(duration):
    # global robot
    # endtime = robot.getTime() + duration
    
    # while robot.step(timestep) != -1 and robot.getTime() < end_time:
        # pass

m1 = robot.getDevice('joint_1')
m2 = robot.getDevice('joint_2')
m3 = robot.getDevice('joint_3')
m4 = robot.getDevice('joint_4')
m5 = robot.getDevice('joint_5')
m6 = robot.getDevice('joint_6')
m7 = robot.getDevice('joint_base_to_jaw_1')
m8 = robot.getDevice('joint_base_to_jaw_2')

m1.setPosition(-1.6)
m2.setPosition(0.5)
m3.setPosition(0.4)
m4.setPosition(0)
m5.setPosition(-0.3)
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

m2.setPosition(0.72)
m3.setPosition(0.5)
m5.setPosition(-0.4)

m2.setVelocity(1)
m3.setVelocity(1)
m5.setVelocity(1)

sleep(3)

m7.setPosition(0) # robot's left prong
m8.setPosition(0) # robot's right (my left)

m7.setVelocity(1)
m8.setVelocity(1)

sleep(4)

m1.setPosition(0)
m2.setPosition(0)
m3.setPosition(0)
m4.setPosition(0)
m5.setPosition(0)
m6.setPosition(0)
m7.setPosition(0)
m8.setPosition(0)

m1.setVelocity(0.5)
m2.setVelocity(0.5)
m3.setVelocity(0.5)
m4.setVelocity(0.5)
m5.setVelocity(0.5)
m6.setVelocity(0.5)
m7.setVelocity(0.5)
m8.setVelocity(0.5)

# position of e-puck stopped
# m1.setPosition(0)
# m2.setPosition(0.7)
# m3.setPosition(0.2)
# m4.setPosition(0)
# m5.setPosition(-0.6)
# m6.setPosition(0)
# m7.setPosition(0.01)
# m8.setPosition(0.01)

