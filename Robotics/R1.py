import pybullet as p
import time
import pybullet_data

physicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId=p.loadURDF("plane.urdf")
cubeStartPos=[0,0,1]
cubeStartOrientation=p.getQuaternionFromEuler([0,0,0])
ur=p.loadURDF("myFirst.urdf",cubeStartPos,cubeStartOrientation)
print("Number of joints: "+str(p.getNumJoints(ur)))
##can write a for loop and base it off of the number of joints

iKey=ord('i')
kKey=ord('k')

for i in range(1000000):
    p.stepSimulation()
    time.sleep(1./240.)
    keys=p.getKeyboardEvents()

    #bottom arm twisting

    if p.B3G_RETURN in keys and keys[p.B3G_RETURN]&p.KEY_WAS_TRIGGERED:
        p.resetBasePositionAndOrientation(ur, [0 , 0, 2], cubeStartOrientation)
        
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=0,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= 5,force= maxForce)

    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=0,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)

    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=0,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= -5,force= maxForce)

    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=0,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)

    #bottom arm up and down

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=1,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= 5,force= maxForce)

    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=1,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)

    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=1,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= -5,force= maxForce)

    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=1,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)

    #upper arrow up and down

    if iKey in keys and keys[iKey]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=2,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= 5,force= maxForce)

    if iKey in keys and keys[iKey]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=2,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)

    if kKey in keys and keys[kKey]&p.KEY_WAS_TRIGGERED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=2,controlMode=p.VELOCITY_CONTROL,
        targetVelocity= -5,force= maxForce)

    if kKey in keys and keys[kKey]&p.KEY_WAS_RELEASED:
        maxForce=500
        p.setJointMotorControl2(bodyUniqueId=ur, jointIndex=2,controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0, force=maxForce)
        
cubePos, cubeOrn=p.getBasePositionAndOrientation(robot)
print(cubePos,cubeOrn)

p.disconnect()

    

    




