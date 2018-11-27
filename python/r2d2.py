import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)

maxForce = 500
p.setJointMotorControl2(bodyUniqueId=0, 
	jointIndex=0, 
	controlMode=p.VELOCITY_CONTROL,
	targetVelocity = 5,
	force = maxForce)


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
	
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

