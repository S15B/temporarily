
import pybullet as p
import time
import numpy as np
import pybullet_data
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize

target_angle = np.pi/4
dt = 1/240      # pybullet simulation step    
physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version (GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

t, theta, omega = [0], np.array([0]), np.array([0])
errow_old, sum_errow = 0, 0
for i in range (360):
    p.stepSimulation()
    time.sleep(dt)
    errow = (target_angle - theta[-1])
    sum_errow += errow * dt
    pid = 500 * errow + 100 * (sum_errow) + 70 * (errow - errow_old)
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=pid)
    errow_old = errow
    theta = np.append(theta, p.getJointState(boxId, 1)[0])
    omega = np.append(omega, p.getJointState(boxId, 1)[1])
    t.append(i * dt)
p.disconnect()
print (target_angle)
print (theta[239])


plt.figure()
plt.title('График')
plt.xlabel('Время')
plt.ylabel('')
plt.plot(t, theta, 'y')
plt.legend(['Положение', 'Скорость'])
plt.axhline(y = 0, color='gray')
plt.axvline(x = 0, color='gray')
plt.show()

