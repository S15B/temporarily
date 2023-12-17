import pybullet as p
import time
import numpy as np
import pybullet_data
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize

for target_angle in np.linspace(-np.pi, np.pi, 10):
    dt = 1/240      # pybullet simulation step    
    physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version (GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    boxId = p.loadURDF("simple.urdf", useFixedBase=True)

    # get rid of all the default damping forces
    p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
    p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

    t, theta = [0], np.array([0])
    for i in range (360):
        p.stepSimulation()
        time.sleep(dt)
        speed = 8 * (target_angle - theta[-1])
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=speed, controlMode=p.VELOCITY_CONTROL)
        theta = np.append(theta, p.getJointState(boxId, 1)[0])
        t.append(i * dt)
        if abs(theta[-1] - target_angle) <= 10**(-3):
            print ('Дошли за {0} секунд'.format(round(t[-1], 5)))
            break
    p.disconnect()

'''
    plt.figure()
    plt.title('График')
    plt.xlabel('Время')
    plt.ylabel('')
    plt.plot(t, theta, t, omega, 'y')
    plt.legend(['Положение', 'Скорость'])
    plt.axhline(y = 0, color='gray')
    plt.axvline(x = 0, color='gray')
    plt.show()
'''
