import pybullet as p
import time
import numpy as np
import pybullet_data
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize

target_angle = np.pi/4   # нужный угол
dt = 1/240      # pybullet simulation step    
physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version (GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# Модель уравнения маятника
k = [1.56249828, 12.499986]
def f(y, x):
    return np.array([y[1], -k[0] * y[1] - k[1] * np.sin(y[0])])

T = 2400

def simulation (K):
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=target_angle, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    # turn off the motor for the free motion
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    
    t, theta, omega = [0], np.array([0]), np.array([0])
    for i in range (1, T):
        p.stepSimulation()
        u = - K[0] * theta[-1] - K[1] * omega[-1]
        tau = k[0] * omega[-1] + k[1] * np.sin(theta[-1]) + u
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=tau)
        theta = np.append(theta, p.getJointState(boxId, 1)[0])
        omega = np.append(omega, p.getJointState(boxId, 1)[1])
        t.append(i * dt)
    
    return t, theta

def error_search(K):    # Функционал, который нужно минимизировать
    theta = simulation(K)[1]
    return sum(list(map(lambda i: (theta[i] - target_angle)**2, range(len(theta)))))

res = minimize(error_search, x0 = [1, 1])
t, theta = simulation (res.x)
p.disconnect()

plt.figure('LQR')
plt.title('Остановка в положение {0}'.format(round(target_angle, 4)))
plt.xlabel('Время')
plt.ylabel('')
plt.plot(t, theta)
plt.axhline(y = target_angle, color='yellow', linestyle='dashed')
plt.legend(['Положение', 'Нужный угол'])
plt.axhline(y = 0, color='gray')
plt.axvline(x = 0, color='gray')
plt.show()

print ('Коэффициенты K', res.x)



