import pybullet as p
import time
import numpy as np
import pybullet_data
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize

#target_angle = np.pi/4   # нужный угол
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
    return np.array([y[1], - k[0] * y[1] - k[1] * np.sin(y[0])])

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

T = 2400    
u = 0.1       # Управление

tau = 0
t, acceler = [0], np.array([0])
theta, omega = np.array([p.getJointState(boxId, 1)[0]]), np.array([p.getJointState(boxId, 1)[1]])
for i in range (1, T):
    p.stepSimulation()
    #time.sleep(dt)
    tau = k[0] * omega[-1] + k[1] * np.sin(theta[-1]) + u
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=tau)
    theta = np.append(theta, p.getJointState(boxId, 1)[0])
    omega = np.append(omega, p.getJointState(boxId, 1)[1])
    acceler = np.append(acceler, (omega[-1] - omega[-2])/dt)
    t.append(i * dt)
p.disconnect()

y0 = [[0, 0]]
# Метод Эйлера c подставленным tau
def euler(end):
    h = dt
    u = 0.1
    tau = 0
    t, y = [0], y0
    while t[-1] < end - h:
        t.append(t[-1] + h)
        tau = k[0] * y[-1][1] + k[1] * np.sin(y[-1][0])
        y.append([0, y[-1][1] + h * f(y[-1], t[-1])[1]])
        y[-1][0] = (y[-2][0] + h * f([y[-2][0], y[-1][1]], t[-1])[0])
        y[-1][0] -= tau + u
        
    tau = k[0] * y[-1][1] + k[1] * np.sin(y[-1][0])
    h = end - t[-1]
    t.append(end)
    y.append([0, y[-1][1] + h * f(y[-1], t[-1])[1]])
    y[-1][0] = (y[-2][0] + h * f([y[-2][0], y[-1][1]], t[-1])[0])
    y[-1][0] -= tau + u
    return t, y

[x, y1] = euler(t[-1])
y11 = np.array(list(map(lambda i: y1[i][0], range(len(y1))))) 
y12 = np.array(list(map(lambda i: y1[i][1], range(len(y1))))) 

# odeint c tau
y21, y22 = [0], [0]
for i in range(len(t) - 1):
    y2 = odeint(f, [y21[-1], y22[-1]], [t[i], t[i + 1]])
    y21.append(y2[-1][0]) # Угол
    y22.append(y2[-1][1]) # Скорость
    tau = k[0] * y22[-1] + k[1] * np.sin(y21[-1])
    y21[-1] -= tau + u


plt.figure('Линеаризация Симуляция Pybullet')
plt.title('u = {0}'.format(u))
plt.xlabel('Время')
plt.ylabel('')
plt.plot(t, theta, t, omega)
plt.legend(['Положение', 'Скорость'])
plt.axhline(y = 0, color='gray')
plt.axvline(x = 0, color='gray')
plt.show()

plt.figure('Линеаризация odeint')
plt.title('u = {0}'.format(u))
plt.xlabel('Время')
plt.ylabel('')
plt.plot(t, y21, t, y22)
plt.legend(['Положение', 'Скорость'])
plt.axhline(y = 0, color='gray')
plt.axvline(x = 0, color='gray')
plt.show()
