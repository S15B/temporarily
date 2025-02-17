import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np

# Инициализация PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Настройка гравитации
# p.setGravity(0, 0, -9.81)

# Создание плоскости
p.loadURDF("plane.urdf")

# Устанавливаем положение и ориентацию камеры
camera_distance = 2.5  # Расстояние от камеры до целевой точки
camera_pitch = 0  # Угол наклона камеры (в градусах)
camera_yaw = 0     # Угол поворота камеры (в градусах)
target_position = [1, 0, 1.5]  # Точка, на которую направлена камера

p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, target_position)

dt = 1/240      # pybullet simulation step 

# Размерности 
radius, height = 0.04, 0.06                         # Цилиндр
width_leg, depth_leg, length_leg = 0.07, 0.06, 0.5   # Плечо
length_platform = 0.25                              

cyl_rot = p.getQuaternionFromEuler([np.pi/2, 0, 0]) 
zero_rot = p.getQuaternionFromEuler([0, 0, 0])

# Создание визуальной и коллизионной форм для базы всех 4-х тел
joint_visual_shape, joint_collision_shape = [], []
for i in range(4):
    joint_visual_shape.append(p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[1, 0.5, 0.3125, 1]))
    joint_collision_shape.append(p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height))

# Создание визуальной и коллизионной форм для 6 ног
leg_visual_shape, leg_collision_shape = [], []
for i in range(6):
    leg_visual_shape.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_leg/2], rgbaColor=[0.25, 0.875, 0.8125, 1]))
    leg_collision_shape.append(p.createCollisionShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_leg/2]))

# Создание трёх неподвижных тел
# База - крутящийся шарнир. Соединения - 2 плеча, соединных призматическим соединением
anchor_id_1 = p.createMultiBody(
    baseMass=0,                          
    basePosition=[np.sqrt(2)/2, 0, 2],      
    baseCollisionShapeIndex=joint_collision_shape[0],          
    baseVisualShapeIndex=joint_visual_shape[0],
    baseOrientation=cyl_rot,
    linkMasses=[1, 1],
    linkCollisionShapeIndices=[-1, leg_collision_shape[1]],
    linkVisualShapeIndices=[leg_visual_shape[0], leg_visual_shape[1]],
    linkPositions=[[0, 0, 0], [0, 0, length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, np.pi]), zero_rot],
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)

anchor_id_2 = p.createMultiBody(
    baseMass=0,                          
    basePosition=[2, 0, 1.5],      
    baseCollisionShapeIndex=joint_collision_shape[1],          
    baseVisualShapeIndex=joint_visual_shape[1],
    baseOrientation=cyl_rot,
    linkMasses=[1, 1],
    linkCollisionShapeIndices=[-1, leg_collision_shape[3]],
    linkVisualShapeIndices=[leg_visual_shape[2], leg_visual_shape[3]],
    linkPositions=[[0, 0, 0], [0, 0, length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, np.pi*3/4]), zero_rot],
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)

anchor_id_3 = p.createMultiBody(
    baseMass=0,                         
    basePosition=[0, 0, 1],      
    baseCollisionShapeIndex=joint_collision_shape[2],          
    baseVisualShapeIndex=joint_visual_shape[2],
    baseOrientation=cyl_rot,
    linkMasses=[1, 1],
    linkCollisionShapeIndices=[-1, leg_collision_shape[5]],
    linkVisualShapeIndices=[leg_visual_shape[4], leg_visual_shape[5]],
    linkPositions=[[0, 0, 0], [0, 0, length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi/2]), zero_rot],
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)

# Создание визуальной формы для 3 ног и 3 joint платформы
plat_leg_visual_shape, plat_joint_visual_shape = [], []
for i in range(3):
    plat_joint_visual_shape.append(p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[1, 0.5, 0.3125, 1]))
    plat_leg_visual_shape.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_platform/2], rgbaColor=[0.25, 0.5, 1, 1]))
    
# Создание платформы 
# База - центр. Соединения - 3 плеча и 3 шарнира
platform_id = p.createMultiBody(
    baseMass=0.2,
    baseCollisionShapeIndex=-joint_collision_shape[3],
    baseVisualShapeIndex=joint_visual_shape[3],
    basePosition=[1, 0, 1.2],
    baseOrientation=cyl_rot,
    linkMasses=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
    linkCollisionShapeIndices=[-1] * 6,     # Без коллизии, иначе будет мешаться вращению
    linkVisualShapeIndices=plat_leg_visual_shape + plat_joint_visual_shape,
    linkPositions=[[0, length_platform/2, 0], 
                   [length_platform*np.sqrt(3)/4, -length_platform/4, 0],
                   [-length_platform*np.sqrt(3)/4, -length_platform/4, 0]] +
                   [[0, 0, length_platform/2]] * 3,
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, 0]), 
                      p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi*2/3]),
                      p.getQuaternionFromEuler([-np.pi/2, 0, np.pi*2/3])] +
                      [cyl_rot] * 3,
    linkInertialFramePositions=[[0, 0, 0]] * 6,
    linkInertialFrameOrientations=[[zero_rot]] * 6,
    linkParentIndices=[0, 0, 0, 1, 2, 3],
    linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 0, 0]] * 3 + [[0, 0, 1]] * 3
)

# # Ограничения на поворот
# p.changeDynamics(anchor_id_1, 0, jointLowerLimit=-np.pi/2, jointUpperLimit=np.pi/4)
# p.changeDynamics(anchor_id_2, 0, jointLowerLimit=-np.pi/4, jointUpperLimit=np.pi/2)

# Соединение плеч с платформой
constraint_id_1 = p.createConstraint(
    parentBodyUniqueId=anchor_id_1, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=1,
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_POINT2POINT, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]  
)
constraint_id_2 = p.createConstraint(
    parentBodyUniqueId=anchor_id_2, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=3,
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_POINT2POINT, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]  
)
constraint_id_3 = p.createConstraint(
    parentBodyUniqueId=anchor_id_3, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=5,
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_POINT2POINT, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]  
)

# Нужно ли это? Пока оставил
p.changeDynamics(anchor_id_1, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_2, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_1, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_2, 1, linearDamping=0, angularDamping=0)

# На стартовую позицию
for i in range(1, 500):
    p.stepSimulation()

# Создание массивов для графиков
t, x, y, angle = np.array([0]), np.array([p.getBasePositionAndOrientation(platform_id)[0][0]]),\
                         np.array([p.getBasePositionAndOrientation(platform_id)[0][2]]),\
                         np.array([p.getEulerFromQuaternion(p.getBasePositionAndOrientation(platform_id)[1])[1]])

# Симуляция
for i in range(1, 500):
    #p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.POSITION_CONTROL, targetPosition=0.5, force=500)
    #p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.POSITION_CONTROL, targetPosition=0.5, force=500)
    p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0.1, force=500)
    p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0.1, force=500)

    x = np.append(x, p.getBasePositionAndOrientation(platform_id)[0][0])
    y = np.append(y, p.getBasePositionAndOrientation(platform_id)[0][2])
    angle = np.append(angle, p.getEulerFromQuaternion(p.getBasePositionAndOrientation(platform_id)[1])[1])
    t = np.append(t, i*dt)

    time.sleep(dt)
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)

angle = angle * 180/np.pi

# Завершение симуляции
p.disconnect

# Отрисовка графиков
plt.figure('')
plt.subplot(3, 1, 1)
plt.grid(True)
#plt.title('Приведение из {0} в положение {1}'.format(round(start_pos, 4), round(target_angle, 4)))
#plt.axhline(y = target_angle, color='yellow', linestyle='dashed')
plt.xlabel('Время')
plt.ylabel('X')
plt.plot(t, x)
plt.subplot(3, 1, 2)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('Y')
plt.plot(t, y)
plt.subplot(3, 1, 3)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('Угол в градусах')
plt.plot(t, angle)
plt.show()
