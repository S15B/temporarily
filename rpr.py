import pybullet as p
import time
import pybullet_data
import numpy as np

# Инициализация PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Настройка гравитации
# p.setGravity(0, 0, -9.81)

# Создание плоскости
p.loadURDF("plane.urdf")

radius = 0.05
height = 0.05
length_leg = 0.5

# Создание формы для платформы (сфера)
sphere_visual_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height)
sphere_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)

# Создание визуальной и коллизионной форм для ног
leg_visual_shape_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2])
leg_collision_shape_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2])
leg_visual_shape_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2])
leg_collision_shape_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2])

# Создание первого фиктивного неподвижного тела
anchor_position_1 = [0, 0, 1]
anchor_id_1 = p.createMultiBody(
    baseMass=0,                         
    basePosition=anchor_position_1,      
    baseCollisionShapeIndex=-1,          
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height),
    linkMasses=[1, 1],
    linkCollisionShapeIndices=[leg_collision_shape_1, sphere_collision_shape],
    linkVisualShapeIndices=[leg_visual_shape_1, sphere_visual_shape],
    linkPositions=[[0, 0, 0], [0, 0, np.sqrt(2)/2]],
    linkOrientations=[p.getQuaternionFromEuler([0, np.pi/4, 0]), p.getQuaternionFromEuler([0, 0, 0])],
    linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
    linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0])],
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, -1, 0], [0, 0, 1]] 
)

# Создание второго фиктивного неподвижного тела
anchor_position_2 = [1, 0, 2]
anchor_id_2 = p.createMultiBody(
    baseMass=0,                          
    basePosition=anchor_position_2,      
    baseCollisionShapeIndex=-1,          
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height),
    linkMasses=[1],
    linkCollisionShapeIndices=[leg_collision_shape_2],
    linkVisualShapeIndices=[leg_visual_shape_2],
    linkPositions=[[0, 0, 0]],
    linkOrientations=[p.getQuaternionFromEuler([0, np.pi/4, 0])],
    linkInertialFramePositions=[[0, 0, 0]],
    linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0])],
    linkParentIndices=[0],
    linkJointTypes=[p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 1, 0]] 
)

# Создание ограничения (шарнир между фиктивным телом и платформой)
constraint_id = p.createConstraint(
    parentBodyUniqueId=anchor_id_2, 
    parentLinkIndex=0,
    childBodyUniqueId=anchor_id_1,
    childLinkIndex=1,
    jointType=p.JOINT_PRISMATIC, 
    jointAxis=[0, 0, 1],  # Движение вдоль оси Z
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[0, 0, 0]  
)

p.changeDynamics(anchor_id_1, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_2, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_1, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(constraint_id, 1, linearDamping=0, angularDamping=0)

for i in range(10000):
    p.setJointMotorControl2(anchor_id_1, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-1, force=10)
    p.setJointMotorControl2(anchor_id_2, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-1, force=10)
    p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.POSITION_CONTROL, targetPosition=1/2 - length_leg/2, force=10)
    p.setJointMotorControl2(constraint_id, 1, controlMode=p.POSITION_CONTROL, targetPosition=-1/2 + length_leg/2, force=1)
    
    p.stepSimulation()
    time.sleep(1 / 240)

# Завершение симуляции
p.disconnect
