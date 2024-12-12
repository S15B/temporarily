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

radius = 0.07
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
anchor_id_1 = p.createMultiBody(
    baseMass=0,                         
    basePosition=[0, 0, 1],      
    baseCollisionShapeIndex=-1,          
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height),
    linkMasses=[1],
    linkCollisionShapeIndices=[leg_collision_shape_1],
    linkVisualShapeIndices=[leg_visual_shape_1],
    linkPositions=[[0, 0, 0]],
    linkOrientations=[p.getQuaternionFromEuler([0, np.pi/4, 0])],
    linkInertialFramePositions=[[0, 0, 0]],
    linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0])],
    linkParentIndices=[0],
    linkJointTypes=[p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 1, 0]] 
)

# Создание второго фиктивного неподвижного тела
anchor_id_2 = p.createMultiBody(
    baseMass=0,                          
    basePosition=[1, 0, 2],      
    baseCollisionShapeIndex=-1,          
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height),
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
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

# Создание платформы
platform_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2]),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.03, length_leg/2]),
    basePosition=[0.5, 0, 1.5],
    baseOrientation=p.getQuaternionFromEuler([0, np.pi/4, 0]),
    linkMasses=[1, 1],
    linkCollisionShapeIndices=[p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height),
                               p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)],
    linkVisualShapeIndices=[p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height),
                            p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height)],
    linkPositions=[[0, 0, -length_leg/2], [0, 0, length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0])],
    linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
    linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0]), p.getQuaternionFromEuler([0, 0, 0])],
    linkParentIndices=[0, 0],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE],
    linkJointAxis=[[0, 1, 0], [0, 1, 0]]
)

# Создание ограничения (шарнир между фиктивным телом и платформой)
constraint_id_1 = p.createConstraint(
    parentBodyUniqueId=anchor_id_1, 
    parentLinkIndex=0,
    childBodyUniqueId=platform_id,
    childLinkIndex=0,
    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointType=p.JOINT_PRISMATIC, 
    jointAxis=[0, 0, 1],  # Движение вдоль оси Z
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[0, 0, 0]  
)
p.changeDynamics(constraint_id_1, 1, linearDamping=0, angularDamping=0, contactDamping=0, contactStiffness=0)

# Создание ограничения (шарнир между фиктивным телом и платформой)
constraint_id_2 = p.createConstraint(
    parentBodyUniqueId=anchor_id_2, 
    parentLinkIndex=0,
    childBodyUniqueId=platform_id,
    childLinkIndex=1,
    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointType=p.JOINT_PRISMATIC, 
    jointAxis=[0, 0, 1],  # Движение вдоль оси Z
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[0, 0, 0]  
)
p.changeDynamics(constraint_id_2, 1, linearDamping=0, angularDamping=0, contactDamping=0, contactStiffness=0)

p.changeDynamics(anchor_id_1, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(anchor_id_2, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(platform_id, 0, linearDamping=0, angularDamping=0)
p.changeDynamics(platform_id, 1, linearDamping=0, angularDamping=0)


for i in range(10000):
   
    p.setJointMotorControl2(anchor_id_1, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-1, force=10)
    p.setJointMotorControl2(anchor_id_2, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=1, force=10)
    #p.setJointMotorControl2(constraint_id_1, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=0.01, force=10)
    #p.setJointMotorControl2(constraint_id_2, 0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-0.01, force=10)
    
    p.stepSimulation()
    time.sleep(1 / 240)

# Завершение симуляции
# p.disconnect
