import pybullet as p
import pybullet_data
import time
import random

# Initialize the physics engine
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the KUKA robot arm
kuka_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Set initial joint positions
def set_joint_positions(robot_id, joint_positions):
    num_joints = p.getNumJoints(robot_id)
    for i in range(min(len(joint_positions), num_joints)):
        p.setJointMotorControl2(bodyUniqueId=robot_id,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_positions[i],
                                force=500)

initial_joint_positions = [0.0, -1.0, 1.5, -1.5, 0.0, 0.0, 0.0]
set_joint_positions(kuka_id, initial_joint_positions)

# Add a blue baffle to simulate an obstacle
plane_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.1, 0.2])
plane_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.1, 0.2], rgbaColor=[0, 0, 1, 1])
plane = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plane_id, baseVisualShapeIndex=plane_visual, basePosition=[0.5, 2, 0.5])

# Add a red sphere as the main target
target_sphere = p.loadURDF("sphere2.urdf", [0.5, 2, 0.5], globalScaling=0.15)
p.changeVisualShape(target_sphere, -1, rgbaColor=[1, 0, 0, 1])

# Create a physical camera model
camera_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
camera_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1], rgbaColor=[0, 0, 0, 1])
camera = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=camera_collision, baseVisualShapeIndex=camera_visual, basePosition=[0, 0, 1])

# Define the movement range for all objects
random_movement_range = {
    "x": [0.0, 1.0],
    "y": [1.5, 2.5],
    "z": [0.0, 1.0],
}

# Current ball position
current_position_ball = [0.5, 2, 0.5]
target_position_ball = current_position_ball.copy()

# Add additional moving objects (e.g., 3 objects with different shapes and colors)
other_objects = []
other_objects_positions = []
other_objects_target_positions = []

# Define additional objects: shape, size, color
object_shapes = [
    (p.GEOM_BOX, [0.05, 0.05, 0.05], [0, 1, 0, 1]),      # Green small cube
    (p.GEOM_CYLINDER, [0.05, 0.05, 0.05], [1, 1, 0, 1]), # Yellow small cylinder
    (p.GEOM_SPHERE, [0.05], [0, 1, 1, 1])                # Cyan small sphere
]

for shape_def in object_shapes:
    geom_type = shape_def[0]
    half_ext = shape_def[1]
    color = shape_def[2]

    if geom_type == p.GEOM_SPHERE:
        # For sphere, we need a radius
        radius = half_ext[0]
        collision_shape_id = p.createCollisionShape(geom_type, radius=radius)
        visual_shape_id = p.createVisualShape(geom_type, radius=radius, rgbaColor=color)

    elif geom_type == p.GEOM_BOX:
        # For box, we need halfExtents
        collision_shape_id = p.createCollisionShape(geom_type, halfExtents=half_ext)
        visual_shape_id = p.createVisualShape(geom_type, halfExtents=half_ext, rgbaColor=color)

    elif geom_type == p.GEOM_CYLINDER:
        # For cylinder, we need radius and height.
        # Let's assume half_ext[0] is the radius. The height is explicitly chosen here.
        radius = half_ext[0]
        height = 0.1
        collision_shape_id = p.createCollisionShape(geom_type, radius=radius, height=height)
        visual_shape_id = p.createVisualShape(geom_type, radius=radius, length=height, rgbaColor=color)

    init_pos = [
        random.uniform(random_movement_range["x"][0], random_movement_range["x"][1]),
        random.uniform(random_movement_range["y"][0], random_movement_range["y"][1]),
        random.uniform(random_movement_range["z"][0], random_movement_range["z"][1])
    ]

    obj_id = p.createMultiBody(baseMass=0.01,
                               baseCollisionShapeIndex=collision_shape_id,
                               baseVisualShapeIndex=visual_shape_id,
                               basePosition=init_pos)
    other_objects.append(obj_id)
    other_objects_positions.append(init_pos)
    other_objects_target_positions.append(init_pos.copy())

# Define an obstacle avoidance range (not extensively used here)
obstacle_range = {
    "x": [0.0, 1.0],
    "y": [1.9, 2.1],
    "z": [0.3, 0.7]
}

# Camera parameters
camera_position = [0, 0, 1.0]  
camera_target = [0.5, 2, 0.5]  
camera_up = [0, 0, 1]

def path_clear(start, end):
    ray_test_results = p.rayTest(start, end)
    for result in ray_test_results:
        if result[0] == plane:
            return False
    return True

def generate_target_position_for_ball(current_position):
    while True:
        new_position = [
            random.uniform(random_movement_range["x"][0], random_movement_range["x"][1]),
            random.uniform(random_movement_range["y"][0], random_movement_range["y"][1]),
            random.uniform(random_movement_range["z"][0], random_movement_range["z"][1]),
        ]
        if path_clear(current_position, new_position):
            return new_position

def generate_target_position_for_others():
    return [
        random.uniform(random_movement_range["x"][0], random_movement_range["x"][1]),
        random.uniform(random_movement_range["y"][0], random_movement_range["y"][1]),
        random.uniform(random_movement_range["z"][0], random_movement_range["z"][1]),
    ]

time_step = 0
while True:
    # Update ball target position every 240 steps
    if time_step % 240 == 0:
        target_position_ball = generate_target_position_for_ball(current_position_ball)
    # Update other objects target position every 300 steps
    if time_step % 300 == 0:
        for i in range(len(other_objects)):
            other_objects_target_positions[i] = generate_target_position_for_others()

    # Smoothly move the ball
    for i in range(3):
        current_position_ball[i] += (target_position_ball[i] - current_position_ball[i]) * 0.05
    p.resetBasePositionAndOrientation(target_sphere, current_position_ball, [0, 0, 0, 1])

    # Smoothly move other objects
    for i, obj_id in enumerate(other_objects):
        for j in range(3):
            other_objects_positions[i][j] += (other_objects_target_positions[i][j] - other_objects_positions[i][j]) * 0.05
        p.resetBasePositionAndOrientation(obj_id, other_objects_positions[i], [0,0,0,1])

    # Update the physical camera's orientation
    p.resetBasePositionAndOrientation(camera, camera_position, p.getQuaternionFromEuler([0, -0.5, 0]))

    # Get the virtual camera image
    view_matrix = p.computeViewMatrix(camera_position, camera_target, camera_up)
    projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=10.0)
    width, height, rgb_image, depth_image, segmentation_image = p.getCameraImage(
        width=640,
        height=480,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    # Assume we detect the ball accurately. 
    detected_ball_position = current_position_ball

    # Ray test to check if the ball is visible or blocked
    end_effector_state = p.getLinkState(kuka_id, 6)
    end_effector_position = end_effector_state[0]
    ray_test_result = p.rayTest(end_effector_position, detected_ball_position)

    # If not blocked by the blue baffle, track the ball; otherwise return to initial position
    if ray_test_result[0][0] != plane:
        joint_positions = p.calculateInverseKinematics(kuka_id, 6, detected_ball_position)
        set_joint_positions(kuka_id, joint_positions)
    else:
        set_joint_positions(kuka_id, initial_joint_positions)

    # Debug info
    print(f"Ball Position: {current_position_ball}, Ray Test: {ray_test_result[0]}")

    # Step simulation
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    time_step += 1
