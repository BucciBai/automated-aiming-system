import pybullet as p
import pybullet_data
import time
import random

# Initialize the physics engine
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Loading the Robot Arm Using the Existing Pybullet Library
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

# Add blue baffles to simulate cars or obstacle shelters
plane_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.1, 0.2])  # 长条状
plane_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.1, 0.2], rgbaColor=[0, 0, 1, 1])
plane = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=plane_id, baseVisualShapeIndex=plane_visual, basePosition=[0.5, 2, 0.5])

# Add a sphere to simulate the target to be tracked
target_sphere = p.loadURDF("sphere2.urdf", [0.5, 2, 0.5], globalScaling=0.15)

# Creating a physical camera model
camera_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
camera_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1], rgbaColor=[0, 0, 0, 1])
camera = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=camera_collision, baseVisualShapeIndex=camera_visual, basePosition=[0, 0, 1])

# Define the range of movement of the sphere
random_movement_range = {
    "x": [0.0, 1.0],
    "y": [1.5, 2.5],
    "z": [0.0, 1.0],
}

# Define the obstacle avoidance range, avoid blue obstacle entities, and avoid penetration
obstacle_range = {
    "x": [0.0, 1.0],  
    "y": [1.9, 2.1],  
    "z": [0.3, 0.7]   
}

# Current ball position
current_position = [0.5, 2, 0.5]
target_position = current_position.copy()

# Camera parameters
camera_position = [0, 0, 1.0]  # Camera located above the robot base
camera_target = [0.5, 2, 0.5]  # Camera looks at the blue baffle
camera_up = [0, 0, 1]  # Camera's up direction

# Function to check if the path intersects the obstacle
def path_clear(start, end):
    ray_test_results = p.rayTest(start, end)
    for result in ray_test_results:
        if result[0] == plane: 
            return False
    return True

# Generates valid target locations and avoids obstacles and clears paths
def generate_target_position():
    while True:
        new_position = [
            random.uniform(random_movement_range["x"][0], random_movement_range["x"][1]),
            random.uniform(random_movement_range["y"][0], random_movement_range["y"][1]),
            random.uniform(random_movement_range["z"][0], random_movement_range["z"][1]),
        ]
        if path_clear(current_position, new_position): 
            return new_position

# Simulation loop
time_step = 0
while True:
    # Every 240 steps, generate a new target position for the ball
    if time_step % 240 == 0:
        target_position = generate_target_position()

    # Smoothly move the ball
    for i in range(3):  #x,y,z
        current_position[i] += (target_position[i] - current_position[i]) * 0.05

    # Update the ball's position
    p.resetBasePositionAndOrientation(target_sphere, current_position, [0, 0, 0, 1])

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

    # Simulate capturing the ball position through the camera
    detected_position = current_position  # Assume the detected position is equal to the real position

    # Use ray casting to check if the ball is obstructed
    end_effector_state = p.getLinkState(kuka_id, 6)
    end_effector_position = end_effector_state[0]
    ray_test_result = p.rayTest(end_effector_position, detected_position)

    if ray_test_result[0][0] != plane:  # If the ray does not hit the baffle
        # Robot arm tracks the ball
        joint_positions = p.calculateInverseKinematics(kuka_id, 6, detected_position)
        set_joint_positions(kuka_id, joint_positions)
    else:
        # If the ray is blocked by the baffle, the arm resets to its initial position
        set_joint_positions(kuka_id, initial_joint_positions)

    # Print debug info.
    print(f"Red Ball Position: {current_position}, Detected: {detected_position}, Ray Test: {ray_test_result[0]}")

    # Step simulation
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    time_step += 1
