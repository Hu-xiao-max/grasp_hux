import numpy as np
from scipy.spatial import cKDTree

def is_point_colliding_with_point_cloud(point, point_cloud, threshold_distance):
    kdtree = cKDTree(point_cloud)
    nearest_distance, nearest_index = kdtree.query(point)
    if nearest_distance <= threshold_distance:
        return True
    else:
        return False

def is_gripper_colliding_with_point_cloud(gripperwidth_size, gripper_grasp_point, gripper_grasp_vector, point_cloud, threshold_distance):
    # Compute the positions of the gripper fingertips in the object coordinate system
    half_gripperwidth_size = gripperwidth_size / 2
    fingertip1_position = gripper_grasp_point + half_gripperwidth_size * gripper_grasp_vector
    fingertip2_position = gripper_grasp_point - half_gripperwidth_size * gripper_grasp_vector

    # Check if the fingertips are colliding with the point cloud
    fingertip1_colliding = is_point_colliding_with_point_cloud(fingertip1_position, point_cloud, threshold_distance)
    fingertip2_colliding = is_point_colliding_with_point_cloud(fingertip2_position, point_cloud, threshold_distance)
    #只要有一个判定为True碰撞即返回True
    return fingertip1_colliding or fingertip2_colliding

# Example usage
gripperwidth_size = 0.1
gripper_grasp_point = np.array([0.5, 0.5, 0.5])
gripper_grasp_vector = np.array([1, 0, 0])

point_cloud = np.random.rand(100, 3)  # Generate a random point cloud with 100 points
threshold_distance = 0.01  # Set a threshold distance for collision detection

collision_with_point_cloud = is_gripper_colliding_with_point_cloud(gripperwidth_size, gripper_grasp_point, gripper_grasp_vector, point_cloud, threshold_distance)
print("Collision with point cloud:", collision_with_point_cloud)