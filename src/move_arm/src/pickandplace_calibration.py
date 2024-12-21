#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import tf2_ros
from intera_interface import gripper as robot_gripper
import rospkg
import roslaunch
import subprocess
import numpy as np

def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('planning')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    tfBuffer = tf2_ros.Buffer()
    print("tfBuffer created")

    tfListener = tf2_ros.TransformListener(tfBuffer)
    print("tfListener created")

    try:
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
        return [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
    except Exception as e:
        print(e)
        print("Retrying ...")
        return None

def GetGrid():
    """
    Dynamically computes the positions of all squares on the chessboard relative to an AR tag.

    Adjust offsets and square_size as needed based on your calibration.
    """
    board_marker_id = 15  # Update if your AR tag for the board is different
    ar_pos = lookup_tag(board_marker_id)
    if ar_pos is None:
        rospy.logerr("Failed to get AR tag position.")
        return {}

    ar_x, ar_y, ar_z = ar_pos

    # Adjust these offsets and increments based on your actual board setup
    # Here we assume a1 is offset from the AR tag.
    a1_x = ar_x - 0.03  # Move forward (negative X) from AR tag to a1
    a1_y = ar_y + 0.15  # Move right (positive Y) from AR tag to a1
    z_const = ar_z + 0.2
    square_size = 0.056  # approximately 5.6 cm

    ranks = '12345678'  # rows
    files = 'abcdefgh'  # columns

    square_positions = {}
    for j, r in enumerate(ranks):       # rank index
        for i, f in enumerate(files):   # file index
            square_x = a1_x - (j * square_size)   # ranks along negative X
            square_y = a1_y + (i * square_size)   # files along positive Y
            square_positions[f + r] = np.array([square_x, square_y, z_const])

    # Print for debugging
    rospy.loginfo("Computed Grid Positions:")
    for sq, pos in square_positions.items():
        rospy.loginfo(f"{sq}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    return square_positions

def move_to_square(square, group, grid_positions):
    """
    Move the arm above the specified square.
    """
    if square not in grid_positions:
        rospy.logerr(f"Square {square} not found in grid.")
        return False

    pos = grid_positions[square]
    rospy.loginfo(f"Moving above square {square} at position: {pos}")

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    # Move the arm ~20 cm above the square
    target_pose.pose.position.x = pos[0]
    target_pose.pose.position.y = pos[1]
    target_pose.pose.position.z = pos[2] + 0.2

    # Gripper orientation pointing down: try x=0,y=1,z=0,w=0 if it works for your setup
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    group.set_max_velocity_scaling_factor(0.7)
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the plan in RVIZ. Press Enter to execute...")
    if isinstance(plan, tuple):
        return group.execute(plan[1], wait=True)
    else:
        return group.execute(plan, wait=True)

def pick_piece(group, gripper, target_position):
    """
    Moves the gripper above the target position, descends to pick up the piece, and closes the gripper.

    Parameters:
        group (MoveGroupCommander): The MoveIt commander for the robot arm.
        gripper (robot_gripper.Gripper): Gripper control.
        target_position (list): [x, y, z] coordinates of the target.
    """
    rospy.loginfo("Moving above target position...")

    # Step 1: Move 20 cm above the target position
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2] + 0.2  # 20 cm above
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    group.set_pose_target(target_pose)
    group.set_planning_time(10.0)
    group.set_num_planning_attempts(10)
    plan = group.plan()
    input("Check the plan to move above the target. Press Enter to execute...")
    group.execute(plan[1], wait=True)

    # Step 3: Move straight down to the pickup height
    rospy.loginfo("Descending to pick up the piece...")
    target_pose.pose.position.z = target_position[2] + 0.05  # 2 cm above block
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the descent plan. Press Enter to execute...")
    group.execute(plan[1], wait=True)
    rospy.sleep(1.0)

    # Step 4: Close the gripper
    rospy.loginfo("Closing gripper...")
    gripper.close()
    rospy.sleep(1.0)

    # Step 5: Lift back up
    rospy.loginfo("Lifting the piece...")
    target_pose.pose.position.z = target_position[2] + 0.2  # Back to safe height
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the lift plan. Press Enter to execute...")
    group.execute(plan[1], wait=True)

    rospy.loginfo("Piece picked successfully.")

def release_piece(group, gripper, target_position):
    """
    Moves the gripper above the target position, descends to place the piece, and opens the gripper.

    Parameters:
        group (MoveGroupCommander): The MoveIt commander for the robot arm.
        gripper (robot_gripper.Gripper): Gripper control.
        target_position (list): [x, y, z] coordinates of the target.
    """
    rospy.loginfo("Moving above target position...")

    # Step 1: Move 20 cm above the target position
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = target_position[0]
    target_pose.pose.position.y = target_position[1]
    target_pose.pose.position.z = target_position[2] + 0.2  # 20 cm above
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the plan to move above the target. Press Enter to execute...")
    group.execute(plan[1], wait=True)

    # Step 2: Move straight down to place the piece
    rospy.loginfo("Descending to place the piece...")
    target_pose.pose.position.z = target_position[2] + 0.05  # 2 cm above surface
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the descent plan. Press Enter to execute...")
    group.execute(plan[1], wait=True)

    # Step 3: Open the gripper
    rospy.loginfo("Opening gripper...")
    gripper.open()
    rospy.sleep(1.0)

    # Step 4: Lift back up
    rospy.loginfo("Lifting gripper...")
    target_pose.pose.position.z = target_position[2] + 0.2  # Back to safe height
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check the lift plan. Press Enter to execute...")
    group.execute(plan[1], wait=True)

    rospy.loginfo("Piece placed successfully.")

def main():
    rospy.init_node('chess_pick_and_place')
    rospy.wait_for_service('compute_ik')

    group = MoveGroupCommander("right_arm")
    right_gripper = robot_gripper.Gripper('right_gripper')

    tuck()
    rospy.loginfo("Initializing... Please wait.")
    rospy.sleep(2.0)

    # Get the grid positions from AR tag
    grid_positions = GetGrid()
    if not grid_positions:
        print("Failed to generate grid positions. Exiting.")
        return

    print("Grid positions ready. Let's pick and place a piece from one square to another.")

    while not rospy.is_shutdown():
        from_square = input("Enter the square to pick the piece from (e.g., 'e2') or 'q' to quit: ")
        if from_square.lower() == 'q':
            break
        if from_square not in grid_positions:
            print("Invalid from_square. Please try again.")
            continue

        to_square = input("Enter the square to place the piece at (e.g., 'e4'): ")
        if to_square not in grid_positions:
            print("Invalid to_square. Please try again.")
            continue

        # Move above from_square
        if move_to_square(from_square, group, grid_positions):
            # Pick piece
            pick_piece(group, right_gripper, grid_positions[from_square])

            # Move above to_square
            if move_to_square(to_square, group, grid_positions):
                # Release piece
                release_piece(group, right_gripper, grid_positions[to_square])
                print(f"Successfully moved piece from {from_square} to {to_square}.")
            else:
                print(f"Failed to move above {to_square}.")
        else:
            print(f"Failed to move above {from_square}.")

    print("Exiting.")

if __name__ == '__main__':
    main()