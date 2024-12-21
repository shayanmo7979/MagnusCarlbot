#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import tf2_ros
import rospkg
import roslaunch
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
    Dynamically computes the positions of all squares on the chessboard relative to the AR tag.

    Adjustments:
    - Fix offsets so the AR tag is correctly referenced as left of 'a1'.
    - Ensure 'a1' is the true starting point for the grid.
    - Derive all squares based on fine-tuned increments.
    """
    board_marker_id = 15  # AR tag ID for the chessboard
    ar_pos = lookup_tag(board_marker_id)
    if ar_pos is None:
        rospy.logerr("Failed to get AR tag position.")
        return {}

    ar_x, ar_y, ar_z = ar_pos

    # Adjusted offsets: AR tag is LEFT of a1, so a1 is slightly to the RIGHT (+Y) and FORWARD (-X)
    a1_x = ar_x - 0.03  # Move forward (negative X) from AR tag to a1
    a1_y = ar_y + 0.15  # Move right (positive Y) from AR tag to a1
    z_const = ar_z + 0.15  # Constant Z height above the board

    # Grid increments based on actual chessboard spacing
    square_size = 0.056  # 6.1 cm increments

    # Predefine ranks and files
    ranks = '12345678'  # Rows go along -X
    files = 'abcdefgh'  # Columns go along +Y

    # Compute grid positions
    square_positions = {}
    for j, r in enumerate(ranks):       # Loop over rows (ranks)
        for i, f in enumerate(files):   # Loop over columns (files)
            square_x = a1_x - (j * square_size)  # Move up the ranks (negative X)
            square_y = a1_y + (i * square_size)  # Move right across the files (positive Y)
            square_positions[f + r] = np.array([square_x, square_y, z_const])

    rospy.loginfo("Adjusted Grid Positions (relative to AR tag):")
    for square, pos in square_positions.items():
        rospy.loginfo(f"{square}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    return square_positions



def move_to_square(square, group, grid_positions):
    """
    Moves the arm above a specified square on the board.

    Parameters:
        square (str): Name of the square (e.g., 'e2')
        group: MoveGroupCommander instance for controlling the robot arm.
        grid_positions (dict): Precomputed positions of all squares.
    """
    if square not in grid_positions:
        rospy.logerr(f"Square {square} not found.")
        return

    pos = grid_positions[square]
    rospy.loginfo(f"Moving to square {square} at position: {pos}")

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = pos[0]
    target_pose.pose.position.y = pos[1]
    target_pose.pose.position.z = pos[2] + 0.15 # Move above the square

    # Gripper orientation (pointing down)
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    group.set_max_velocity_scaling_factor(0.8)
    group.set_pose_target(target_pose)
    plan = group.plan()
    input("Check plan in RVIZ. Press Enter to execute move above tag/coord...")
    if isinstance(plan, tuple):
        group.execute(plan[1], wait=True)
    else:
        group.execute(plan, wait=True)
    return True


def main():
    rospy.init_node('service_query')
    group = MoveGroupCommander("right_arm")

    tuck()
    print("Initializing... Please wait.")
    rospy.sleep(2.0)

    # Generate the grid positions relative to the AR tag
    grid_positions = GetGrid()
    if not grid_positions:
        print("Failed to generate grid positions. Exiting.")
        return

    print("Grid positions generated. Ready to test.")
    print("Enter the square name (e.g., 'a1', 'b2') to move above it. Type 'q' to quit.")

    while not rospy.is_shutdown():
        square = input("Enter square to move to: ")
        if square.lower() == 'q':
            print("Exiting.")
            break
        elif square in grid_positions:
            move_to_square(square, group, grid_positions)
        else:
            print("Invalid square name. Please try again.")   
    

if __name__ == '__main__':
    main()