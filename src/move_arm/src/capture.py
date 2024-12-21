#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import tf2_ros
from intera_interface import gripper as robot_gripper
import rospkg
import roslaunch
import numpy as np

def tuck():
    """ Tuck the robot arm to the start position. Use with caution """
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
    """ Returns the position of an AR tag in the robot's base frame. """
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(3.0))
        return [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
    except Exception:
        return None

def GetGrid():
    """ Computes grid positions for all squares on a chessboard relative to an AR tag. """
    board_marker_id = 15
    ar_pos = lookup_tag(board_marker_id)
    if ar_pos is None:
        rospy.logerr("Failed to get AR tag position.")
        return {}

    ar_x, ar_y, ar_z = ar_pos
    a1_x = ar_x - 0.02
    a1_y = ar_y + 0.155
    z_const = ar_z + 0.2
    square_size = 0.056

    ranks = '12345678'
    files = 'abcdefgh'
    square_positions = {}
    for j, r in enumerate(ranks):
        for i, f in enumerate(files):
            square_x = a1_x - (j * square_size)
            square_y = a1_y + (i * square_size)
            square_positions[f + r] = np.array([square_x, square_y, z_const])

    return square_positions

# AR piece map (IDs to pieces) and detection functions from previous steps
piece_map = {
   0: "N", 1: "n", 2: "B", 3: "p", 4: "P", 5: "b", 6: "r",
   7: "R", 8: "k", 9: "K", 10:"q", 11:"Q"
}

def detect_black_pieces_on_board(grid_positions, threshold=0.2):
   black_pieces = {
       1: "n", 3: "p", 8: "k", 10:"q", 6: "r", 5: "b"
   }
   detected_pieces = {}
   for tag_id, piece_name in black_pieces.items():
       tag_position = lookup_tag(tag_id)
       if tag_position is not None:
           tag_position[1] -= 0.03
           tag_position[0] += 0.02
           nearest_square, nearest_distance = None, float('inf')
           for square, pos in grid_positions.items():
               distance = np.linalg.norm(np.array(tag_position) - np.array(pos))
               if distance < nearest_distance:
                   nearest_square = square
                   nearest_distance = distance
           if nearest_distance <= threshold and nearest_square:
               detected_pieces[nearest_square] = piece_name
   return detected_pieces


def detect_white_pieces_on_board(grid_positions, threshold=0.2):
   white_pieces = {
       0: "N", 4: "P", 9: "K", 11:"Q", 7: "R", 2: "B"
   }
   detected_pieces = {}
   for tag_id, piece_name in white_pieces.items():
       tag_position = lookup_tag(tag_id)
       if tag_position is not None:
           tag_position[1] -= 0.03
           tag_position[0] +=0.06
           nearest_square, nearest_distance = None, float('inf')
           for square, pos in grid_positions.items():
               distance = np.linalg.norm(np.array(tag_position) - np.array(pos))
               if distance < nearest_distance:
                   nearest_square = square
                   nearest_distance = distance
           if nearest_distance <= threshold and nearest_square:
               detected_pieces[nearest_square] = piece_name
   return detected_pieces


def move_to_position(group, position, z_offset):
    """ Move the arm to a position with a specific z-offset. """
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.pose.position.x = position[0]
    target_pose.pose.position.y = position[1]
    target_pose.pose.position.z = position[2] + z_offset
    target_pose.pose.orientation.y = 1.0
    group.set_max_velocity_scaling_factor(0.7)
    group.set_pose_target(target_pose)
    group.go(wait=True)

def pick_and_release_piece(group, gripper, from_position, to_position):
    """ Pick a piece from a position and release it at another position. """
    move_to_position(group, from_position, 0.2)  # Hover above
    move_to_position(group, from_position, 0.045)  # Descend
    gripper.close()
    rospy.sleep(1.0)
    move_to_position(group, from_position, 0.2)  # Lift
    move_to_position(group, to_position, 0.2)  # Move above release location
    move_to_position(group, to_position, 0.045)  # Descend
    gripper.open()
    rospy.sleep(1.0)
    move_to_position(group, to_position, 0.2)  # Lift


def check_for_piece(to_square, detected_white, detected_black):
    """
    Check if the given square is occupied by a white or black piece.
    Returns True if occupied, False otherwise.
    """
    if to_square in detected_white or to_square in detected_black:
        return True
    return False

def main():
    rospy.init_node('piece_detection_and_move')
    group = MoveGroupCommander("right_arm")
    gripper = robot_gripper.Gripper('right_gripper')
    tuck()

    rospy.loginfo("Initializing grid positions...")
    grid_positions = GetGrid()
    if not grid_positions:
        print("Failed to compute grid positions. Exiting.")
        return

    # Example "clear" location for captured pieces
    grid_positions["clear_location"] = [0.669, -0.092, -0.048]

    while not rospy.is_shutdown():
        # Detect all pieces on the board
        print("DETECTING")
        detected_white = detect_white_pieces_on_board(grid_positions)
        detected_black = detect_black_pieces_on_board(grid_positions)

        print(f'WHITE: {detected_white}\n')
        print("-------------------------------------------\n")
        print(f'BLACK: {detected_black}\n')

        to_square = input("Enter the destination square (e.g., 'e4') or 'q' to quit: ")
        if to_square.lower() == 'q':
            break

        if to_square not in grid_positions:
            print(f"Invalid square '{to_square}'. Please try again.")
            continue

        # Check if the to_square is occupied
        tag_id = check_for_piece(to_square, detected_white, detected_black)
        if tag_id:
            print(f"Piece detected at {to_square}. Moving it to the clear location...")
            pick_and_release_piece(group, gripper, grid_positions[to_square], grid_positions["clear_location"])

        from_square = input("Enter the square to pick the piece from (e.g., 'e2'): ")
        if from_square not in grid_positions:
            print(f"Invalid square '{from_square}'. Please try again.")
            continue

        print(f"Picking up piece from {from_square} and moving it to {to_square}...")
        pick_and_release_piece(group, gripper, grid_positions[from_square], grid_positions[to_square])

    print("Exiting...")


if __name__ == '__main__':
    main()


