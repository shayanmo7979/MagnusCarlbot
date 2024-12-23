#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import tf2_ros
import numpy as np
import chess
import requests
import json
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper

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
   a1_x = ar_x - 0.03  # Move forward (negative X) from AR tag to a1
   a1_y = ar_y + 0.15  # Move right (positive Y) from AR tag to a1
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
           tag_position[1]
           tag_position[0] += 0.05
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
           tag_position[1] -= 0.005
           tag_position[0] +=0.03
           nearest_square, nearest_distance = None, float('inf')
           for square, pos in grid_positions.items():
               distance = np.linalg.norm(np.array(tag_position) - np.array(pos))
               if distance < nearest_distance:
                   nearest_square = square
                   nearest_distance = distance
           if nearest_distance <= threshold and nearest_square:
               detected_pieces[nearest_square] = piece_name
   return detected_pieces


def set_pieces_from_array(board, pieces_array):
   board.clear()
   if len(pieces_array) % 2 != 0:
       raise ValueError("Pieces array length must be even.")


   for i in range(0, len(pieces_array), 2):
       square = pieces_array[i]
       piece_symbol = pieces_array[i + 1]
       if square not in chess.SQUARE_NAMES:
           continue
       try:
           piece = chess.Piece.from_symbol(piece_symbol)
           board.set_piece_at(chess.parse_square(square), piece)
       except ValueError:
           continue


def get_stockfish_move(fen):
    payload = {
        "fen": fen,
        "depth": 6  
    }
    try:
        response = requests.post("https://chess-api.com/v1", json=payload)
        if response.status_code == 200:
            output_dictionary = json.loads(response.text)
            return output_dictionary["move"]
        else:
            rospy.logerr(f"API Error: {response.status_code}, {response.text}")
            return None
    except Exception as e:
        rospy.logerr(f"Error calling Stockfish API: {e}")
        return None


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
   # Approach the piece
   move_to_position(group, from_position, 0.2)  # Hover above
   move_to_position(group, from_position, 0.045) # Descend
   gripper.close()
   rospy.sleep(1.0)
   move_to_position(group, from_position, 0.2)  # Lift


   # Move to new location
   move_to_position(group, to_position, 0.2)    # Hover above
   move_to_position(group, to_position, 0.045)   # Descend
   gripper.open()
   rospy.sleep(1.0)
   move_to_position(group, to_position, 0.2)    # Lift away


def is_square_occupied(square, white_pieces, black_pieces):
   if square in white_pieces or square in black_pieces:
       print(f"Square {square} is occupied. Removing the piece...")


       pick_and_release_piece(group, gripper, grid_positions[square], captured_piece_location)


       # Update dictionaries
       if square in white_pieces:
           del white_pieces[square]
       if square in black_pieces:
           del black_pieces[square]


       return True
   return False

def check_for_piece(to_square, board):
    """
    Check if the given square is occupied by a white or black piece.
    Returns True if occupied, False otherwise.
    """
    square = chess.parse_square(to_square)
    piece = board.piece_at(square)
    return True if piece else False
