#!/usr/bin/env python
import rospy
import rospkg
import roslaunch
import tf2_ros
import numpy as np
import chess
import requests
import json
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper


###############################
# Basic Robot Setup Functions #
###############################


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


####################################
# Chess and Move Generation Functions
####################################

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


####################################
# Sawyer Pick and Place Functions  #
####################################

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
    move_to_position(group, to_position, 0.045)  # Descend
    gripper.open()
    rospy.sleep(1.0)
    move_to_position(group, to_position, 0.2)    # Lift away


def check_for_piece(to_square, board):
    """
    Check if the given square is occupied by a white or black piece.
    Returns True if occupied, False otherwise.
    """
    square = chess.parse_square(to_square)
    piece = board.piece_at(square)
    return True if piece else False


####################################
# Main Full Game Mode Logic        #
####################################

if __name__ == "__main__":
    rospy.init_node('chess_sawyer_full_game_mode', anonymous=True)
    group = MoveGroupCommander("right_arm")
    gripper = robot_gripper.Gripper('right_gripper')
    tuck()

    # Get the grid positions of the board squares via AR tag
    grid_positions = GetGrid()
    print(f'GRID POS: {grid_positions}\n')
    print("-------------------------------------------\n")
    if not grid_positions:
        print("Could not detect board position. Exiting.")
        exit(1)

    # Additional location for captured pieces
    captured_piece_location = [0.669, -0.2, -0.01]  # Adjust as needed

    # Standard chess initial position mapping
    pieces_array = [
        # White pieces
        'a1','R','b1','N','c1','B','d1','Q','e1','K','f1','B','g1','N','h1','R',
        'a2','P','b2','P','c2','P','d2','P','e2','P','f2','P','g2','P','h2','P',

        # Black pieces
        'a7','p','b7','p','c7','p','d7','p','e7','p','f7','p','g7','p','h7','p',
        'a8','r','b8','n','c8','b','d8','q','e8','k','f8','b','g8','n','h8','r'
    ]

    board = chess.Board()
    set_pieces_from_array(board, pieces_array)

    # Choose color
    player_is_white = True  # Example: player is White and moves first
    player_turn = player_is_white

    while not board.is_game_over():
        if player_turn:
            # Player physically moves their piece; the robot does NOT move it.

            # Player's turn
            player_move = input("Enter your move in UCI notation (e.g. e2e4): ").strip()
            if len(player_move) == 0:
                print("No input detected. Please enter a valid move like 'e2e4'.")
                continue

            try:
                move = chess.Move.from_uci(player_move)
            except chess.InvalidMoveError:
                print(f"Invalid move '{player_move}'. Please enter a move like 'e2e4'.")
                continue

            if move not in board.legal_moves:
                print("Illegal move. Try again.")
                continue

            # We only update the internal chess board state (no Sawyer pick/place).
            board.push(move)

        else:
            # Sawyer (Black) move with Stockfish
            fen = board.fen()
            stockfish_move = get_stockfish_move(fen)
            if stockfish_move:
                move = chess.Move.from_uci(stockfish_move)
                if move in board.legal_moves:
                    from_sq = stockfish_move[:2]
                    to_sq = stockfish_move[2:4]

                    # Check if capture is needed
                    if check_for_piece(to_sq, board):
                        print(f"Piece detected at {to_sq}. Moving it to the captured location...")
                        pick_and_release_piece(group, gripper, grid_positions[to_sq], captured_piece_location)

                    # Execute the Stockfish move on the board physically
                    print(f"Moving piece from {from_sq} to {to_sq}")
                    pick_and_release_piece(group, gripper, grid_positions[from_sq], grid_positions[to_sq])
                    board.push(move)
                    print(board)
                else:
                    print("Stockfish provided an illegal move.")
                    break
            else:
                print("No move received from Stockfish.")
                break

        # Switch turns
        player_turn = not player_turn

    print("Game over:", board.result())

