#!/usr/bin/env python
import rospy
import chess
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper
from chess_utils import *

if __name__ == "__main__":
   rospy.init_node('chess_sawyer_integration', anonymous=True)
   group = MoveGroupCommander("right_arm")
   gripper = robot_gripper.Gripper('right_gripper')
   tuck()


   # Get the grid positions of the board squares
   grid_positions = GetGrid()
   print(f'GRID POS: {grid_positions}\n')
   print("-------------------------------------------\n")
   if not grid_positions:
       print("Could not detect board position. Exiting.")
       exit(1)
  
   # Additional location for captured pieces
   captured_piece_location = [0.669, -0.2, -0.01]  


   # Detect initial board setup via AR
   detected_white = detect_white_pieces_on_board(grid_positions)
   detected_black = detect_black_pieces_on_board(grid_positions)


   print(f'WHITE: {detected_white}\n')
   print("-------------------------------------------\n")
   print(f'BLACK: {detected_black}\n')


   # Convert detections into array for set_pieces_from_array
   pieces_array = []
   for sq, pc in detected_white.items():
       pieces_array.append(sq)
       pieces_array.append(pc)
   for sq, pc in detected_black.items():
       pieces_array.append(sq)
       pieces_array.append(pc)
   print("-------------------------------------------")
   print(f'PIECES: {pieces_array}\n')
   board = chess.Board()
   set_pieces_from_array(board, pieces_array)


   # Choose color
   player_is_white = True  
   player_turn = player_is_white


   while not board.is_game_over():
       if player_turn:
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

           board.push(move)
       else:
           # Stockfish move
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

                   # Move piece from 'from_sq' to 'to_sq'
                   print(f"Moving piece from {from_sq} to {to_sq}")
                   pick_and_release_piece(group, gripper, grid_positions[from_sq], grid_positions[to_sq])

                   # Update chess board
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
  
