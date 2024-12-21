import matplotlib.pyplot as plt
import chess
import requests
import json

def display_eval_bar(score):
    """Displays the evaluation bar based on the score."""
    normalized_score = max(-1, min(1, score / 1000))  # Normalize between -1 and 1

    fig, ax = plt.subplots(figsize=(4, 0.4))
    ax.barh(0, normalized_score, color="green" if normalized_score > 0 else "red", align="center")
    ax.set_xlim([-1, 1])
    ax.set_yticks([])
    ax.set_xticks([-1, 0, 1])
    ax.set_xticklabels(["Black", "Equal", "White"])
    ax.set_title("Evaluation Bar")
    plt.show()

def initialize_10x10_board():
    """Create a 10x10 matrix for the chessboard with red outer borders."""
    board = [[" " for _ in range(10)] for _ in range(10)]
    for row in range(10):
        for col in range(10):
            if row == 0 or row == 9 or col == 0 or col == 9:
                board[row][col] = "R"  # Red border
            elif (row + col) % 2 == 0:
                board[row][col] = "."  # Light square
            else: 
                board[row][col] = "#"  # Dark square
    return board

def update_10x10_with_8x8(ten_board, board):
    """Update the 10x10 board to reflect the occupied positions of the 8x8 board."""
    for row in range(8):
        for col in range(8):
            piece = board.piece_at(chess.square(col, 7 - row))
            if piece:
                ten_board[row + 1][col + 1] = "O"  # Mark as "occupied"
    return ten_board

def print_board(matrix):
    """Generic print function for any board."""
    for row in matrix:
        print(" ".join(row))

def display_8x8_board(board):
    """Display the 8x8 chessboard using matplotlib."""
    plt.figure(figsize=(6, 6))
    for row in range(8):
        for col in range(8):
            color = "white" if (row + col) % 2 == 0 else "black"
            plt.fill_between(
                [col, col + 1],
                [7 - row, 7 - row],
                [8 - row, 8 - row],
                color=color,
            )
            piece = board.piece_at(chess.square(col, 7 - row))
            if piece:
                plt.text(
                    col + 0.5,
                    7 - row + 0.5,
                    str(piece),
                    ha="center",
                    va="center",
                    fontsize=14,
                    color="red" if color == "white" else "white",
                )
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xticks(range(9), labels=list("ABCDEFGH") + [""])
    plt.yticks(range(9), labels=list(range(1, 9)) + [""])
    plt.grid(True, which="both", color="gray", linewidth=0.5)
    plt.show()

def choose_color():
    """Prompt the player to choose their color."""
    player_color = input("Choose your color (White/Black): ").strip().lower()
    return player_color == "white"  # Return True if white, False if black

def get_stockfish_move(fen):
    """Get a move from Stockfish API."""
    payload = {"fen": fen}  # Use the provided FEN string
    response = requests.post("https://chess-api.com/v1", json=payload)
    if response.status_code == 200:
        output_dictionary = json.loads(response.text)
        print(f"Move: {output_dictionary['move']}, Capture: {output_dictionary.get('capture')}")
        return output_dictionary["move"]
    else:
        print(f"Error: API returned status code {response.status_code}")
        return None

def manually_set_pieces(board):
    """
    Allow the user to manually set the positions of pieces on the board.
    Example input: "e2,P" to place a White pawn at e2.
    """
    board.clear()  # Clear the board to start with an empty configuration
    print("Manually set up the board. Enter positions as '<square>,<piece>' (e.g., e2,P).")
    print("Use uppercase for White pieces and lowercase for Black pieces.")
    print("Pieces: K (king), Q (queen), R (rook), B (bishop), N (knight), P (pawn).")
    print("Type 'done' when you are finished.\n")
    
    while True:
        user_input = input("Enter position (or 'done' to finish): ").strip()
        if user_input.lower() == "done":
            break

        try:
            square, piece = user_input.split(",")
            if square not in chess.SQUARE_NAMES:
                print(f"Invalid square: {square}. Try again.")
                continue
            
            piece = chess.Piece.from_symbol(piece)
            board.set_piece_at(chess.parse_square(square), piece)
        except Exception as e:
            print(f"Invalid input format. Error: {e}. Try again.")
    
    print("Final manual board setup:")
    print(board)
    print("\n")

if __name__ == "__main__":
    # Initialize chess board
    board = chess.Board()
    board.clear()  # Start with an empty board

    # Allow user to manually set the board
    manual_setup = input("Would you like to set up the board manually? (yes/no): ").strip().lower()
    if manual_setup == "yes":
        manually_set_pieces(board)
    else:
        board.reset()  # Start with a standard board setup
    
    # Determine player's color
    player_turn = choose_color()

    # Initialize score for the evaluation bar
    score = 0

    # Game loop
    while not board.is_game_over():
        if player_turn:
            # Player's move
            player_move = input("Enter your move (in UCI notation): ")

            # Validate and apply the player's move
            if chess.Move.from_uci(player_move) in board.legal_moves:
                board.push(chess.Move.from_uci(player_move))
                print(f"Player moved: {player_move}")
            else:
                print("Invalid move. Try again.")
                continue
        else:
            # Stockfish's turn
            fen = board.fen()
            stockfish_move = get_stockfish_move(fen)
            if stockfish_move:
                board.push(chess.Move.from_uci(stockfish_move))
                print(f"Stockfish moved: {stockfish_move}")
            else:
                print("Failed to get Stockfish move. Try again.")
                continue

        # Alternate turns
        player_turn = not player_turn

        # Visualize the updated boards
        updated_10x10 = update_10x10_with_8x8(initialize_10x10_board(), board)
        
        display_8x8_board(board)




    # End of the game section
    print("Game over. Result:", board.result())
