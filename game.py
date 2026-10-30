"""game.py — กติกาเกม XO: สถานะกระดาน ตรวจผู้ชนะ และตรวจว่าถึงตาใคร"""

EMPTY = "-"
PLAYER = "X"   # ผู้เล่น (คน)
ROBOT = "O"    # หุ่นยนต์

# หมายเลขช่อง (index 0-8) เรียงซ้ายไปขวา บนลงล่าง
#  0 | 1 | 2
#  3 | 4 | 5
#  6 | 7 | 8
WIN_PATTERNS = [
    (0, 1, 2), (3, 4, 5), (6, 7, 8),  # แนวนอน
    (0, 3, 6), (1, 4, 7), (2, 5, 8),  # แนวตั้ง
    (0, 4, 8), (2, 4, 6),             # แนวทแยง
]


def new_board():
    return [EMPTY] * 9


def check_winner(board):
    """คืนค่า 'X' / 'O' / 'DRAW' หรือ None ถ้าเกมยังไม่จบ"""
    for a, b, c in WIN_PATTERNS:
        if board[a] == board[b] == board[c] != EMPTY:
            return board[a]
    if EMPTY not in board:
        return "DRAW"
    return None


def winning_line(board):
    """คืน (หมากที่ชนะ, (ช่อง a, b, c)) ของแถวที่ชนะ หรือ None ถ้ายังไม่มีใครชนะ"""
    for a, b, c in WIN_PATTERNS:
        if board[a] == board[b] == board[c] != EMPTY:
            return board[a], (a, b, c)
    return None


def count_pieces(board):
    return board.count(PLAYER), board.count(ROBOT)


def is_valid_board(board, player_first=True):
    """ตรวจว่าจำนวนหมากสมเหตุสมผลตามลำดับการเดิน"""
    if len(board) != 9 or any(c not in (PLAYER, ROBOT, EMPTY) for c in board):
        return False
    x, o = count_pieces(board)
    if player_first:
        return x == o or x == o + 1
    return o == x or o == x + 1


def is_robot_turn(board, player_first=True):
    """True เมื่อถึงตาหุ่นยนต์ (เรียกใช้กับกระดานที่ผ่าน is_valid_board แล้ว)"""
    x, o = count_pieces(board)
    if player_first:
        return x == o + 1
    return x == o


def print_board(board):
    rows = []
    for r in range(3):
        cells = []
        for c in range(3):
            i = r * 3 + c
            cells.append(board[i] if board[i] != EMPTY else str(i + 1))
        rows.append(" " + " | ".join(cells))
    print("\n---+---+---\n".join(rows))
