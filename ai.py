"""ai.py — เลือกการเดินด้วย Minimax (มีคะแนนตามความลึก เพื่อให้ชนะเร็วที่สุด/แพ้ช้าที่สุด)"""

from functools import lru_cache

from game import EMPTY, PLAYER, ROBOT, check_winner

# ลำดับการลองช่อง: กลาง -> มุม -> ขอบ (ถ้าคะแนนเท่ากันจะเลือกช่องที่มาก่อน)
MOVE_ORDER = [4, 0, 2, 6, 8, 1, 3, 5, 7]

def minimax(board, is_robot_turn, depth=0):
    return _minimax(tuple(board), is_robot_turn, depth)


@lru_cache(maxsize=None)
def _minimax(board, is_robot_turn, depth):
    """คะแนนของกระดาน (tuple) · จำผลไว้ กระดานเดิมไม่ต้องคิดซ้ำ (ตาแรกจาก ~1 วิ เหลือแทบ 0)"""
    result = check_winner(list(board))
    if result == ROBOT:
        return 10 - depth
    if result == PLAYER:
        return depth - 10
    if result == "DRAW":
        return 0

    mark = ROBOT if is_robot_turn else PLAYER
    scores = []
    for i in MOVE_ORDER:
        if board[i] == EMPTY:
            scores.append(_minimax(board[:i] + (mark,) + board[i + 1:], not is_robot_turn, depth + 1))
    return max(scores) if is_robot_turn else min(scores)


def choose_move(board):
    """คืน index 0-8 ของช่องที่หุ่นยนต์ควรวาง หรือ None ถ้าเกมจบแล้ว"""
    if check_winner(board) is not None:
        return None
    best_score = float("-inf")
    best_move = None
    for i in MOVE_ORDER:
        if board[i] == EMPTY:
            board[i] = ROBOT
            score = minimax(board, False, 1)
            board[i] = EMPTY
            if score > best_score:
                best_score, best_move = score, i
    return best_move
