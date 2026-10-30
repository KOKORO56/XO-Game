"""test_game.py — รันด้วย: python -m unittest test_game -v"""

import unittest
from game import WIN_PATTERNS, check_winner, is_robot_turn, is_valid_board, new_board
from ai import choose_move


class TestWinner(unittest.TestCase):
    def test_all_win_patterns(self):
        for mark in "XO":
            for pattern in WIN_PATTERNS:
                b = new_board()
                for i in pattern:
                    b[i] = mark
                self.assertEqual(check_winner(b), mark, pattern)

    def test_draw(self):
        b = list("XOXXOOOXX")
        self.assertEqual(check_winner(b), "DRAW")

    def test_not_finished(self):
        self.assertIsNone(check_winner(new_board()))


class TestTurn(unittest.TestCase):
    def test_player_first(self):
        self.assertFalse(is_robot_turn(new_board(), player_first=True))
        b = new_board(); b[0] = "X"
        self.assertTrue(is_robot_turn(b, player_first=True))

    def test_robot_first(self):
        self.assertTrue(is_robot_turn(new_board(), player_first=False))

    def test_invalid_counts(self):
        b = new_board(); b[0] = b[1] = "X"
        self.assertFalse(is_valid_board(b, player_first=True))


class TestAI(unittest.TestCase):
    def test_example_from_guide_blocks(self):
        b = ["X", "X", "-", "O", "-", "-", "-", "-", "O"]
        self.assertEqual(choose_move(b), 2)  # ช่อง 3

    def test_takes_win_over_block(self):
        b = ["X", "X", "-", "O", "O", "-", "X", "-", "-"]
        self.assertEqual(choose_move(b), 5)  # ช่อง 6 ชนะเลย

    def _never_loses(self, board, robot_turn):
        result = check_winner(board)
        if result is not None:
            self.assertNotEqual(result, "X", f"AI แพ้: {board}")
            return
        if robot_turn:
            m = choose_move(board)
            board[m] = "O"
            self._never_loses(board, False)
            board[m] = "-"
        else:
            for i in range(9):  # ลองทุกการเดินที่ผู้เล่นทำได้
                if board[i] == "-":
                    board[i] = "X"
                    self._never_loses(board, True)
                    board[i] = "-"

    def test_never_loses_player_first(self):
        self._never_loses(new_board(), robot_turn=False)

    def test_never_loses_robot_first(self):
        self._never_loses(new_board(), robot_turn=True)


if __name__ == "__main__":
    unittest.main()
