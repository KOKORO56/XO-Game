"""test_controller.py — ทดสอบลำดับเกมทั้งระบบด้วยกล้อง/หุ่นยนต์จำลอง
รันด้วย: python -m unittest test_controller -v
"""

import random
import unittest
from controller import GameController, Match, rule_violation
from hardware import SimWorld, SimCamera, SimRobot


class FailingRobot(SimRobot):
    """จำลองหุ่นยนต์วางหมากผิดช่อง"""
    def place_piece(self, cell, progress=None):
        wrong = next(i for i in range(9) if self.world.board[i] == "-" and i != cell)
        self.world.place(wrong, "O")


def make(robot_cls=SimRobot):
    world = SimWorld()
    events = []
    ctrl = GameController(SimCamera(world, delay=0), robot_cls(world, delay=0),
                          on_event=lambda k, m, d: events.append((k, m)))
    return world, ctrl, events


class TestController(unittest.TestCase):
    def play_random(self, player_first, seed):
        rng = random.Random(seed)
        world, ctrl, _ = make()
        ctrl.new_game(player_first)
        ctrl.start()
        while ctrl.result is None:
            free = [i for i in range(9) if world.board[i] == "-"]
            world.place(rng.choice(free), "X")
            ctrl.player_done()
        return ctrl.result

    def test_robot_never_loses_random_games(self):
        for seed in range(15):
            for first in (True, False):
                self.assertNotEqual(self.play_random(first, seed), "X")

    def test_rejects_two_pieces_at_once(self):
        world, ctrl, events = make()
        ctrl.new_game(True)
        world.place(0, "X")
        world.place(1, "X")
        ctrl.player_done()
        self.assertEqual(events[-1], ("rule", "วางเกิน 1 ตัว"))
        self.assertEqual(world.board.count("O"), 0)   # หุ่นยนต์ต้องไม่ขยับ
        world.board[1] = "-"                          # แก้บนกระดานแล้ว เกมเล่นต่อได้
        ctrl.player_done()
        self.assertEqual(world.board.count("O"), 1)

    def test_rejects_no_new_piece(self):
        world, ctrl, events = make()
        ctrl.new_game(True)
        ctrl.player_done()
        self.assertEqual(events[-1], ("rule", "ยังไม่ได้วางหมาก"))

    def test_parks_at_start_and_end(self):
        """ทุกเกม: แขนไปจุดจอดตอนเริ่ม และตอนจบเกม (ไม่ว่าใครชนะ)"""
        parks = []

        class ParkRobot(SimRobot):
            def go_park(self, progress=None):
                parks.append(list(self.world.board))
                return True

        world, ctrl, events = make(ParkRobot)
        ctrl.new_game(True)
        ctrl.start()
        self.assertEqual(len(parks), 1)                 # เริ่มเกม
        while ctrl.result is None:
            world.place(next(i for i in range(9) if world.board[i] == "-"), "X")
            ctrl.player_done()
        self.assertEqual(len(parks), 2)                 # จบเกม
        self.assertEqual(events[-1][0], "game_over")    # ไปจอดก่อน แล้วจึงประกาศผล

    def test_player_places_next_x_early(self):
        """ผู้เล่นวาง X ตาถัดไปทันที ระหว่างแขนกลับจุดจอด (ก่อนกล้องยืนยัน O):
        ต้องไม่ขึ้น 'วางเกิน 1 ตัว' · ยืนยัน O ได้ และนับ X นั้นเป็นตาถัดไปของผู้เล่น"""
        class EagerPlayerRobot(SimRobot):
            def place_piece(self, cell, progress=None):
                super().place_piece(cell, progress)
                free = next(i for i in range(9) if self.world.board[i] == "-")
                self.world.place(free, "X")          # ผู้เล่นวางเร็ว

        world, ctrl, events = make(EagerPlayerRobot)
        ctrl.new_game(True)
        world.place(4, "X")
        ctrl.player_done()
        kinds = [k for k, _ in events]
        self.assertIn("robot_moved", kinds)
        self.assertNotIn("error", kinds)
        self.assertNotIn("rule", kinds)
        self.assertEqual(ctrl.board.count("O"), 1)
        self.assertEqual(ctrl.board.count("X"), 1)         # X ที่วางเร็วยังไม่นับ จนกว่าจะถึงตาผู้เล่น
        ctrl.player_done()                                  # ตาผู้เล่น: กล้องเห็น X ตัวใหม่ 1 ตัว = ถูกต้อง
        self.assertEqual(ctrl.board.count("X"), 2)
        self.assertNotIn("rule", [k for k, _ in events])

    def test_rule_messages(self):
        old = ["X", "O", "-", "-", "-", "-", "-", "-", "-"]
        def msg(*changes):
            new = list(old)
            for i, v in changes:
                new[i] = v
            return rule_violation(old, new, "X")
        self.assertIsNone(msg((4, "X")))
        self.assertEqual(msg((0, "-")), "หมากช่อง 1 หายไป")
        self.assertEqual(msg((1, "X")), "หมากช่อง 2 ถูกเปลี่ยน")
        self.assertEqual(msg((4, "O")), "ต้องวางสีฟ้า")

    def test_detects_robot_wrong_cell(self):
        world, ctrl, events = make(FailingRobot)
        ctrl.new_game(True)
        world.place(0, "X")
        ctrl.player_done()
        self.assertEqual(events[-1][0], "error")


class TestMatch(unittest.TestCase):
    def play(self, *results):
        m = Match()
        for r in results:
            m.add(r)
        return m

    def test_two_wins_ends_after_two(self):
        m = self.play("O", "O")
        self.assertTrue(m.finished)
        self.assertEqual(m.winner, "O")

    def test_win_and_draw_ends_after_two(self):
        m = self.play("DRAW", "X")
        self.assertTrue(m.finished)
        self.assertEqual(m.winner, "X")

    def test_tie_after_two_needs_third(self):
        for first_two in (("X", "O"), ("DRAW", "DRAW")):
            m = self.play(*first_two)
            self.assertFalse(m.finished)
            self.assertEqual(m.game_number, 3)

    def test_third_game_decides(self):
        self.assertEqual(self.play("X", "O", "O").winner, "O")
        self.assertEqual(self.play("DRAW", "DRAW", "DRAW").winner, "DRAW")
        with self.assertRaises(ValueError):
            self.play("X", "O", "DRAW").add("X")


if __name__ == "__main__":
    unittest.main()
