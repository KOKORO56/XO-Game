"""controller.py — ควบคุมลำดับเกมทั้งหมด (ตาม flowchart) โดยไม่สนว่ากล้อง/หุ่นยนต์เป็นของจริงหรือจำลอง

ส่งความคืบหน้าออกไปทาง on_event(kind, message, data) ให้ UI หรือ console แสดงผล
kind ที่ใช้: step, log, player_moved, ai_move, robot_moved, wait_player, game_over,
rule (ผู้เล่นผิดกติกา · เกมรอให้แก้), error (ระบบผิดพลาด · เกมหยุด)
"""

from game import EMPTY, PLAYER, ROBOT, check_winner
from ai import choose_move

STEPS = ["รอผู้เล่น", "อ่านกระดาน", "ตรวจสถานะ", "AI เลือกช่อง", "หยิบและวาง", "ตรวจยืนยัน"]


class Match:
    """แข่งแบบ 2–3 เกม: เล่น 2 เกมก่อน ถ้าจำนวนเกมที่ชนะเท่ากัน (รวมเสมอทั้งคู่) จึงเล่นเกมที่ 3"""

    MAX_GAMES = 3

    def __init__(self):
        self.results = []          # ผลแต่ละเกม: "X", "O" หรือ "DRAW"

    def add(self, result):
        if self.finished:
            raise ValueError("แมตช์จบแล้ว")
        self.results.append(result)

    def wins(self, mark):
        return self.results.count(mark)

    @property
    def game_number(self):
        """เกมที่กำลังเล่นหรือจะเล่นต่อไป (เริ่มที่ 1)"""
        return len(self.results) + 1

    @property
    def finished(self):
        n = len(self.results)
        if n < 2:
            return False
        if n == 2:
            return self.wins(PLAYER) != self.wins(ROBOT)
        return True

    @property
    def winner(self):
        """'X', 'O', 'DRAW' หรือ None ถ้ายังไม่จบ"""
        if not self.finished:
            return None
        x, o = self.wins(PLAYER), self.wins(ROBOT)
        return PLAYER if x > o else ROBOT if o > x else "DRAW"


class GameError(Exception):
    """สถานะกระดานผิดปกติ — ต้องหยุดและให้คนตรวจ ห้ามสั่งหุ่นยนต์ต่อ"""


class RuleError(GameError):
    """ผู้เล่นทำผิดกติกา: แจ้งเตือนสั้น ๆ แล้วรอให้แก้บนกระดาน"""


MARK_NAME = {PLAYER: "สีฟ้า", ROBOT: "สีแดง"}


def rule_violation(old, new, expected_mark):
    """ข้อความสั้น ๆ บอกว่าผิดกติกาข้อไหน · None = ถูกต้อง (มีหมากใหม่ 1 ตัว ถูกสี หมากเดิมไม่ถูกแตะ)"""
    moved = [i for i in range(9) if old[i] != EMPTY and new[i] != old[i]]
    if moved:
        i = moved[0]
        return f"หมากช่อง {i + 1} {'หายไป' if new[i] == EMPTY else 'ถูกเปลี่ยน'}"
    added = [i for i in range(9) if old[i] == EMPTY and new[i] != EMPTY]
    if not added:
        return "ยังไม่ได้วางหมาก"
    if len(added) > 1:
        return "วางเกิน 1 ตัว"
    i = added[0]
    if new[i] != expected_mark:
        return f"ต้องวาง{MARK_NAME.get(expected_mark, expected_mark)}"
    return None


class GameController:
    def __init__(self, camera, robot, on_event=None):
        self.camera = camera
        self.robot = robot
        self.on_event = on_event or (lambda kind, msg, data: print(f"[{kind}] {msg}"))
        self.player_first = True
        self.board = [EMPTY] * 9
        self.result = None

    # ---------- ช่วยส่งเหตุการณ์ ----------
    def emit(self, kind, msg="", **data):
        self.on_event(kind, msg, data)

    def step(self, index, msg):
        self.emit("step", msg, index=index)

    # ---------- เริ่มเกม ----------
    def new_game(self, player_first=True):
        self.player_first = player_first
        self.board = [EMPTY] * 9
        self.result = None
        self.robot.reset_tray()

    def park(self, why):
        """ส่งแขนไปจุดจอดนอกมุมกล้อง · ไปไม่ได้ (ยังไม่เชื่อมต่อ/ไม่ Enable) แค่บันทึกไว้ ไม่หยุดเกม"""
        try:
            if self.robot.go_park(progress=lambda m: self.emit("log", m)):
                self.emit("log", f"แขนอยู่ที่จุดจอดนอกมุมกล้องแล้ว ({why})")
        except (RuntimeError, OSError) as e:
            self.emit("log", f"ไปจุดจอดไม่สำเร็จ ({why}): {e}")

    def start(self):
        """เรียกหลัง new_game() — แขนไปจุดจอดก่อนเสมอ แล้วถ้าหุ่นยนต์เริ่มก่อนจะเดินทันที"""
        self.park("เริ่มเกม")
        try:
            if self.player_first:
                self.wait_player()
            else:
                self.robot_turn()
        except GameError as e:
            self.emit("error", str(e))

    def wait_player(self):
        self.step(0, "ตาผู้เล่น: วางหมาก X แล้วยืนยัน")
        self.emit("wait_player", "ตาผู้เล่น")

    # ---------- ตาผู้เล่น ----------
    def player_done(self):
        """เรียกเมื่อผู้เล่นวางหมากเสร็จ (กด Enter / คลิกในหน้า UI)"""
        try:
            new_board = self.read_and_validate(PLAYER)
        except RuleError as e:       # ผิดกติกา: แจ้งสั้น ๆ แล้วรอให้แก้บนกระดาน (เกมไม่หยุด)
            self.emit("rule", str(e))
            return
        except GameError as e:
            self.emit("error", str(e))
            return
        try:
            cell = self.changed_cell(new_board)
            self.board = new_board
            self.emit("player_moved", f"ผู้เล่นวางช่อง {cell + 1}", cell=cell)
            if self.check_end():
                return
            self.robot_turn()
        except GameError as e:
            self.emit("error", str(e))

    # ---------- ตาหุ่นยนต์ ----------
    def robot_turn(self):
        self.step(3, "AI กำลังคิด (Minimax)")
        move = choose_move(list(self.board))
        self.emit("ai_move", f"AI เลือกช่อง {move + 1}", cell=move)

        self.step(4, f"หุ่นยนต์หยิบหมากไปวางช่อง {move + 1}")
        try:
            self.robot.place_piece(move, progress=lambda m: self.emit("log", m))
        except (RuntimeError, OSError) as e:
            raise GameError(f"หุ่นยนต์ทำงานไม่สำเร็จ: {e}")

        self.step(5, "ถ่ายภาพยืนยันผลการวาง")
        self.board = self.confirm_robot_move(move)
        self.emit("robot_moved", f"ยืนยันแล้ว: O อยู่ช่อง {move + 1}", cell=move)
        if not self.check_end():
            self.wait_player()

    def confirm_robot_move(self, move):
        """ยืนยันตาหุ่นยนต์: ดูแค่ว่ามี O ในช่องที่สั่ง และหมากเดิมยังอยู่ครบ
        ถ้าผู้เล่นวาง X ตาถัดไปไว้แล้วระหว่างแขนกลับจุดจอด จะไม่นับเป็นผิดกติกา
        (X นั้นถูกนับเป็นตาถัดไปของผู้เล่นตามปกติ)"""
        try:
            seen = self.camera.read_board(ROBOT)
        except RuntimeError as e:
            raise GameError(f"อ่านกระดานจากกล้องไม่ได้: {e}")
        if len(seen) != 9:
            raise GameError("ข้อมูลจากกล้องไม่ถูกต้อง")
        if seen[move] != ROBOT:
            raise GameError(f"หมาก O ไม่อยู่ช่อง {move + 1} ตามที่สั่ง")
        gone = [i + 1 for i in range(9) if self.board[i] != EMPTY and seen[i] != self.board[i]]
        if gone:
            raise GameError(f"หมากเดิมช่อง {', '.join(map(str, gone))} หายหรือถูกเปลี่ยน")
        board = list(self.board)
        board[move] = ROBOT
        return board

    # ---------- อ่านและตรวจสถานะ ----------
    def read_and_validate(self, expected_mark, step_index=1):
        if step_index == 1:
            self.step(1, "กล้องอ่านกระดาน")
        try:
            new_board = self.camera.read_board(expected_mark)
        except RuntimeError as e:          # เช่น ไม่มีภาพจากกล้อง
            raise GameError(f"อ่านกระดานจากกล้องไม่ได้: {e}")
        if step_index == 1:
            self.step(2, "ตรวจสถานะกระดาน")
        self.validate(new_board, expected_mark)
        return new_board

    def changed_cell(self, new_board):
        return next(i for i in range(9) if new_board[i] != self.board[i])

    def validate(self, new_board, expected_mark):
        if len(new_board) != 9 or any(c not in (PLAYER, ROBOT, EMPTY) for c in new_board):
            raise GameError("ข้อมูลจากกล้องไม่ถูกต้อง")
        err = rule_violation(self.board, new_board, expected_mark)
        if err:   # ตาผู้เล่น = ผิดกติกา (แก้แล้วเล่นต่อได้) · ตาหุ่นยนต์ = ระบบผิดพลาด
            raise (RuleError if expected_mark == PLAYER else GameError)(err)

    def check_end(self):
        result = check_winner(self.board)
        if result is None:
            return False
        self.result = result
        self.park("จบเกม")
        text = {"X": "ผู้เล่นชนะ", "O": "หุ่นยนต์ชนะ", "DRAW": "เสมอ"}[result]
        self.emit("game_over", f"จบเกม: {text}", result=result)
        return True
