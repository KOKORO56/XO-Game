"""hardware.py — "สัญญา" (interface) ของกล้องและหุ่นยนต์ + ตัวจำลองสำหรับตอนที่ยังไม่มีอุปกรณ์จริง

แนวคิด: โค้ดส่วนอื่น (controller, UI) คุยกับกล้อง/หุ่นยนต์ผ่านเมธอดชุดเดียวกันเสมอ
    กล้อง   : read_board() -> list 9 ช่อง ("X" / "O" / "-")
    หุ่นยนต์ : reset_tray(), place_piece(cell, progress)
ตัวจำลอง: SimCamera / SimRobot · ของจริง: VisionCamera (vision.py) และ MG400Robot (robot.py)
ที่มีเมธอดชื่อเดียวกัน แล้วสลับตัวที่ส่งเข้า controller เท่านั้น
"""

import time
from game import EMPTY, ROBOT


class CameraBase:
    def read_board(self, expected_mark=None):
        """คืน list 9 ช่องที่อ่านได้จากกระดานจริง
        expected_mark = หมากของฝ่ายที่เพิ่งเดิน (กล้องจริงใช้ระบุหมากในช่องที่เพิ่งมีของ)"""
        raise NotImplementedError


class RobotBase:
    def connect(self, ip=None):
        """เชื่อมต่อหุ่นยนต์ (ตัวจำลองถือว่าเชื่อมต่อแล้วเสมอ)"""

    def disconnect(self):
        pass

    @property
    def is_connected(self):
        return True

    def reset_tray(self):
        """เริ่มเกมใหม่: ถาดหมากเต็ม"""
        raise NotImplementedError

    def place_piece(self, cell, progress=None):
        """หยิบหมาก O จากถาดไปวางที่ช่อง cell (0-8); progress(msg) ใช้รายงานความคืบหน้า"""
        raise NotImplementedError

    def go_park(self, progress=None):
        """ไปจุดจอดนอกมุมกล้อง (ตอนเริ่ม/จบเกม) · ตัวจำลองไม่ต้องทำอะไร · คืน True ถ้าขยับไปจอดแล้ว"""
        return False


# ---------------- ตัวจำลอง ----------------

class SimWorld:
    """แทน "กระดาษกระดานจริงบนโต๊ะ" — ผู้เล่นและหุ่นยนต์จำลองวางหมากลงที่นี่"""

    def __init__(self):
        self.board = [EMPTY] * 9

    def clear(self):
        self.board = [EMPTY] * 9

    def place(self, cell, mark):
        self.board[cell] = mark


class SimCamera(CameraBase):
    def __init__(self, world, delay=0.4):
        self.world = world
        self.delay = delay

    def read_board(self, expected_mark=None):
        time.sleep(self.delay)          # จำลองเวลาถ่ายภาพ + ประมวลผล
        return list(self.world.board)   # คืนสำเนา ไม่ใช่ตัวจริง


class SimRobot(RobotBase):
    def __init__(self, world, delay=0.4):
        self.world = world
        self.delay = delay
        self.slot = 0

    def reset_tray(self):
        self.slot = 0

    def place_piece(self, cell, progress=None):
        """หยิบหมาก O จากจุดหยิบเดิมทุกครั้ง แล้ววางที่ช่อง cell"""
        say = progress or (lambda msg: None)
        steps = [
            f"ไปเหนือจุดหยิบหมาก (ชิ้นที่ {self.slot + 1})",
            "ลดลงและหยิบหมาก",
            "ยกขึ้นที่ความสูงปลอดภัย",
            f"เคลื่อนไปเหนือช่อง {cell + 1}",
            "ลดลงและปล่อยหมาก",
        ]
        for msg in steps:
            say(msg)
            time.sleep(self.delay)
        self.world.place(cell, ROBOT)
        self.slot += 1
        say("กลับจุดจอดนอกมุมกล้อง")
        time.sleep(self.delay)
