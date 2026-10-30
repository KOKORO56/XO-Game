"""test_vision.py — ทดสอบการหาจุดมุมและดัดภาพกระดาน ด้วยภาพสังเคราะห์ (ไม่ต้องมีกล้อง)
รันด้วย: python -m unittest test_vision -v
"""

import unittest

import cv2
import numpy as np

import vision as v

PX = 3          # พิกเซลต่อ มม. ของภาพแผ่นสนามสังเคราะห์
PAD = 25        # ขอบกระดาษรอบจุดมุม (มม.)


def board_sheet():
    """วาดแผ่นสนามแบบเดียวกับ print/make_board.py (พื้นที่ -PAD .. BOARD+PAD มม.)"""
    size = int((v.BOARD_MM + 2 * PAD) * PX)
    img = np.full((size, size, 3), 245, np.uint8)
    mm = lambda a: int(round((a + PAD) * PX))
    for k in range(4):
        a = k * v.CELL_MM
        cv2.rectangle(img, (mm(a - v.LINE_MM / 2), mm(-v.LINE_MM / 2)),
                      (mm(a + v.LINE_MM / 2), mm(v.BOARD_MM + v.LINE_MM / 2)), (0, 0, 0), -1)
        cv2.rectangle(img, (mm(-v.LINE_MM / 2), mm(a - v.LINE_MM / 2)),
                      (mm(v.BOARD_MM + v.LINE_MM / 2), mm(a + v.LINE_MM / 2)), (0, 0, 0), -1)
    for cx, cy in v.MARK_CENTERS_MM:
        h = v.MARK_MM / 2
        cv2.rectangle(img, (mm(cx - h), mm(cy - h)), (mm(cx + h), mm(cy + h)), (0, 0, 0), -1)
    return img


def put_piece(sheet, cell, kind):
    """วางหมากจำลองบนแผ่นสนาม: kind = 'black' ก้อนดำ, 'orange' กากบาทสีส้ม, 'ring' วงสีเขียว"""
    r, c = divmod(cell, 3)
    cx = int((c * v.CELL_MM + v.CELL_MM / 2 + PAD) * PX)
    cy = int((r * v.CELL_MM + v.CELL_MM / 2 + PAD) * PX)
    s = int(16 * PX)
    if kind == "black":
        cv2.rectangle(sheet, (cx - s, cy - s), (cx + s, cy + s), (25, 25, 25), -1)
    elif kind in ("red", "blue"):      # บล็อกจริงในแล็บ: แดง = O, ฟ้า = X
        color = (45, 45, 210) if kind == "red" else (220, 140, 40)
        cv2.rectangle(sheet, (cx - s, cy - s), (cx + s, cy + s), color, -1)
    elif kind == "orange":
        cv2.line(sheet, (cx - s, cy - s), (cx + s, cy + s), (30, 90, 230), int(6 * PX))
        cv2.line(sheet, (cx - s, cy + s), (cx + s, cy - s), (30, 90, 230), int(6 * PX))
    else:
        cv2.circle(sheet, (cx, cy), s, (60, 160, 30), int(5 * PX))


def camera_scene(tilt=True, clutter=True, pieces=()):
    """วางแผ่นสนามบนโต๊ะ เอียงแบบมุมกล้อง และมีของสีดำรบกวน (เช่น แขนหุ่นยนต์)"""
    sheet = board_sheet()
    for cell, kind in pieces:
        put_piece(sheet, cell, kind)
    s = sheet.shape[0]
    frame = np.full((720, 1280, 3), (120, 150, 170), np.uint8)
    src = np.float32([[0, 0], [s, 0], [s, s], [0, s]])
    dst = (np.float32([[420, 110], [930, 140], [900, 650], [390, 620]]) if tilt
           else np.float32([[390, 110], [890, 110], [890, 610], [390, 610]]))
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(sheet, M, (1280, 720))
    mask = cv2.warpPerspective(np.full((s, s), 255, np.uint8), M, (1280, 720))
    frame[mask > 0] = warped[mask > 0]
    if clutter:
        cv2.rectangle(frame, (1000, 200), (1250, 500), (20, 20, 20), -1)   # แขนหุ่นยนต์
        cv2.circle(frame, (150, 600), 40, (10, 10, 10), -1)                # ปุ่ม E-Stop
        cv2.rectangle(frame, (80, 80), (100, 100), (0, 0, 0), -1)          # ก้อนดำเล็กหลอก
    return frame, M


def mm_to_frame(M, x, y):
    p = np.float32([[[(x + PAD) * PX, (y + PAD) * PX]]])
    return cv2.perspectiveTransform(p, M)[0, 0]


class TestVision(unittest.TestCase):
    def test_finds_markers_in_tilted_cluttered_scene(self):
        frame, M = camera_scene()
        markers = v.find_markers(frame)
        self.assertIsNotNone(markers)
        for found, (x, y) in zip(markers, v.MARK_CENTERS_MM):
            self.assertLess(np.linalg.norm(found - mm_to_frame(M, x, y)), 6)

    def test_warp_puts_grid_lines_in_the_right_place(self):
        frame, _ = camera_scene()
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        top = cv2.cvtColor(v.warp_board(frame, quad, 300), cv2.COLOR_BGR2GRAY)
        for k in (1, 2):                                 # เส้นแบ่งช่องต้องอยู่ที่ 1/3 และ 2/3
            self.assertLess(top[150, k * 100], 90)
            self.assertLess(top[k * 100, 150], 90)
        for r in range(3):                               # กลางทุกช่องต้องเป็นกระดาษขาว
            for c in range(3):
                self.assertGreater(top[r * 100 + 50, c * 100 + 50], 200)

    def test_no_board_returns_none_and_guide_fallback(self):
        frame = np.full((720, 1280, 3), 140, np.uint8)
        self.assertIsNone(v.find_markers(frame))
        quad = v.grid_quad((1280, 720), None)
        self.assertEqual(quad[0].tolist(), [352.0, 72.0])     # กรอบสีเขียว 80% กลางภาพ
        self.assertEqual(quad[2].tolist(), [928.0, 648.0])

    def test_detects_occupied_cells(self):
        frame, _ = camera_scene(pieces=[(0, "orange"), (4, "black"), (8, "ring")])
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        occ = v.occupied_cells(v.warp_board(frame, quad, 150))
        self.assertEqual([i for i, o in enumerate(occ) if o], [0, 4, 8])

    def test_empty_board_has_no_pieces(self):
        frame, _ = camera_scene()
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        self.assertFalse(any(v.occupied_cells(v.warp_board(frame, quad, 150))))

    def test_vision_camera_reads_new_piece_as_expected_mark(self):
        frame, _ = camera_scene(pieces=[(4, "black"), (2, "black")])     # หมากไม่มีสี: เดาจากตาเดิน
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        known = ["-", "-", "-", "-", "O", "-", "-", "-", "-"]          # ช่อง 5 เป็น O มาก่อนแล้ว
        cam = v.VisionCamera(lambda: frame, lambda: quad, lambda: known, samples=3, gap_s=0)
        self.assertEqual(cam.read_board("X"), ["-", "-", "X", "-", "O", "-", "-", "-", "-"])
        cam.override = (7, "X")                                        # คลิกบอกเองบนจอ
        self.assertEqual(cam.read_board("X")[7], "X")

    def test_rotation_90_numbers_from_bottom_left(self):
        """หมุน 90°: ช่องมุมซ้ายล่างของภาพ = ช่อง 1, ช่องซ้ายกลาง = ช่อง 2, มุมขวาบน = ช่อง 9"""
        frame, _ = camera_scene(tilt=False, pieces=[(6, "black"), (3, "orange"), (2, "ring")])
        markers = v.find_markers(frame)
        quad = v.grid_quad(frame.shape[1::-1], markers, rotation=90)
        occ = v.occupied_cells(v.warp_board(frame, quad, 150))
        self.assertEqual([i + 1 for i, o in enumerate(occ) if o], [1, 2, 9])

    def test_rotate_cells_keeps_physical_positions(self):
        old = [f"p{i}" for i in range(9)]          # p6 = ช่องมุมซ้ายล่างของภาพ (เลข 7 เดิม)
        new = v.rotate_cells(old)
        self.assertEqual(new[0], "p6")            # ช่อง 1 ใหม่ = ช่องจริงเดิมของเลข 7
        self.assertEqual(new[1], "p3")            # ช่อง 2 ใหม่ = ช่องจริงเดิมของเลข 4
        self.assertEqual(v.rotate_cells(v.rotate_cells(v.rotate_cells(new))), old)   # ครบ 360°

    def test_block_colors_red_is_O_blue_is_X(self):
        frame, _ = camera_scene(pieces=[(0, "red"), (1, "blue"), (4, "black")])
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        marks = v.cell_marks(v.warp_board(frame, quad, 150))
        self.assertEqual(marks[0], "O")
        self.assertEqual(marks[1], "X")
        self.assertIsNone(marks[4])                   # สีดำ บอกไม่ได้ว่าเป็นของใคร
        self.assertIsNone(marks[8])                   # ช่องว่าง

    def test_vision_camera_uses_block_color(self):
        """เปิดแยกสี: ตาผู้เล่นแต่เห็นบล็อกแดง → อ่านเป็น O (controller แจ้งว่าผิดสี)
        ปิดแยกสี (ค่าเริ่มต้น): บล็อกสีอะไรก็ได้ เป็นหมากของฝ่ายที่กำลังเดิน"""
        frame, _ = camera_scene(pieces=[(3, "red")])
        quad = v.grid_quad(frame.shape[1::-1], v.find_markers(frame))
        cam = v.VisionCamera(lambda: frame, lambda: quad, lambda: ["-"] * 9, samples=3, gap_s=0)
        self.assertEqual(cam.read_board("X")[3], "X")      # ค่าเริ่มต้น: ไม่ดูสี
        cam.use_colors = True
        self.assertEqual(cam.read_board("X")[3], "O")

    def test_hand_detection(self):
        """มือยื่นเข้ามาจากขอบกระดาน = เห็นมือ · บล็อกแดง/ฟ้ากลางช่อง = ไม่ใช่มือ"""
        top = np.full((150, 150, 3), 235, np.uint8)
        cv2.rectangle(top, (60, 60), (90, 90), (40, 40, 200), -1)       # บล็อกแดง
        cv2.rectangle(top, (10, 10), (40, 40), (200, 90, 30), -1)       # บล็อกฟ้า
        self.assertFalse(v.hand_in_view(top))
        cv2.rectangle(top, (95, 0), (125, 30), (130, 160, 210), -1)     # นิ้วมือ (สีผิว) จากขอบบน
        self.assertTrue(v.hand_in_view(top))

    def test_ignores_row_of_dark_block_holders(self):
        """ช่องเสียบบล็อกสีดำวางเรียงข้างกระดาน (แบบในแล็บ) ต้องไม่ถูกจับเป็นจุดมุม"""
        frame, M = camera_scene()
        for k in range(5):                              # แถวช่องเสียบ ขนาดใหญ่กว่าจุดมุมมาก วางชิดกัน
            x = 60 + k * 62
            cv2.rectangle(frame, (x, 300), (x + 54, 354), (25, 25, 25), -1)
            cv2.rectangle(frame, (x, 380), (x + 54, 434), (25, 25, 25), -1)
        markers = v.find_markers(frame)
        self.assertIsNotNone(markers)
        for found, (x, y) in zip(markers, v.MARK_CENTERS_MM):
            self.assertLess(np.linalg.norm(found - mm_to_frame(M, x, y)), 6)

    def test_block_holders_alone_are_not_a_board(self):
        frame = np.full((720, 1280, 3), (170, 180, 185), np.uint8)
        for r in range(3):
            for k in range(3):
                cv2.rectangle(frame, (400 + k * 70, 200 + r * 70), (450 + k * 70, 250 + r * 70),
                              (25, 25, 25), -1)
        self.assertIsNone(v.find_markers(frame))

    def test_order_corners(self):
        pts = [[10, 90], [90, 90], [10, 10], [90, 10]]
        self.assertEqual(v.order_corners(pts).tolist(),
                         [[10, 10], [90, 10], [90, 90], [10, 90]])


if __name__ == "__main__":
    unittest.main()
