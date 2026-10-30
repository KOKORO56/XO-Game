"""vision.py — หากระดานจากภาพกล้อง (ชั้นที่ 2 ข้าง robot.py)

ตอนนี้ทำได้:
  find_markers(frame)   หาสี่เหลี่ยมดำ 4 มุมของแผ่นสนาม (จาก print/make_board.py)
  grid_quad(...)        มุมทั้ง 4 ของตาราง 3x3 ในภาพ (จากจุดมุม หรือจากกรอบสีเขียวถ้าหาไม่เจอ)
  warp_board(...)       ดัดภาพให้เป็นกระดานมองจากด้านบนตรง ๆ ขนาดจัตุรัส
  occupied_cells(top)   ช่องไหนมีหมาก (จากภาพที่ดัดแล้ว)
  VisionCamera          กล้องจริงสำหรับ GameController: อ่านกระดานแทนการคลิกบอกเอง
"""

import itertools
import time

import cv2
import numpy as np

from game import EMPTY, ROBOT
from hardware import CameraBase

# ---- รูปทรงแผ่นสนาม (มม.) ต้องตรงกับ print/make_board.py ----
CELL_MM = 55
LINE_MM = 3
MARK_MM = 8
GAP_MM = 5
BOARD_MM = CELL_MM * 3
_OFF = LINE_MM / 2 + GAP_MM + MARK_MM / 2          # ระยะจากมุมตารางถึงกึ่งกลางจุดมุม
MARK_CENTERS_MM = np.float32([[-_OFF, -_OFF], [BOARD_MM + _OFF, -_OFF],
                              [BOARD_MM + _OFF, BOARD_MM + _OFF], [-_OFF, BOARD_MM + _OFF]])
GRID_MM = np.float32([[0, 0], [BOARD_MM, 0], [BOARD_MM, BOARD_MM], [0, BOARD_MM]])


MAX_CANDIDATES = 24     # จำนวนก้อนสี่เหลี่ยมดำที่นำมาลองจับคู่เป็นจุดมุม


def order_corners(pts):
    """เรียง 4 จุดเป็น ซ้ายบน, ขวาบน, ขวาล่าง, ซ้ายล่าง"""
    pts = np.float32(pts)
    s, d = pts.sum(axis=1), np.diff(pts, axis=1).ravel()
    return np.float32([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]])


def _candidates(frame, work_width):
    """จุดศูนย์กลางของก้อนสีดำที่เป็นรูปสี่เหลี่ยมจัตุรัสเล็ก ๆ (ผู้สมัครเป็นจุดมุม)"""
    h, w = frame.shape[:2]
    scale = work_width / w if w > work_width else 1.0
    small = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale < 1 else frame
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 51, 15)
    contours, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = gray.shape[0] * gray.shape[1]
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (frame_area * 0.00003 < area < frame_area * 0.01):
            continue
        approx = cv2.approxPolyDP(c, 0.08 * cv2.arcLength(c, True), True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue
        (cx, cy), (rw, rh), _ = cv2.minAreaRect(c)
        if min(rw, rh) == 0 or max(rw, rh) / min(rw, rh) > 1.6 or area / (rw * rh) < 0.75:
            continue
        # ต้องเป็นก้อนทึบ ไม่ใช่กรอบกลวง (เช่น ช่องตาราง)
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [c], -1, 255, -1)
        if cv2.mean(th, mask=mask)[0] < 200:
            continue
        found.append((cx / scale, cy / scale, area / (scale * scale)))
    # ก้อนซ้ำซ้อน (ขอบในขอบนอก) เก็บแค่ก้อนใหญ่สุด
    found.sort(key=lambda f: -f[2])
    unique = []
    for f in found:
        if all(np.hypot(f[0] - u[0], f[1] - u[1]) > 4 for u in unique):
            unique.append(f)
    # เก็บได้มากพอ ไม่ให้ก้อนใหญ่ (บล็อก/ช่องเสียบสีดำ) เบียดจุดมุมจริงที่เล็กกว่าออกไป
    return unique[:MAX_CANDIDATES]


def find_markers(frame, work_width=1280):
    """หาจุดมุม 4 จุดของแผ่นสนาม คืน array 4x2 (ซ้ายบน ขวาบน ขวาล่าง ซ้ายล่าง) หรือ None
    ถ้ามองเห็นแค่ 3 มุม (เช่น แขนหุ่นยนต์บังอยู่ 1 มุม) จะคำนวณมุมที่ 4 ให้"""
    quad, _ = find_markers_info(frame, work_width)
    return quad


MIN_GRID_SCORE = 35    # ช่องต้องสว่างกว่าเส้นตารางอย่างน้อยเท่านี้ (ระดับเทา 0-255)


def find_markers_info(frame, work_width=1280):
    """เหมือน find_markers แต่คืน (มุม, จำนวนมุมที่มองเห็นจริง 0/3/4)
    ทุกชุดที่รูปทรงเข้าเค้าจะถูกตรวจว่าข้างในมีตาราง 3x3 จริง แล้วเลือกชุดที่คะแนนดีที่สุด"""
    cands = _candidates(frame, work_width)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    best, best_score, best_seen = None, MIN_GRID_SCORE, 0
    for seen, quads in ((4, _quads_of_four(frame, cands)), (3, _quads_of_three(frame, cands))):
        for quad in quads:
            # เห็นครบ 4 มุมน่าเชื่อกว่า: 3 มุมต้องได้คะแนนตารางสูงกว่าชัดเจนถึงจะชนะ
            score = grid_score(gray, quad) - (20 if seen == 3 else 0)
            if score > best_score:
                best, best_score, best_seen = quad, score, seen
    return best, best_seen


def grid_score(gray, markers, n=120):
    """ดัดภาพตามมุมที่เสนอ แล้ววัดว่า 'ช่องสว่าง เส้นมืด' แค่ไหน (มากยิ่งเหมือนตาราง XO)"""
    quad = grid_quad(gray.shape[1::-1], markers)
    top = warp_board(gray, quad, n).astype(np.float32)
    c, b, m = n // 3, 3, 6
    # เส้นบาง: ตามแนวเส้นแต่ละจุด เอาค่าที่มืดที่สุดในแถบกว้าง ±b พิกเซล (เผื่อตำแหน่งคลาดเล็กน้อย)
    lines = np.concatenate([top[m:n - m, k * c - b:k * c + b + 1].min(axis=1) for k in (1, 2)] +
                           [top[k * c - b:k * c + b + 1, m:n - m].min(axis=0) for k in (1, 2)])
    cells = [np.median(top[r * c + 10:(r + 1) * c - 10, k * c + 10:(k + 1) * c - 10])
             for r in range(3) for k in range(3)]
    # ค่ากลางของช่อง และค่ากลางของเส้น (ส่วนที่ถูกแขนหรือหมากบังไม่ทำให้ผลเพี้ยนมาก)
    return float(np.median(cells) - np.median(lines))


# ขนาดจุดมุม เทียบกับระยะห่างระหว่างจุดมุม (8 มม. ต่อ ~186 มม. ≈ 0.043) ยอมให้คลาดได้ 2 เท่า
MARK_RATIO = MARK_MM / (BOARD_MM + 2 * _OFF)


def _size_ok(areas, side_lengths):
    """จุดที่เลือกต้องเล็กพอเทียบกับระยะห่าง แบบจุดมุมจริง
    กันการจับบล็อก/ช่องเสียบบล็อกสีดำที่วางชิดกันมาเป็นมุมกระดาน (ก้อนใหญ่ ระยะห่างสั้น)"""
    ratio = np.mean(np.sqrt(areas)) / max(np.mean(side_lengths), 1)
    return MARK_RATIO * 0.5 <= ratio <= MARK_RATIO * 2


def _quads_of_three(frame, cands):
    """3 จุดที่เป็นมุมฉากและด้านยาวเท่ากัน = 3 มุมของจัตุรัส · มุมที่ 4 = A + C - B"""
    h, w = frame.shape[:2]
    for combo in itertools.combinations(cands, 3):
        areas = [c[2] for c in combo]
        if max(areas) / min(areas) > 3:
            continue
        pts = [np.float32(c[:2]) for c in combo]
        for i in range(3):                      # ลองให้แต่ละจุดเป็นมุมฉาก B
            b, a, c = pts[i], pts[(i + 1) % 3], pts[(i + 2) % 3]
            ba, bc = a - b, c - b
            la, lc = np.linalg.norm(ba), np.linalg.norm(bc)
            if min(la, lc) < 1 or max(la, lc) / min(la, lc) > 1.4:
                continue
            if abs(np.dot(ba, bc) / (la * lc)) > 0.35:          # ต้องใกล้ 90 องศา
                continue
            if not _size_ok(areas, [la, lc]):
                continue
            quad = order_corners([a, b, c, a + c - b])
            if cv2.contourArea(quad) > w * h * 0.01:      # กระดานต้องกินพื้นที่อย่างน้อย 1% ของภาพ
                yield quad


def _quads_of_four(frame, cands):
    h, w = frame.shape[:2]
    for combo in itertools.combinations(cands, 4):
        areas = [c[2] for c in combo]
        if max(areas) / min(areas) > 3:              # จุดมุมขนาดเท่ากัน
            continue
        quad = order_corners([(c[0], c[1]) for c in combo])
        if not cv2.isContourConvex(quad.reshape(-1, 1, 2)):
            continue
        sides = [np.linalg.norm(quad[i] - quad[(i + 1) % 4]) for i in range(4)]
        diag = [np.linalg.norm(quad[0] - quad[2]), np.linalg.norm(quad[1] - quad[3])]
        if max(sides) / max(min(sides), 1) > 1.6 or max(diag) / max(min(diag), 1) > 1.3:
            continue                                 # แผ่นสนามเป็นจัตุรัส (เอียงได้นิดหน่อย)
        if not _size_ok(areas, sides):
            continue
        if cv2.contourArea(quad) > w * h * 0.01:
            yield quad


def grid_quad(frame_size, markers=None, rotation=0):
    """มุมทั้ง 4 ของตาราง 3x3 ในพิกัดภาพ เรียงตามเลขช่อง: มุมช่อง 1, มุมช่อง 3, มุมช่อง 9, มุมช่อง 7
    มีจุดมุม: คำนวณจากรูปทรงแผ่นสนามจริง · ไม่มี: ใช้กรอบสีเขียว 80% กลางภาพ (เหมือน camera_test.py)
    rotation = หมุนเลขช่อง 0/90/180/270 องศา (90 = ช่อง 1 อยู่มุมซ้ายล่างของภาพ ช่อง 2 อยู่เหนือมัน)"""
    if markers is not None:
        H = cv2.getPerspectiveTransform(MARK_CENTERS_MM, np.float32(markers))
        quad = cv2.perspectiveTransform(GRID_MM.reshape(-1, 1, 2), H).reshape(4, 2)
    else:
        fw, fh = frame_size
        size = min(fw, fh) * 0.8
        x0, y0 = (fw - size) / 2, (fh - size) / 2
        quad = np.float32([[x0, y0], [x0 + size, y0], [x0 + size, y0 + size], [x0, y0 + size]])
    return np.roll(quad, (int(rotation) // 90) % 4, axis=0)


def rotate_cells(values):
    """จัดลำดับข้อมูล 9 ช่องใหม่เมื่อหมุนเลขช่องเพิ่ม 90° (ข้อมูลยังชี้ช่องจริงเดิมบนกระดาษ)
    ช่องที่เคยเป็นเลข (แถว r, คอลัมน์ c) จะกลายเป็นเลข c*3 + (2-r)"""
    out = [None] * 9
    for old in range(9):
        r, c = divmod(old, 3)
        out[c * 3 + (2 - r)] = values[old]
    return out


def warp_board(frame, quad, size=360):
    """ภาพกระดานมองจากด้านบน ขนาด size x size (ขอบภาพ = ขอบตาราง แต่ละช่องกว้าง size/3)"""
    dst = np.float32([[0, 0], [size, 0], [size, size], [0, size]])
    M = cv2.getPerspectiveTransform(np.float32(quad), dst)
    return cv2.warpPerspective(frame, M, (size, size))


# ---------------- ตรวจว่าช่องไหนมีหมาก ----------------
FILL_THRESHOLD = 0.12   # ช่องที่มีพื้นที่ "ไม่ใช่กระดาษ" เกินสัดส่วนนี้ = มีหมาก
CELL_MARGIN = 0.2       # ตัดขอบช่องออก 20% ทุกด้าน (เลี่ยงเส้นตาราง และเลขช่องสีเทา)


def cell_fill(top):
    """สัดส่วนพิกเซลที่ไม่ใช่กระดาษขาว ในพื้นที่ตรงกลางของแต่ละช่อง (9 ค่า 0..1)
    ไม่ใช่กระดาษ = มืดกว่ากระดาษชัดเจน หรือมีสีสด (ใช้ได้ทั้งหมากสีดำและหมากสี)"""
    gray = cv2.cvtColor(top, cv2.COLOR_BGR2GRAY).astype(np.float32)
    sat = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)[..., 1]
    paper = max(np.percentile(gray, 75), 60)            # กระดาษเป็นพื้นที่ส่วนใหญ่ของกระดาน
    ink = (gray < paper * 0.65) | ((sat > 80) & (gray > 40))
    n = top.shape[0]
    c, m = n / 3, n / 3 * CELL_MARGIN
    fills = []
    for r in range(3):
        for k in range(3):
            region = ink[int(r * c + m):int((r + 1) * c - m), int(k * c + m):int((k + 1) * c - m)]
            fills.append(float(region.mean()) if region.size else 0.0)
    return fills


def occupied_cells(top, threshold=FILL_THRESHOLD):
    """ช่องไหนมีหมาก (True/False 9 ค่า) จากภาพกระดานที่ดัดแล้ว"""
    return [f > threshold for f in cell_fill(top)]


# ---------------- แยกหมากจากสี ----------------
PIECE_COLORS = {"X": "blue", "O": "red"}      # บล็อกสีฟ้า = X (ผู้เล่น) · บล็อกสีแดง = O (หุ่นยนต์)
HUE_RANGES = {                                # ช่วงสี (Hue) ของ OpenCV 0-179
    "red": ((0, 10), (165, 180)), "orange": ((10, 22),), "yellow": ((22, 35),),
    "green": ((35, 85),), "blue": ((85, 130),), "purple": ((130, 165),),
}
COLOR_MIN_FRACTION = 0.08     # ต้องมีพิกเซลสีสดอย่างน้อยเท่านี้ของพื้นที่กลางช่อง ถึงจะดูสี


def piece_centers(top):
    """จุดกึ่งกลางของหมากในแต่ละช่อง (u, v เป็นสัดส่วน 0..1 ของทั้งกระดาน) หรือ None ถ้าช่องว่าง
    ใช้วางป้าย X / O ให้ทับบนตัวบล็อกจริง ถึงบล็อกจะไม่ได้วางกลางช่องพอดี"""
    gray = cv2.cvtColor(top, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    paper = max(np.percentile(gray, 75), 60)
    mask = (gray < paper * 0.65) | ((hsv[..., 1] > 80) & (hsv[..., 2] > 50))
    n = top.shape[0]
    c, m = n / 3, n / 3 * 0.12
    centers = []
    for r in range(3):
        for k in range(3):
            y0, x0 = int(r * c + m), int(k * c + m)
            ys, xs = np.nonzero(mask[y0:int((r + 1) * c - m), x0:int((k + 1) * c - m)])
            centers.append(((x0 + xs.mean()) / n, (y0 + ys.mean()) / n) if len(xs) > 15 else None)
    return centers


def hand_in_view(top, edge_area=0.003, big_area=0.03):
    """มีมืออยู่บนภาพกระดานไหม (สีผิวในช่อง Cr/Cb)
    มือ = ก้อนสีผิวที่ยื่นเข้ามาจากขอบกระดาน (แตะขอบภาพ) หรือก้อนใหญ่มาก
    ขอบบาง ๆ รอบบล็อกแดงที่สีคล้ายผิว อยู่กลางช่อง ไม่แตะขอบ และเล็ก จึงไม่นับ"""
    ycc = cv2.cvtColor(top, cv2.COLOR_BGR2YCrCb)
    mask = cv2.inRange(ycc, (60, 138, 85), (255, 172, 128))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))   # ตัดเส้นบาง/จุดเล็ก
    h, w = mask.shape
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    for x, y, bw, bh, area in stats[1:]:
        share = area / float(h * w)
        touches_edge = x <= 2 or y <= 2 or x + bw >= w - 2 or y + bh >= h - 2
        if (touches_edge and share >= edge_area) or share >= big_area:
            return True
    return False


def cell_marks(top, colors=PIECE_COLORS):
    """อ่านสีหมากในแต่ละช่อง คืน 9 ค่า: 'X' / 'O' / None (ไม่มีสีสดพอจะบอกได้)"""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    n = top.shape[0]
    c, m = n / 3, n / 3 * CELL_MARGIN
    marks = []
    for r in range(3):
        for k in range(3):
            region = hsv[int(r * c + m):int((r + 1) * c - m), int(k * c + m):int((k + 1) * c - m)]
            vivid = (region[..., 1] > 80) & (region[..., 2] > 50)
            if not region.size or vivid.mean() < COLOR_MIN_FRACTION:
                marks.append(None)
                continue
            hues = region[..., 0][vivid]
            best, best_frac = None, 0.5               # สีหลักต้องเกินครึ่งของพิกเซลสีสด
            for mark, color in colors.items():
                frac = sum(((hues >= lo) & (hues < hi)).mean() for lo, hi in HUE_RANGES[color])
                if frac > best_frac:
                    best, best_frac = mark, frac
            marks.append(best)
    return marks


class VisionCamera(CameraBase):
    """กล้องจริงสำหรับ GameController (แทน SimCamera)

    get_frame()   คืนภาพล่าสุดจากกล้อง (หรือ None)
    get_quad()    คืนมุมทั้ง 4 ของตารางในภาพ (จาก find_markers หรือกรอบสีเขียว)
    known_board() คืนกระดานที่ยืนยันแล้วล่าสุด (หมากเดิมถือว่ายังอยู่ ถึงจะถูกบังชั่วขณะ)

    read_board(expected_mark) อ่านหลายเฟรมแล้วโหวต ช่องที่ว่างอยู่เดิมแต่ตอนนี้มีหมาก
    จะใส่เป็น expected_mark (ตาใครเดินก็เป็นหมากของคนนั้น) · ตัวตรวจกติกาอยู่ที่ controller
    """

    def __init__(self, get_frame, get_quad, known_board, samples=5, gap_s=0.12, size=150):
        self.get_frame, self.get_quad, self.known_board = get_frame, get_quad, known_board
        self.samples, self.gap_s, self.size = samples, gap_s, size
        self.override = None      # (ช่อง, หมาก) ใช้ครั้งเดียว เมื่อผู้เล่นคลิกบอกเองบนจอ
        # True = แยก X/O จากสีบล็อก (ฟ้า/แดง) · False = หมากใหม่เป็นของฝ่ายที่กำลังเดิน (บล็อกสีอะไรก็ได้)
        self.use_colors = False
        # คืนกระดานจำลองเมื่อหุ่นยนต์ไม่ได้วางหมากจริง (dry run) หรือ None = ใช้ภาพจริง
        self.virtual_robot_board = lambda: None

    def observe(self):
        """อ่านหลายเฟรมแล้วโหวต คืน (มีหมากไหม 9 ค่า, สีบอกว่าเป็น X/O/None 9 ค่า)"""
        votes, used = [0] * 9, 0
        mark_votes = [{} for _ in range(9)]
        for i in range(self.samples):
            frame = self.get_frame()
            if frame is not None:
                top = warp_board(frame, self.get_quad(), self.size)
                for k, (o, mk) in enumerate(zip(occupied_cells(top), cell_marks(top))):
                    votes[k] += o
                    if mk:
                        mark_votes[k][mk] = mark_votes[k].get(mk, 0) + 1
                used += 1
            if i < self.samples - 1:
                time.sleep(self.gap_s)
        if used == 0:
            raise RuntimeError("ไม่มีภาพจากกล้อง")
        occ = [v * 2 > used for v in votes]
        marks = [max(mv, key=mv.get) if mv and max(mv.values()) * 2 > used else None
                 for mv in mark_votes]
        return occ, marks

    def occupancy(self):
        return self.observe()[0]

    def read_board(self, expected_mark=None):
        board = list(self.known_board())
        if self.override is not None:
            cell, mark = self.override
            self.override = None
            board[cell] = mark
            return board
        if expected_mark == ROBOT:
            virtual = self.virtual_robot_board()
            if virtual is not None:
                return virtual
        occ, marks = self.observe()
        for i in range(9):
            if board[i] == EMPTY and (occ[i] or marks[i]):
                # สีบอกได้ = เชื่อสี (วางผิดสี controller จะแจ้งเตือน) · บอกไม่ได้ = ถือว่าเป็นหมากของฝ่ายที่เดิน
                if self.use_colors:
                    board[i] = marks[i] or expected_mark or "?"
                else:
                    board[i] = expected_mark or marks[i] or "?"
        return board
