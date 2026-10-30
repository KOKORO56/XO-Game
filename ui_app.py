"""ui_app.py — หน้าจอเล่น XO กับ AI + แผงเชื่อมต่อหุ่นยนต์ Dobot MG400

รัน:  python ui_app.py
- คลิกช่องว่าง = วางหมาก X (เหมือนวางหมากจริงแล้วกด Enter)
- โหมด "จำลอง"   : ไม่ต้องมีหุ่นยนต์
- โหมด "MG400 จริง": กด "เชื่อมต่อ" ก่อน ค่าเริ่มต้นเป็น dry run (ไม่ส่งคำสั่งเคลื่อนที่)
  กระดานด้านซ้ายจะเปลี่ยนเป็นภาพสดจากกล้อง วางกระดาษให้ตรงกรอบสีเขียว
  (ยังไม่มี vision.py จึงยังต้องคลิกช่องบนภาพเพื่อบอกว่าผู้เล่นวางหมากช่องไหน)
- แท็บ "ตั้งค่า / สอบเทียบ": ตั้งพิกัดหุ่นยนต์และบันทึกลง robot_config.json
"""

import ipaddress
import os
from collections import deque
import queue
import random
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont, messagebox, ttk

import cv2
import numpy as np

from game import EMPTY, PLAYER, ROBOT, winning_line
from controller import GameController, Match, STEPS, rule_violation
from hardware import SimWorld, SimCamera, SimRobot
from robot import (MG400Robot, MG400Error, MAX_REACH_MM, CONFIG_FILE, save_config, load_config,
                   grid_from_corners, out_of_reach, low_points)
from camera_test import open_camera, find_cameras
from vision import (find_markers_info, grid_quad, warp_board, occupied_cells, cell_marks, hand_in_view,
                    piece_centers, rotate_cells, VisionCamera)

RULE_STABLE_TICKS = 8  # เห็นการผิดกติกาเดิมติดกันกี่รอบ (~1.6 วิ) ก่อนแจ้ง (กันมือที่ผ่านไปมา)
MOTION_STILL = 4.0     # ภาพกระดานเปลี่ยนเฉลี่ยน้อยกว่าเท่านี้ (0–255) = ไม่มีมือ/ของเคลื่อนไหวอยู่
STILL_TICKS = 4        # ต้องนิ่งติดกันกี่รอบ (~0.8 วิ) ก่อนนับว่าเอามือออกจากกระดานแล้ว
AUTO_STABLE_TICKS = 5  # ต้องเห็นหมากใหม่ช่องเดิมติดกันกี่รอบ (~0.2 วิ/รอบ) ก่อนถือว่าผู้เล่นวางเสร็จ
LOCK_SAMPLES = 3        # ล็อกตำแหน่งกระดานเมื่อหาจุดมุมได้ตรงกันกี่ครั้งติด
LOCK_TOL_PX = 8         # จุดมุมแต่ละครั้งต้องห่างจากค่าเฉลี่ยไม่เกินกี่พิกเซล
MARKER_HOLD_S = 1.0  # ใช้ตำแหน่งมุมล่าสุดต่ออีกเท่านี้ ถ้าเฟรมถัดมาหาไม่เจอ (เช่น แขนบังชั่วขณะ)

CELL = 120                         # ขนาดช่องอ้างอิงสำหรับคำนวณขนาดตัวอักษรหมาก
RIGHT_W = 420                     # ความกว้างคอลัมน์ขวา (px ที่ 100%) กระดานใช้พื้นที่ที่เหลือ
CAM_AUTO = "อัตโนมัติ"

# ---------------- ธีมสี ----------------
BG, CARD, BORDER = "#F1F4F8", "#FFFFFF", "#DCE1E8"
TEXT, MUTED = "#1E293B", "#64748B"
HEADER, HEADER_MUTED = "#0F172A", "#94A3B8"
PRIMARY, PRIMARY_SOFT = "#2563EB", "#DBEAFE"
SUCCESS, WARN, DANGER = "#16A34A", "#D97706", "#DC2626"
SOFT = "#F8FAFC"
COLOR_X, COLOR_O, COLOR_ON = "#EA580C", "#059669", PRIMARY
COLOR_GUIDE = "#00C853"
LAMP = {"off": "#94A3B8", "ok": SUCCESS, "busy": WARN, "error": DANGER}
TONE = {"info": PRIMARY, "ok": SUCCESS, "warn": WARN, "error": DANGER}

# ปุ่มแต่ละแบบ: (พื้น, ตัวอักษร, พื้นตอนเมาส์ชี้)
BUTTON = {
    "primary": (PRIMARY, "white", "#1D4ED8"),
    "secondary": ("#E8EDF3", TEXT, "#D9E0E8"),
    "success": ("#DCFCE7", "#15803D", "#BBF7D0"),
    "danger": ("#FEE2E2", "#B91C1C", "#FECACA"),
}
BUTTON_OFF = ("#EEF1F5", "#A0AEC0")

RESULT_TEXT = {"X": "ผู้เล่นชนะ", "O": "หุ่นยนต์ชนะ", "DRAW": "เสมอ"}
RESULT_COLOR = {"X": COLOR_X, "O": COLOR_O, "DRAW": MUTED}
MATCH_TEXT = {"X": "ผู้เล่นชนะแมตช์", "O": "หุ่นยนต์ชนะแมตช์", "DRAW": "แมตช์เสมอ"}
MIN_APPROACH_MM = 20   # ความสูง Approach ควรสูงกว่าจุดหยิบ/วางทุกจุดอย่างน้อยเท่านี้
CELL_Z_SPREAD_MM = 15  # Z ของช่องใดต่างจากค่ากลางของทุกช่องเกินเท่านี้ = เตือนว่าน่าจะสอนพลาด
BOARD_MOVED_PX = 15   # จุดมุมกระดานเลื่อนจากตอนสอบเทียบเกินกี่พิกเซล = เตือนว่ากระดานถูกขยับ
SINGLE_POINTS = ("park_xyz",)                    # ค่าพิกัดที่เป็นจุดเดียว ไม่ใช่รายการ
POINT_LISTS = {"cell_xyz": 9, "pick_xyz": 5}    # รายการจุดที่สอน และจำนวนจุด
POINT_NAMES = {"cell_xyz": "ช่อง", "pick_xyz": "จุดหยิบ", "park_xyz": "จุดจอด"}
NEXT_GAME_DELAY_S = 3    # โหมดจำลอง: รอกี่วินาทีก่อนเริ่มเกมถัดไปในแมตช์


class LiveCamera:
    """อ่านภาพจากกล้องใน thread แยก เก็บไว้เฉพาะภาพล่าสุด"""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.index = None
        self.error = ""
        self._stop = threading.Event()
        self._thread = None

    def start(self, index=None):
        """index=None คือสแกนหากล้องเอง แล้วใช้ตัวท้ายสุด (เหมือน camera_test.py)"""
        self.stop()
        self.index, self.error = index, ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(index, self._stop), daemon=True)
        self._thread.start()

    def _run(self, index, stop):
        if index is None:
            cams = find_cameras()
            if stop.is_set():
                return
            if not cams:
                self.error = "ไม่พบกล้อง ตรวจสาย USB หรือปิดโปรแกรมอื่นที่ใช้กล้องอยู่"
                return
            index = self.index = cams[-1]
        cap = open_camera(index)
        if not cap.isOpened():
            self.error = f"เปิดกล้องหมายเลข {index} ไม่ได้"
            return
        while not stop.is_set():
            ok, frame = cap.read()
            if not ok:
                self.error = f"อ่านภาพจากกล้องหมายเลข {index} ไม่ได้"
                break
            with self.lock:
                self.frame = frame
        cap.release()

    def stop(self):
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=1.5)
            self._thread = None
        with self.lock:
            self.frame = None

    def latest(self):
        with self.lock:
            return self.frame


def grid_geometry(cw, ch, frame_size, camera):
    """มุมซ้ายบนและขนาดต่อช่องของตาราง 3x3 บน canvas ขนาด cw x ch
    camera=True: กรอบจัตุรัส 80% ของด้านสั้นของภาพกล้อง ตรงกับเส้นนำใน camera_test.py"""
    if camera:
        fw, fh = frame_size
        size = int(min(fw, fh) * min(cw / fw, ch / fh) * 0.8)
    else:
        size = int(min(cw, ch) * 0.94)
    cell = max(1, size // 3)
    return (cw - cell * 3) // 2, (ch - cell * 3) // 2, cell


def pick_font(root):
    families = set(tkfont.families(root))
    for name in ("Leelawadee UI", "Tahoma", "Thonburi", "Noto Sans Thai", "Sarabun"):
        if name in families:
            return name
    return "TkDefaultFont"


class XOApp:
    def __init__(self, root):
        self.root = root
        root.title("AI XO — Dobot MG400")
        # ไอคอนหน้าต่าง/แถบงาน · รันจาก .exe ไฟล์ไอคอนอยู่ในโฟลเดอร์ที่ PyInstaller แตกไว้ (sys._MEIPASS)
        icon = os.path.join(getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__))),
                            "app_icon.ico")
        try:
            root.iconbitmap(icon)
        except tk.TclError:
            pass
        root.configure(bg=BG)
        self.ui_scale = max(1.0, root.winfo_fpixels("1i") / 96)   # 1.25 = จอขยาย 125%
        f = self.font_family = pick_font(root)
        self.f_body, self.f_small = (f, 10), (f, 9)
        self.f_head, self.f_title = (f, 11, "bold"), (f, 15, "bold")
        self.f_status, self.f_piece = (f, 13, "bold"), (f, 40, "bold")
        self.f_mono = ("Consolas", 10)

        self.events = queue.Queue()
        emit = lambda k, m, d: self.events.put((k, m, d))

        # ---- อุปกรณ์ (ยังไม่มี vision.py จึงใช้กล้องจำลองอ่านกระดานเสมอ) ----
        self.world = SimWorld()
        camera = SimCamera(self.world, delay=0.4)
        self.sim_robot = SimRobot(self.world, delay=0.35)
        self.mg400 = MG400Robot(dry_run=True, sim_world=self.world,
                                log=lambda m: emit("log", m, {}))
        self.ctrl = GameController(camera, self.sim_robot, on_event=emit)
        self.sim_camera = camera
        # กล้องจริง: อ่านกระดานจากภาพ (ใช้เมื่อเลือก MG400 จริง + เปิดกล้อง + เปิดตรวจจับอัตโนมัติ)
        # dry run: หุ่นยนต์ไม่ได้วางหมากจริง จึงเชื่อกระดานจำลองแทนตอนตรวจตาหุ่นยนต์
        self.vcam = VisionCamera(lambda: self.cam.latest(), lambda: self.current_quad(),
                                 lambda: list(self.ctrl.board))
        self.vcam.virtual_robot_board = lambda: list(self.world.board) if self.mg400.dry_run else None
        self.vcam.use_colors = bool(self.mg400.cfg.get("piece_colors", False))
        self.auto_cand, self.auto_count = None, 0
        self.rule_cand, self.rule_count, self.rule_msg = None, 0, None   # คำเตือนผิดกติกาที่แสดงอยู่

        self.busy = False
        self.polling = False
        self.poll_failures = 0       # อ่านสถานะหุ่นยนต์พลาดติดกันกี่ครั้ง
        self.score = {"X": 0, "O": 0, "DRAW": 0}
        self.match = None            # Match() เมื่อเลือกแข่ง 2–3 เกม
        self.waiting_next = False    # จบเกมในแมตช์แล้ว รอเริ่มเกมถัดไป
        self.next_job = None         # ตัวนับถอยหลังก่อนเกมถัดไป

        # ---- ภาพกล้อง (แสดงเฉพาะโหมด MG400 จริง) ----
        self.cam = LiveCamera()
        self.camera_view = False
        self.cw, self.ch = 480, 320          # ขนาด canvas กระดาน (อัปเดตเมื่อปรับขนาดหน้าต่าง)
        self.frame_size = (1280, 720)        # ขนาดภาพจากกล้องล่าสุด
        self.highlight = None
        # ---- ผลการหากระดานจากภาพ (vision.py) ----
        self.markers, self.markers_seen, self.markers_time = None, 0, 0.0
        self.detecting, self.frame_count = False, 0
        self.locked_markers, self.lock_candidates = None, []   # ตำแหน่งกระดานที่ล็อกไว้
        self.board_moved, self.move_checks = False, 0           # กระดานเลื่อนจากตอนสอบเทียบหรือไม่
        # ---- สิ่งที่กล้องเห็นบนกระดานจริงตอนนี้ (อัปเดตเรียลไทม์) ----
        self.live_marks, self.live_centers = [None] * 9, [None] * 9
        self.live_hist = deque(maxlen=3)
        self.prev_gray, self.still_ticks = None, 0     # ตรวจว่ามีมือเคลื่อนไหวเหนือกระดานไหม
        self.hand_seen = False                         # เห็นสีผิว (มือ) บนกระดานอยู่ไหม

        self.build_ui()
        self.set_view(False)
        self.show_idle()                # ยังไม่เริ่มเกม รอผู้ใช้กดปุ่มเริ่มเกม

        # เปิดเต็มหน้าจอ (ขยายหน้าต่าง) · F11 = เต็มจอแบบไม่มีขอบ · Esc = ออกจากเต็มจอ
        root.minsize(int(1000 * self.ui_scale), int(700 * self.ui_scale))
        try:
            root.state("zoomed")                  # Windows
        except tk.TclError:
            root.attributes("-zoomed", True)      # Linux
        root.bind("<F11>", lambda e: self.toggle_fullscreen())
        root.bind("<Escape>", lambda e: self.toggle_fullscreen(False))
        self.root.after(50, self.poll_events)
        self.root.after(2000, self.poll_robot_status)
        self.root.after(33, self.update_camera)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.cam.stop()
        self.mg400.disconnect()   # ปล่อยพอร์ตของหุ่นยนต์ทันที เปิดโปรแกรมใหม่จะได้เชื่อมต่อได้
        self.root.destroy()

    # ================= ชิ้นส่วนหน้าจอที่ใช้ซ้ำ =================
    def label(self, parent, text="", font=None, fg=TEXT, **kw):
        return tk.Label(parent, text=text, font=font or self.f_body, fg=fg,
                        bg=parent.cget("bg"), **kw)

    def card(self, parent, title=None, note=None):
        """กล่องพื้นขาวมีเส้นขอบ ใส่ของลงใน .body"""
        outer = tk.Frame(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=BORDER)
        if title:
            head = tk.Frame(outer, bg=CARD, padx=14, pady=10)
            head.pack(fill="x")
            outer.title = self.label(head, title, self.f_head)
            outer.title.pack(side="left")
            outer.note = self.label(head, note or "", self.f_small, MUTED)
            outer.note.pack(side="right")
            tk.Frame(outer, bg=BORDER, height=1).pack(fill="x")
        outer.body = tk.Frame(outer, bg=CARD, padx=14, pady=10)
        outer.body.pack(fill="both", expand=True)
        return outer

    def button(self, parent, text, command, kind="secondary", small=False, **kw):
        bg, fg, hover = BUTTON[kind]
        b = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                      activebackground=hover, activeforeground=fg,
                      disabledforeground=BUTTON_OFF[1], relief="flat", bd=0, cursor="hand2",
                      font=self.f_small if small else self.f_body,
                      padx=8 if small else 14, pady=2 if small else 6, **kw)
        b.kind = kind
        b.bind("<Enter>", lambda e: str(b["state"]) != "disabled" and b.config(bg=hover))
        b.bind("<Leave>", lambda e: str(b["state"]) != "disabled" and b.config(bg=bg))
        return b

    @staticmethod
    def enable_button(b, on):
        b.config(state="normal" if on else "disabled",
                 bg=BUTTON[b.kind][0] if on else BUTTON_OFF[0],
                 cursor="hand2" if on else "arrow")

    def entry(self, parent, width=8, font=None, justify="right"):
        return tk.Entry(parent, width=width, font=font or self.f_small, justify=justify,
                        relief="flat", bg="white", fg=TEXT, insertbackground=TEXT,
                        disabledbackground="#F1F5F9", disabledforeground=MUTED,
                        highlightthickness=1, highlightbackground=BORDER, highlightcolor=PRIMARY)

    def segmented(self, parent, var, options, command=None):
        """ปุ่มเลือกแบบแถบติดกัน เช่น [ จำลอง | MG400 จริง ]"""
        bar = tk.Frame(parent, bg=BORDER, padx=1, pady=1)
        for value, text in options:
            tk.Radiobutton(bar, text=text, variable=var, value=value, command=command,
                           indicatoron=0, font=self.f_body, bg="#F1F5F9", fg=TEXT,
                           selectcolor=PRIMARY_SOFT, activebackground="#E2E8F0",
                           relief="flat", offrelief="flat", bd=0, padx=14, pady=5,
                           cursor="hand2").pack(side="left", padx=(0, 1))
        return bar

    # ================= สร้างหน้าจอ =================
    def build_ui(self):
        self.build_header()

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(20, 10, 20, 0),
                        bordercolor=BG, lightcolor=BG, darkcolor=BG)
        style.configure("TNotebook.Tab", font=self.f_body, padding=(18, 7), background=BG,
                        foreground=MUTED, borderwidth=0, lightcolor=BG, bordercolor=BORDER)
        style.map("TNotebook.Tab", background=[("selected", CARD)],
                  foreground=[("selected", PRIMARY)], lightcolor=[("selected", CARD)])
        tabs = self.tabs = ttk.Notebook(self.root)
        tabs.pack(fill="both", expand=True)
        main = tk.Frame(tabs, bg=BG, padx=20, pady=16)
        settings = tk.Frame(tabs, bg=BG)
        tabs.add(main, text="หน้าหลัก")
        tabs.add(settings, text="ตั้งค่า / สอบเทียบ")
        page = self.scrollable(settings)
        self.build_settings(page)   # สร้างก่อน เพราะแผงหุ่นยนต์หน้าหลักอัปเดตป้ายตำแหน่งในหน้านี้
        tabs.bind("<<NotebookTabChanged>>",
                  lambda e: self.settings_visible() and self.draw_preview())

        # คอลัมน์ซ้ายยืดตามขนาดหน้าต่าง คอลัมน์ขวากว้างคงที่
        main.columnconfigure(0, weight=1)
        main.rowconfigure(0, weight=1)
        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky="nsew")
        right = tk.Frame(main, bg=BG, width=int(RIGHT_W * self.ui_scale))
        right.grid(row=0, column=1, sticky="ns", padx=(16, 0))
        right.pack_propagate(False)

        # ---- ซ้าย: [กระดานจากกล้อง | กระดานเกม] / แถบสถานะ / ควบคุมเกม / ประวัติการเดิน ----
        # (ภาพกล้องเต็มพร้อมจุดมุม อยู่ในหน้าตั้งค่า · หน้านี้เหลือแค่ปุ่มเลือก/เปิดกล้อง)
        self.left = left
        for col in range(2):
            left.columnconfigure(col, weight=1, uniform="view")
        left.rowconfigure(0, weight=1)

        # กระดานจากกล้อง: ดัดภาพแผ่นสนามให้เป็นมุมมองจากด้านบน (vision.py) + ตัวเลือกกล้อง
        self.warp_card = self.card(left, "กระดานจากกล้อง", "")
        self.warp_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.cam_card = self.warp_card                  # สถานะกล้องแสดงที่หัวการ์ดนี้
        self.cam_toggle = self.button(self.warp_card.title.master, "เปิดกล้อง", self.toggle_camera,
                                      small=True)
        self.cam_toggle.pack(side="right", padx=(8, 0), before=self.warp_card.note)
        body = self.warp_card.body
        self.cam_row = tk.Frame(body, bg=CARD, pady=4)
        self.cam_row.pack(side="bottom", fill="x")
        self.label(self.cam_row, "กล้องหมายเลข", self.f_small, MUTED).pack(side="left")
        saved = self.mg400.cfg.get("camera_index")
        self.cam_choice = tk.StringVar(value=CAM_AUTO if saved is None else str(saved))
        tk.Spinbox(self.cam_row, values=(CAM_AUTO, "0", "1", "2", "3", "4"), width=8,
                   textvariable=self.cam_choice, state="readonly", font=self.f_small,
                   relief="flat", readonlybackground="white", buttonbackground="#E8EDF3",
                   highlightthickness=1, highlightbackground=BORDER,
                   command=self.on_camera_change).pack(side="left", padx=8)
        self.cam_status = self.label(self.cam_row, "", self.f_small, MUTED)
        self.cam_status.pack(side="left")
        self.button(self.cam_row, "หาตำแหน่งกระดานใหม่", self.relock, small=True).pack(side="right")
        self.cam_moved_btn = self.button(self.cam_row, "กล้องขยับ · ใช้ตำแหน่งใหม่", self.accept_camera_moved,
                                         "primary", small=True)
        self.cam_moved_btn.pack(side="right", padx=(0, 6))
        self.cam_moved_btn.pack_forget()          # แสดงเฉพาะตอนเตือนว่ากระดานขยับ
        self.accept_new_view = False
        self.auto_detect = tk.BooleanVar(value=True)
        tk.Checkbutton(body, text="กล้องตรวจหมากที่ผู้เล่นวางเอง (ไม่ต้องคลิกบนจอ) · ใช้กับ MG400 จริง",
                       variable=self.auto_detect, font=self.f_small, bg=CARD, fg=TEXT,
                       activebackground=CARD, selectcolor="white", bd=0, cursor="hand2",
                       command=self.on_detect_change).pack(side="bottom", anchor="w")
        self.piece_colors = tk.BooleanVar(value=bool(self.mg400.cfg.get("piece_colors", False)))
        tk.Checkbutton(body, text="แยก X/O จากสีบล็อก (ฟ้า = X, แดง = O) · ไม่ติ๊ก = ใช้บล็อกสีอะไรก็ได้ ดูจากตาเดิน",
                       variable=self.piece_colors, font=self.f_small, bg=CARD, fg=TEXT,
                       activebackground=CARD, selectcolor="white", bd=0, cursor="hand2",
                       command=self.on_colors_change).pack(side="bottom", anchor="w")
        self.warp_canvas = tk.Canvas(body, width=240, height=200, bg="#0B1220",
                                     highlightthickness=0, cursor="hand2")
        self.warp_canvas.pack(fill="both", expand=True)
        self.warp_img = self.warp_canvas.create_image(0, 0, anchor="center")
        self.warp_msg = self.warp_canvas.create_text(0, 0, text="", fill="white", font=self.f_small,
                                                     width=220, justify="center")
        self.warp_photo = None
        self.warp_w, self.warp_h = 240, 200
        self.warp_canvas.bind("<Configure>", self.on_warp_resize)
        self.warp_canvas.bind("<Button-1>", lambda e: self.on_click(e, view="warp"))

        # กระดานเกม: สถานะเกมปัจจุบัน คลิกช่องเพื่อบอกว่าผู้เล่นวาง X ช่องไหน
        self.board_card = self.card(left, "กระดานเกม", "คลิกช่องที่วางหมาก X")
        self.board_card.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        self.canvas = tk.Canvas(self.board_card.body, width=320, height=200, bg="white",
                                highlightthickness=0, cursor="hand2")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Configure>", self.on_canvas_resize)

        # แถบสถานะ (เต็มความกว้าง)
        banner = self.status_banner = tk.Frame(left, bg=CARD, highlightthickness=1,
                                               highlightbackground=BORDER)
        banner.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        self.status_bar = tk.Frame(banner, bg=PRIMARY, width=4)
        self.status_bar.pack(side="left", fill="y")
        inner = tk.Frame(banner, bg=CARD, padx=14, pady=8)
        inner.pack(side="left", fill="x", expand=True)
        self.status = self.label(inner, "", self.f_status, anchor="w")
        self.status.pack(fill="x")
        self.error = self.label(inner, "", self.f_small, DANGER, anchor="w", justify="left",
                                wraplength=700)
        self.error.pack(fill="x")
        banner.bind("<Configure>", lambda e: self.error.config(wraplength=max(200, e.width - 60)))

        # ควบคุมเกม + คะแนน
        game_holder = tk.Frame(left, bg=BG)
        game_holder.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        self.build_game_card(game_holder)

        # ประวัติการเดินของคนและ AI
        self.build_history(left)

        # ---- คอลัมน์ขวา ----
        self.build_robot_panel(right)

        box = self.card(right, "ขั้นตอนการทำงาน")
        box.pack(fill="x", pady=(14, 0))
        self.step_items = []
        for i, name in enumerate(STEPS):
            cell = tk.Frame(box.body, bg=CARD, pady=3)
            cell.grid(row=i // 2, column=i % 2, sticky="w", padx=(0, 18))
            dot = tk.Canvas(cell, width=22, height=22, bg=CARD, highlightthickness=0)
            circle = dot.create_oval(1, 1, 21, 21, fill="#E2E8F0", outline="")
            num = dot.create_text(11, 11, text=str(i + 1), fill=MUTED, font=self.f_small)
            dot.pack(side="left")
            lb = self.label(cell, name, self.f_small, MUTED, width=12, anchor="w")
            lb.pack(side="left", padx=(6, 0))
            self.step_items.append((dot, circle, num, lb))

        box = self.card(right, "บันทึกการทำงาน")
        box.pack(fill="both", expand=True, pady=(14, 0))
        holder = tk.Frame(box.body, bg=SOFT, highlightthickness=1, highlightbackground=BORDER)
        holder.pack(fill="both", expand=True)
        self.log = tk.Text(holder, width=40, height=6, font=self.f_small, state="disabled",
                           relief="flat", bd=0, bg=SOFT, fg=TEXT, padx=8, pady=6, wrap="word")
        scroll = tk.Scrollbar(holder, command=self.log.yview, relief="flat", width=10)
        self.log.config(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        self.log.tag_config("time", foreground="#94A3B8", font=("Consolas", 9))
        self.log.tag_config("error", foreground=DANGER)
        self.log.tag_config("ok", foreground=SUCCESS)

    def build_game_card(self, parent):
        """การ์ดควบคุมเกม: ตัวเลือกอยู่แถวเดียว คะแนนแบบกะทัดรัด เพื่อเหลือพื้นที่ให้กระดาน"""
        game = self.card(parent, None)
        game.pack(fill="x")
        body = game.body
        # แถวว่าง 4 แถว ของจริงจะถูกจัดลงแถวเหล่านี้ตามความกว้างการ์ด (ดู layout_game_card)
        self.game_lines = [tk.Frame(body, bg=CARD) for _ in range(4)]

        self.opt_first = tk.Frame(body, bg=CARD)
        self.label(self.opt_first, "เริ่มก่อน", self.f_small, MUTED).pack(side="left", padx=(0, 8))
        self.first = tk.StringVar(value="rand")
        self.segmented(self.opt_first, self.first, (("p", "ผู้เล่น (X)"), ("r", "หุ่นยนต์ (O)"),
                                                    ("rand", "สุ่ม"))).pack(side="left")
        self.opt_mode = tk.Frame(body, bg=CARD)
        self.label(self.opt_mode, "รูปแบบ", self.f_small, MUTED).pack(side="left", padx=(0, 8))
        self.play_mode = tk.StringVar(value="single")
        self.segmented(self.opt_mode, self.play_mode, (("single", "เกมเดียว"),
                                                       ("match", "แข่ง 2–3 เกม (อัตโนมัติ)")),
                       self.on_play_mode_change).pack(side="left")
        self.start_btn = self.button(body, "เริ่มเกมใหม่", self.on_start_button, "primary")

        # คะแนนรวมทุกเกม
        tiles = self.score_tiles = tk.Frame(body, bg=CARD)
        self.score_vals = {}
        for key, name, color in (("X", "ผู้เล่นชนะ", COLOR_X), ("O", "หุ่นยนต์ชนะ", COLOR_O),
                                 ("DRAW", "เสมอ", MUTED)):
            tile = tk.Frame(tiles, bg=SOFT, highlightthickness=1, highlightbackground=BORDER,
                            padx=10, pady=2)
            tile.pack(side="left", padx=(8, 0))
            self.score_vals[key] = self.label(tile, "0", (self.font_family, 14, "bold"), color)
            self.score_vals[key].pack(side="left")
            self.label(tile, name, self.f_small, MUTED).pack(side="left", padx=(6, 0))
        self.update_score()

        # ผลแต่ละเกมในแมตช์: แสดงเฉพาะตอนเลือกแข่ง 2–3 เกม
        self.match_row = tk.Frame(body, bg=CARD)
        self.match_title = self.label(self.match_row, "", self.f_small, MUTED)
        self.match_title.pack(side="left", padx=(0, 10))
        self.match_pills = []
        for i in range(Match.MAX_GAMES):
            lb = tk.Label(self.match_row, text="", font=self.f_small, bg=SOFT, fg=MUTED,
                          padx=8, pady=3, highlightthickness=1, highlightbackground=BORDER)
            lb.pack(side="left", padx=(0, 6))
            self.match_pills.append(lb)

        self.game_wide = None
        body.bind("<Configure>", lambda e: self.layout_game_card())

    def layout_game_card(self, force=False):
        """จอกว้าง: 2 แถว (ตัวเลือก+ปุ่ม / ผลแมตช์+คะแนน)  จอแคบ: แยกเป็น 4 แถว"""
        body = self.opt_first.master
        show_match = self.match is not None
        need = max(self.opt_first.winfo_reqwidth() + self.opt_mode.winfo_reqwidth()
                   + self.start_btn.winfo_reqwidth() + 40,
                   (self.match_row.winfo_reqwidth() if show_match else 0)
                   + self.score_tiles.winfo_reqwidth() + 20)
        wide = body.winfo_width() >= need
        if wide == self.game_wide and not force:
            return
        self.game_wide = wide
        l1, l2, l3, l4 = self.game_lines
        for w in (self.opt_first, self.opt_mode, self.start_btn, self.match_row,
                  self.score_tiles, *self.game_lines):
            w.pack_forget()
        if wide:
            self.opt_first.pack(in_=l1, side="left")
            self.opt_mode.pack(in_=l1, side="left", padx=(20, 0))
            self.start_btn.pack(in_=l1, side="right")
            if show_match:
                self.match_row.pack(in_=l3, side="left")
            self.score_tiles.pack(in_=l3, side="right")
            lines = (l1, l3)
        else:
            self.opt_first.pack(in_=l1, side="left")
            self.start_btn.pack(in_=l1, side="right")
            self.opt_mode.pack(in_=l2, side="left")
            if show_match:
                self.match_row.pack(in_=l3, side="left")
            self.score_tiles.pack(in_=l4, side="left")
            lines = (l1, l2, l3, l4) if show_match else (l1, l2, l4)
        for i, line in enumerate(lines):
            line.pack(fill="x", pady=(0 if i == 0 else 10, 0))

    def scrollable(self, parent):
        """พื้นที่ที่เลื่อนขึ้นลงได้เมื่อเนื้อหาสูงกว่าหน้าต่าง (หน้าต่างใหญ่พอจะไม่มีแถบเลื่อน)"""
        sc = tk.Canvas(parent, bg=BG, highlightthickness=0)
        bar = tk.Scrollbar(parent, orient="vertical", command=sc.yview, width=12)
        sc.configure(yscrollcommand=bar.set)
        sc.pack(side="left", fill="both", expand=True)
        page = tk.Frame(sc, bg=BG, padx=20, pady=16)
        win = sc.create_window(0, 0, window=page, anchor="nw")

        def fit(event=None):
            need = page.winfo_reqheight()
            height = max(sc.winfo_height(), need)
            sc.itemconfig(win, width=sc.winfo_width(), height=height)
            sc.configure(scrollregion=(0, 0, sc.winfo_width(), height))
            if need > sc.winfo_height():
                bar.pack(side="right", fill="y")
            else:
                bar.pack_forget()
                sc.yview_moveto(0)

        sc.bind("<Configure>", fit)
        page.bind("<Configure>", fit, add="+")
        sc.bind_all("<MouseWheel>", lambda e: self.settings_visible() and bar.winfo_ismapped()
                    and sc.yview_scroll(int(-e.delta / 120), "units"), add="+")
        return page

    def build_history(self, parent):
        """ตารางประวัติการเดินของคนและ AI (ล่าสุดอยู่บน)"""
        box = self.card(parent, "ประวัติการเดินของคนและ AI", "ล่าสุดอยู่บนสุด")
        box.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        style = ttk.Style()
        style.configure("History.Treeview", font=self.f_small, rowheight=int(22 * self.ui_scale),
                        background=CARD, fieldbackground=CARD, foreground=TEXT, borderwidth=0)
        style.configure("History.Treeview.Heading", font=(self.font_family, 9, "bold"),
                        background=SOFT, foreground=MUTED, relief="flat", borderwidth=0)
        style.map("History.Treeview", background=[("selected", PRIMARY_SOFT)],
                  foreground=[("selected", TEXT)])
        cols = (("game", "เกม", 60), ("turn", "ตาที่", 60), ("time", "เวลา", 90),
                ("who", "ผู้เดิน", 140), ("cell", "ช่อง", 70), ("note", "รายละเอียด", 260))
        holder = tk.Frame(box.body, bg=CARD)
        holder.pack(fill="x")
        self.history = ttk.Treeview(holder, columns=[c[0] for c in cols], show="headings",
                                    height=4, style="History.Treeview")
        for key, text, width in cols:
            self.history.heading(key, text=text)
            self.history.column(key, width=int(width * self.ui_scale), anchor="center",
                                stretch=key == "note")
        self.history.column("note", anchor="w")
        scroll = ttk.Scrollbar(holder, orient="vertical", command=self.history.yview)
        self.history.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.history.pack(side="left", fill="x", expand=True)
        self.history.tag_configure("X", foreground=COLOR_X)
        self.history.tag_configure("O", foreground=COLOR_O)
        self.history.tag_configure("end", foreground=TEXT, background=SOFT)
        self.history.tag_configure("error", foreground=DANGER)
        self.turn_no = 0

    def add_history(self, who, cell="", note="", tag=""):
        game = self.match.game_number if self.match else 1
        if tag in ("X", "O"):
            self.turn_no += 1
        turn = self.turn_no if tag in ("X", "O") else ""
        self.history.insert("", 0, values=(game, turn, time.strftime("%H:%M:%S"), who, cell, note),
                            tags=(tag,) if tag else ())

    def toggle_fullscreen(self, on=None):
        full = not self.root.attributes("-fullscreen") if on is None else on
        self.root.attributes("-fullscreen", full)

    def build_header(self):
        head = tk.Frame(self.root, bg=HEADER, padx=20, pady=12)
        head.pack(fill="x")
        tk.Label(head, text="AI XO", font=self.f_title, fg="white", bg=HEADER).pack(side="left")
        tk.Label(head, text="ระบบเล่นเกม XO กับแขนกล Dobot MG400", font=self.f_small,
                 fg=HEADER_MUTED, bg=HEADER).pack(side="left", padx=(12, 0), pady=(4, 0))

        chips = tk.Frame(head, bg=HEADER)
        chips.pack(side="right")
        self.chip_mode = tk.Label(chips, text="", font=self.f_small, fg="white", bg="#1E293B",
                                  padx=10, pady=3)
        self.chip_mode.pack(side="left", padx=(0, 8))
        conn = tk.Frame(chips, bg="#1E293B", padx=10, pady=3)
        conn.pack(side="left")
        self.chip_dot = tk.Canvas(conn, width=10, height=10, bg="#1E293B", highlightthickness=0)
        self.chip_dot_item = self.chip_dot.create_oval(1, 1, 9, 9, fill=LAMP["ok"], outline="")
        self.chip_dot.pack(side="left")
        self.chip_conn = tk.Label(conn, text="", font=self.f_small, fg="white", bg="#1E293B")
        self.chip_conn.pack(side="left", padx=(6, 0))

    def update_header(self):
        if not self.using_mg400():
            self.chip_mode.config(text="โหมดจำลอง", bg="#1E293B")
        elif self.mg400.dry_run:
            self.chip_mode.config(text="MG400 จริง · Dry run", bg="#1E3A8A")
        else:
            self.chip_mode.config(text="MG400 จริง · เคลื่อนที่จริง", bg="#991B1B")

    def build_robot_panel(self, parent):
        box = self.card(parent, "การเชื่อมต่อหุ่นยนต์")
        box.pack(fill="x")
        body = box.body

        self.mode = tk.StringVar(value="sim")
        self.segmented(body, self.mode, (("sim", "จำลอง"), ("mg400", "MG400 จริง")),
                       self.on_mode_change).pack(anchor="w")

        row = tk.Frame(body, bg=CARD, pady=10)
        row.pack(fill="x")
        self.label(row, "IP", self.f_small, MUTED).pack(side="left")
        self.ip = self.entry(row, width=18, font=self.f_mono, justify="left")
        self.ip.insert(0, self.mg400.cfg["ip"])
        self.ip.pack(side="left", padx=8, ipady=3)
        self.ip.bind("<Key>", lambda e: self.conn_error.config(text=""))

        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        self.btn_connect = self.button(row, "เชื่อมต่อ", self.on_connect, "primary")
        self.btn_disconnect = self.button(row, "ตัดการเชื่อมต่อ", self.on_disconnect)
        self.btn_enable = self.button(row, "Enable", self.on_enable, "success")
        self.btn_disable = self.button(row, "Disable", self.on_disable, "danger")
        for b in (self.btn_connect, self.btn_disconnect, self.btn_enable, self.btn_disable):
            b.pack(side="left", padx=(0, 6))

        self.dry = tk.BooleanVar(value=True)
        tk.Checkbutton(body, text="Dry run · แสดงคำสั่ง แต่ไม่สั่งแขนเคลื่อนที่จริง",
                       variable=self.dry, font=self.f_small, bg=CARD, fg=TEXT,
                       activebackground=CARD, selectcolor="white", bd=0, cursor="hand2",
                       command=self.on_dry_change).pack(anchor="w", pady=(10, 0))

        tk.Frame(body, bg=BORDER, height=1).pack(fill="x", pady=10)
        row = tk.Frame(body, bg=CARD)
        row.pack(fill="x")
        self.lamp = tk.Canvas(row, width=12, height=12, bg=CARD, highlightthickness=0)
        self.lamp_dot = self.lamp.create_oval(1, 1, 11, 11, fill=LAMP["off"], outline="")
        self.lamp.pack(side="left")
        self.conn_text = self.label(row, "", self.f_body)
        self.conn_text.pack(side="left", padx=8)
        self.pose_text = self.label(body, "", self.f_mono, MUTED)
        self.pose_text.pack(anchor="w", pady=(4, 0))
        self.conn_error = self.label(body, "", self.f_small, DANGER, wraplength=340, justify="left")
        self.conn_error.pack(anchor="w")
        self.refresh_robot_panel()

    # ================= หน้าตั้งค่า / สอบเทียบ =================
    def build_settings(self, page):
        fs = self.f_small
        page.columnconfigure(0, weight=1)
        page.rowconfigure(1, weight=1)

        # ---- แถวบน: ตำแหน่งแขน + ความคืบหน้า + ปุ่มขยับแขน ----
        top = self.card(page)
        top.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        info = tk.Frame(top.body, bg=CARD)
        info.pack(side="left", fill="x", expand=True)
        self.pose_now = self.label(info, "", self.f_head, PRIMARY)
        self.pose_now.pack(anchor="w")
        self.label(info, "วิธีสอนจุด: ขยับแขนให้หัวจับอยู่ตรงตำแหน่งจับ/ปล่อยหมากพอดี แล้วกด บันทึก  ·  "
                         "ลง = ไปค้างที่จุดจริง ไว้ปรับละเอียดแล้วบันทึกใหม่  ·  Move = ทดสอบ ลง-ยกขึ้น เหมือนเล่นจริง  ·  "
                         "ทุกปุ่มยกข้ามที่ความสูง Approach เสมอ  ·  ทุกคำสั่งทำตาม Dry run  ·  หน่วย มม.",
                   fs, MUTED, wraplength=int(620 * self.ui_scale), justify="left").pack(
            anchor="w", pady=(2, 8))
        chips = tk.Frame(info, bg=CARD)
        chips.pack(anchor="w")
        self.label(chips, "ความคืบหน้า", fs, MUTED).pack(side="left", padx=(0, 8))
        self.progress_chips = {}
        for key in ("cell_xyz", "pick_xyz", "approach_z", "park_xyz"):
            lb = tk.Label(chips, text="", font=fs, padx=10, pady=2)
            lb.pack(side="left", padx=(0, 6))
            self.progress_chips[key] = lb
        self.build_jog(top.body)

        # ---- ซ้าย: ภาพกล้องพร้อมเลขช่อง (ซ่อนเองเมื่อหน้าต่างแคบ) ----
        self.preview_card = self.card(page, "ภาพจากกล้อง", "ตรวจเลขช่องก่อนวัดพิกัด")
        self.preview_card.grid(row=1, column=0, sticky="nsew", padx=(0, 14))
        self.button(self.preview_card.title.master, "หมุนเลขช่อง 90°", self.rotate_numbering,
                    small=True).pack(side="right", padx=(8, 0), before=self.preview_card.note)
        self.preview = tk.Canvas(self.preview_card.body, width=320, height=180, bg="#0B1220",
                                 highlightthickness=0)
        self.preview.pack(fill="both", expand=True)
        self.pv_img = self.preview.create_image(0, 0, anchor="center")
        self.pv_w, self.pv_h, self.pv_cell, self.pv_photo = 320, 180, None, None
        self.preview.bind("<Configure>", self.on_preview_resize)
        self.label(self.preview_card.body, "กรอบสีฟ้า = ตารางที่หาได้จากจุดมุม (วงแดง) · คลิกช่องกรอกตัวเลข"
                                           "ของช่องไหน ภาพจะชี้ช่องนั้น", fs, MUTED).pack(anchor="w", pady=(8, 0))

        # ---- ขวา: ฟอร์มสอบเทียบ 3 คอลัมน์ ----
        forms = self.settings_forms = tk.Frame(page, bg=BG)
        forms.grid(row=1, column=1, sticky="n")
        page.bind("<Configure>", lambda e: self.layout_settings(e.width), add="+")

        self.points = {}   # key -> [[entry X, entry Y(, entry Z)], ...]
        self.fields = {}   # key -> entry

        col_a = tk.Frame(forms, bg=BG)
        col_a.grid(row=0, column=0, sticky="n", padx=(0, 14))
        box = self.card(col_a, "① จุดวาง · ช่องบนกระดาน", "ตำแหน่งปล่อยหมาก")
        box.pack(fill="x")
        self.point_rows(box.body, "cell_xyz", "ช่อง", 9)
        self.button(box.body, "คำนวณช่องที่เหลือจากช่อง 1, 3, 7", self.fill_grid,
                    small=True).grid(row=11, column=0, columnspan=8, pady=(10, 0), sticky="ew")
        self.tour_bar(box.body, "cell_xyz", "Move ทุกช่อง 1 → 9", 9).grid(
            row=12, column=0, columnspan=8, pady=(6, 0), sticky="ew")

        col_b = tk.Frame(forms, bg=BG)
        col_b.grid(row=0, column=1, sticky="n", padx=(0, 14))
        box = self.card(col_b, "② จุดหยิบหมาก O (5 จุด)", "หยิบเรียง 1 → 5")
        box.pack(fill="x")
        self.point_rows(box.body, "pick_xyz", "หยิบ", 5)
        self.tour_bar(box.body, "pick_xyz", "Move ทุกจุดหยิบ 1 → 5", 5).grid(
            row=7, column=0, columnspan=8, pady=(8, 0), sticky="ew")
        self.label(box.body, "สอนกี่จุดก็ได้ (อย่างน้อย 1) · เกมใหม่เริ่มหยิบจากจุดที่ 1", fs, MUTED).grid(
            row=8, column=0, columnspan=8, sticky="w", pady=(6, 0))

        box = self.card(col_b, "③ ความสูง Approach (Z)", "ค่าเดียว ใช้ทุกจุด")
        box.pack(fill="x", pady=(14, 0))
        self.label(box.body, "Z Approach (มม.)", fs).grid(row=0, column=0, sticky="w")
        self.fields["approach_z"] = self.entry(box.body, width=8)
        self.fields["approach_z"].grid(row=0, column=1, padx=6, pady=2, ipady=2)
        self.button(box.body, "บันทึก", lambda: self.read_pose("approach_z", 0), "primary",
                    small=True).grid(row=0, column=2, padx=(0, 3))
        self.button(box.body, "ยกขึ้นไปที่ Z นี้", self.goto_approach_z,
                    small=True).grid(row=0, column=3)
        self.label(box.body, "ทุกจุดหยิบ/วาง แขนจะลอยที่ Z นี้ → ลงตรง → ยกกลับขึ้น Z นี้\n"
                             "และเคลื่อนที่ข้ามจุดที่ความสูงนี้เสมอ\n"
                             "วิธีตั้ง: ยกหัวจับสูงพ้นหมาก 3–5 ซม. แล้วกด บันทึก\n"
                             "(ต้องสูงกว่าจุดหยิบ/วางทุกจุดอย่างน้อย 20 มม.)", fs, MUTED,
                   justify="left").grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        box = self.card(col_b, "④ จุดจอดนอกมุมกล้อง")
        box.pack(fill="x", pady=(14, 0))
        self.point_rows(box.body, "park_xyz", "จุดจอด", 1)

        col_c = tk.Frame(forms, bg=BG)
        col_c.grid(row=0, column=2, sticky="n")
        box = self.card(col_c, "การเคลื่อนที่และหัวจับ")
        box.pack(fill="x")
        rows = (("speed_percent", "ความเร็ว (%)"), ("suck_pulse_s", "เปิดดูดนาน (วิ)"),
                ("gripper_wait_s", "เปิดพ่นนาน (วิ)"),
                ("gripper_do", "ดูด · ขา D"), ("release_do", "พ่นออก · ขา D"))
        for i, (key, name) in enumerate(rows):
            self.label(box.body, name, fs).grid(row=i, column=0, sticky="w")
            self.fields[key] = self.entry(box.body)
            self.fields[key].grid(row=i, column=1, padx=(10, 0), pady=2, ipady=2)
        # ตารางคำสั่ง DO ตามเลขขาที่ใส่ (อัปเดตทันทีที่แก้ตัวเลข)
        self.do_info = tk.Label(box.body, text="", font=("Consolas", 10), bg=SOFT, fg=TEXT,
                                justify="left", anchor="w", padx=10, pady=6)
        self.do_info.grid(row=len(rows), column=0, columnspan=2, sticky="ew", pady=(6, 0))
        for key in ("gripper_do", "release_do"):
            self.fields[key].bind("<KeyRelease>", lambda e: self.update_do_info(), add="+")
        rows = rows + (None,)
        row = tk.Frame(box.body, bg=CARD)
        row.grid(row=len(rows), column=0, columnspan=2, sticky="w", pady=(10, 0))
        self.label(row, "ทดสอบหัวดูด", fs, MUTED).pack(side="left")
        self.button(row, "ดูด", lambda: self.test_gripper(True), small=True).pack(side="left", padx=6)
        self.button(row, "พ่นออก", lambda: self.test_gripper(False), small=True).pack(side="left")
        self.label(box.body, "ใช้เลขขาที่แสดงด้านบน (ไม่ต้องกดบันทึกก่อน)", fs, MUTED).grid(
            row=len(rows) + 1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        box = self.card(col_c, "ขั้นสูง")
        box.pack(fill="x", pady=(14, 0))
        rows = (("dashboard_port", "พอร์ต Dashboard"), ("move_port", "พอร์ต Move"),
                ("move_timeout_s", "รอแขนวิ่งเสร็จนานสุด (วิ)"))
        for i, (key, name) in enumerate(rows):
            self.label(box.body, name, fs).grid(row=i, column=0, sticky="w")
            self.fields[key] = self.entry(box.body)
            self.fields[key].grid(row=i, column=1, padx=(10, 0), pady=2, ipady=2)

        # ทดสอบหยิบ-วางเต็มรอบ: ยืนยันว่าพิกัดทั้งชุดใช้ได้จริงก่อนเริ่มเล่น
        box = self.card(col_c, "ทดสอบหยิบ-วางเต็มรอบ")
        box.pack(fill="x", pady=(14, 0))
        row = tk.Frame(box.body, bg=CARD)
        row.pack(anchor="w")
        self.test_slot = tk.StringVar(value="1")
        self.test_cell = tk.StringVar(value="5")
        for text, var, top_n in (("จากจุดหยิบ", self.test_slot, 5), ("ไปวางช่อง", self.test_cell, 9)):
            self.label(row, text, fs).pack(side="left", padx=(0, 4))
            tk.Spinbox(row, from_=1, to=top_n, width=3, textvariable=var, state="readonly",
                       font=fs, relief="flat", readonlybackground="white",
                       buttonbackground="#E8EDF3", highlightthickness=1,
                       highlightbackground=BORDER).pack(side="left", padx=(0, 10))
        self.button(box.body, "เริ่มทดสอบ", self.test_full_cycle, "success", small=True).pack(
            anchor="w", pady=(8, 0))
        self.label(box.body, "ใช้ค่าที่บันทึกแล้ว · หยิบ → วาง → กลับจุดจอด", fs, MUTED).pack(
            anchor="w", pady=(6, 0))

        bar = tk.Frame(col_c, bg=BG, pady=14)
        bar.pack(fill="x")
        self.button(bar, "บันทึกการตั้งค่า", self.save_settings, "primary").pack(side="left")
        self.button(bar, "โหลดจากไฟล์", self.reload_settings).pack(side="left", padx=8)
        self.cal_status = self.label(col_c, "", fs, wraplength=int(300 * self.ui_scale),
                                     justify="left")
        self.cal_status.pack(anchor="w")

        self.fill_settings(self.mg400.cfg)
        for e in [e for rows in self.points.values() for r in rows for e in r] + list(self.fields.values()):
            e.bind("<KeyRelease>", lambda ev: self.update_progress())
        self.update_progress()

    def build_jog(self, parent):
        """ปุ่มขยับแขนทีละนิด ใช้เลื่อนหัวจับให้ตรงจุดก่อนกด อ่าน"""
        jog = tk.Frame(parent, bg=CARD)
        jog.pack(side="right", padx=(20, 0))
        head = tk.Frame(jog, bg=CARD)
        head.pack(anchor="e")
        self.label(head, "ขยับแขนทีละ", self.f_small, MUTED).pack(side="left", padx=(0, 8))
        self.jog_step = tk.StringVar(value="5")
        self.segmented(head, self.jog_step, (("1", "1"), ("5", "5"), ("10", "10"),
                                             ("20", "20 มม."))).pack(side="left")
        pad = tk.Frame(jog, bg=CARD)
        pad.pack(anchor="e", pady=(8, 0))
        for i, axis in enumerate("XYZ"):
            for j, sign in enumerate((-1, 1)):
                self.button(pad, f"{axis}{'−' if sign < 0 else '+'}",
                            lambda a=axis, s=sign: self.jog(a, s), small=True, width=4).grid(
                    row=0, column=i * 2 + j, padx=(0 if j else (0 if i == 0 else 12), 3))

    def layout_settings(self, width):
        """หน้าต่างแคบ: ซ่อนภาพกล้อง ให้ฟอร์มมีที่พอ"""
        need = self.settings_forms.winfo_reqwidth() + int(360 * self.ui_scale)
        if width < need:
            self.preview_card.grid_remove()
            self.settings_forms.grid(row=1, column=0, columnspan=2)   # ฟอร์มอยู่กลาง
        else:
            self.preview_card.grid()
            self.settings_forms.grid(row=1, column=1, columnspan=1)

    # ---------- ภาพกล้องในหน้าตั้งค่า ----------
    def settings_visible(self):
        return self.tabs.index("current") == 1

    def on_preview_resize(self, event):
        self.pv_w, self.pv_h = event.width, event.height
        self.preview.coords(self.pv_img, self.pv_w // 2, self.pv_h // 2)
        self.draw_preview()

    def select_cell(self, i):
        self.pv_cell = i
        self.draw_preview()

    def draw_preview(self, frame=None):
        c, w, h = self.preview, self.pv_w, self.pv_h
        if frame is not None:
            fh, fw = frame.shape[:2]
            scale = min(w / fw, h / fh)
            small = cv2.resize(frame, (max(1, int(fw * scale)), max(1, int(fh * scale))),
                               interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
            ok, buf = cv2.imencode(".ppm", small)
            if ok:
                self.pv_photo = tk.PhotoImage(data=buf.tobytes(), format="PPM")
                c.itemconfig(self.pv_img, image=self.pv_photo)
        elif not self.camera_view:
            self.pv_photo = None
            c.itemconfig(self.pv_img, image="")
        c.delete("grid")
        if self.camera_view and self.markers_fresh():
            self.paint_preview_grid(c, w, h)
        elif self.camera_view and self.pv_photo is not None:
            c.create_text(w // 2, h - 20, fill="white", font=self.f_small, tags="grid",
                          text="ไม่พบมุมกระดาน · วางแผ่นสนามให้กล้องเห็นสี่เหลี่ยมดำอย่างน้อย 3 มุม")
        if not self.camera_view:
            c.create_text(w // 2, h - 20, text="กล้องปิดอยู่ · กด เปิดกล้อง ที่หน้าหลัก",
                          fill="white", font=self.f_small, tags="grid")
        elif frame is None and self.cam.error:
            c.create_text(w // 2, h - 20, text=self.cam.error, fill="white", font=self.f_small,
                          tags="grid")

    def rotate_numbering(self):
        """หมุนเลขช่องเพิ่ม 90° ให้ตรงกับมุมที่ผู้เล่นมองกระดาน
        พิกัดหุ่นยนต์ของแต่ละช่องที่วัดไว้จะย้ายตามเลขใหม่ (ยังชี้ช่องจริงเดิมบนกระดาษ)"""
        if self.busy:
            self.cal_say("รอให้หุ่นยนต์ทำงานเสร็จก่อน", True)
            return
        cfg = self.mg400.cfg
        cfg["board_rotation"] = (cfg.get("board_rotation", 0) + 90) % 360
        rows = self.points["cell_xyz"]
        values = rotate_cells([[e.get() for e in r] for r in rows])
        for entries, vals in zip(rows, values):
            for e, v in zip(entries, vals):
                self.set_entry(e, v)
        if cfg.get("cell_xyz"):
            cfg["cell_xyz"] = rotate_cells(cfg["cell_xyz"])
        try:
            save_config(cfg)
        except OSError as e:
            self.cal_say(f"เขียนไฟล์ไม่ได้: {e}", True)
            return
        self.add_log(f"หมุนเลขช่องเป็น {cfg['board_rotation']}°")
        self.cal_say(f"หมุนเลขช่องแล้ว ({cfg['board_rotation']}°) และบันทึกแล้ว · พิกัดของแต่ละช่องย้ายตาม"
                     "เลขใหม่ให้แล้ว แขนยังวางตรงช่องจริงเหมือนเดิม")
        self.draw_preview(self.cam.latest() if self.camera_view else None)
        self.draw_board(self.highlight)

    def paint_preview_grid(self, c, w, h):
        """กรอบตารางที่หาได้จริง (สีฟ้า) จุดมุม (วงแดง) และเลขช่อง ทับบนภาพกล้องตามมุมมองจริง"""
        fw, fh = self.frame_size
        scale = min(w / fw, h / fh)
        ox, oy = (w - fw * scale) / 2, (h - fh * scale) / 2
        to_canvas = lambda pts: [(ox + x * scale, oy + y * scale) for x, y in pts]
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [3, 0], [3, 3], [0, 3]]),
                                        np.float32(self.current_quad()))
        grid = lambda pts: to_canvas(cv2.perspectiveTransform(
            np.float32(pts).reshape(-1, 1, 2), H).reshape(-1, 2))
        flat = lambda pts: [v for p in pts for v in p]
        line = "#22D3EE"
        if self.pv_cell is not None:
            r, col = divmod(self.pv_cell, 3)
            cell = grid([(col, r), (col + 1, r), (col + 1, r + 1), (col, r + 1)])
            c.create_polygon(*flat(cell), outline="#60A5FA", fill="", width=4, tags="grid")
        c.create_polygon(*flat(grid([(0, 0), (3, 0), (3, 3), (0, 3)])), outline=line, fill="",
                         width=2, tags="grid")
        for k in (1, 2):
            c.create_line(*flat(grid([(k, 0), (k, 3)])), fill=line, width=1, tags="grid")
            c.create_line(*flat(grid([(0, k), (3, k)])), fill=line, width=1, tags="grid")
        size = np.linalg.norm(np.subtract(*grid([(0, 0), (1, 0)])))
        num_font = (self.font_family, int(max(9, min(28, size / 3))), "bold")
        for i, (x, y) in enumerate(grid([(k % 3 + 0.5, k // 3 + 0.5) for k in range(9)])):
            c.create_text(x, y, text=str(i + 1), font=num_font, tags="grid",
                          fill="#60A5FA" if i == self.pv_cell else line)
        corners = self.locked_markers if self.locked_markers is not None else self.markers
        for x, y in to_canvas(corners if corners is not None else []):
            c.create_oval(x - 7, y - 7, x + 7, y + 7, outline="#F43F5E", width=2, tags="grid")

    # ---------- ความคืบหน้าการสอบเทียบ ----------
    def update_progress(self):
        def filled(entries):
            try:
                return all(e.get().strip() and float(e.get()) is not None for e in entries)
            except ValueError:
                return False
        counts = {
            "cell_xyz": ("จุดวาง", sum(filled(r) for r in self.points["cell_xyz"]), 9, 9),
            "pick_xyz": ("จุดหยิบ", sum(filled(r) for r in self.points["pick_xyz"]), 5, 1),
            "approach_z": ("Z Approach", int(filled([self.fields["approach_z"]])), 1, 1),
            "park_xyz": ("จุดจอด", int(filled(self.points["park_xyz"][0])), 1, 1),
        }
        for key, (name, n, total, enough) in counts.items():
            if n >= enough:             # พอใช้เล่นได้แล้ว (จุดหยิบมี 1 จุดก็พอ)
                text, bg, fg = f"✓ {name} {n}/{total}", "#DCFCE7", "#15803D"
            elif n:
                text, bg, fg = f"{name} {n}/{total}", "#FEF3C7", "#B45309"
            else:
                text, bg, fg = f"{name} 0/{total}", SOFT, MUTED
            self.progress_chips[key].config(text=text, bg=bg, fg=fg)

    # ---------- ขยับแขน / ทดสอบ ----------
    def jog(self, axis, sign):
        if not self.robot_free():
            return
        if self.mg400.dry_run:
            self.cal_say("Dry run เปิดอยู่ แขนจึงไม่ขยับ · ปิด Dry run ที่หน้าหลักก่อนใช้ปุ่มขยับแขน", True)
            return
        step = float(self.jog_step.get()) * sign
        delta = [0.0, 0.0, 0.0]
        delta["XYZ".index(axis)] = step
        self.busy = True
        self.cal_say(f"กำลังขยับ {axis} {step:+g} มม. ...")
        self.background(lambda: self.mg400.jog(*delta), "jog_done", "calib_fail")

    def test_full_cycle(self):
        if not self.robot_free():
            return
        cell, slot = int(self.test_cell.get()) - 1, int(self.test_slot.get()) - 1
        dry = self.mg400.dry_run
        if not dry and not messagebox.askyesno(
                "ยืนยันทดสอบหยิบ-วาง",
                f"แขนจะหยิบหมากจากจุดหยิบ {slot + 1} ไปวางช่อง {cell + 1} แล้วกลับจุดจอด\n\n"
                "ตรวจว่าไม่มีสิ่งกีดขวาง และเอื้อมถึงปุ่ม E-Stop ได้"):
            return
        self.select_cell(cell)
        self.busy = True
        done = (f"[dry run] ทดสอบเสร็จ ดูคำสั่งที่บันทึกการทำงานในหน้าหลัก" if dry
                else f"ทดสอบเสร็จ: วางช่อง {cell + 1} แล้วกลับจุดจอด · ตรวจว่าหมากอยู่กลางช่อง")
        say = lambda m: self.events.put(("cal_progress", m, {}))
        self.background(lambda: (self.mg400.test_pick_place(cell, slot, say), done)[1],
                        "calib_done", "calib_fail")

    def point_rows(self, parent, key, label, count, header=True, row0=1):
        """แถวสอนจุด: X Y Z + [บันทึก] [เหนือ] [ลง] (จุดเดี่ยว เช่น จุดจอด/★ มีแค่ [บันทึก] [ไป])"""
        if header:
            for j, axis in enumerate("XYZ"):
                self.label(parent, axis, self.f_small, MUTED).grid(row=row0 - 1, column=1 + j)
        self.points[key] = []
        for i in range(count):
            r = row0 + i
            name = label if count == 1 else f"{label} {i + 1}"
            self.label(parent, name, self.f_small).grid(row=r, column=0, sticky="w", padx=(0, 6))
            entries = []
            for j in range(3):
                e = self.entry(parent, width=7)
                e.grid(row=r, column=1 + j, padx=2, pady=2, ipady=2)
                if key == "cell_xyz":   # คลิกช่องกรอกของช่องไหน ภาพกล้องจะชี้ช่องนั้น
                    e.bind("<FocusIn>", lambda ev, i=i: self.select_cell(i))
                entries.append(e)
            self.points[key].append(entries)
            self.button(parent, "บันทึก", lambda i=i: self.read_pose(key, i), "primary",
                        small=True).grid(row=r, column=4, padx=(6, 3))
            if key in SINGLE_POINTS:
                self.button(parent, "ไป", lambda i=i: self.goto_point(key, i, down=True),
                            small=True).grid(row=r, column=5)
            else:
                self.button(parent, "ลง", lambda i=i: self.goto_point(key, i, down=True),
                            small=True).grid(row=r, column=5, padx=(0, 3))
                self.button(parent, "Move", lambda i=i: self.move_points(key, [i]), "success",
                            small=True).grid(row=r, column=6)

    @staticmethod
    def fmt(v):
        return "" if v is None else f"{float(v):.2f}".rstrip("0").rstrip(".")

    @staticmethod
    def set_entry(entry, value):
        entry.delete(0, "end")
        entry.insert(0, value)

    def fill_settings(self, cfg):
        for key, rows in self.points.items():
            data = cfg.get(key)
            if key in SINGLE_POINTS:
                data = [data]
            for i, entries in enumerate(rows):
                point = data[i] if data and i < len(data) else None
                for j, e in enumerate(entries):
                    self.set_entry(e, self.fmt(point[j]) if point else "")
        for key, e in self.fields.items():
            self.set_entry(e, self.fmt(cfg.get(key)))
        self.update_do_info()

    def update_do_info(self):
        """แสดงคำสั่ง DO ที่โปรแกรมจะส่ง ตามเลขขาดูด/พ่นที่กรอกไว้"""
        suck = self.fields["gripper_do"].get().strip() or "?"
        blow = self.fields["release_do"].get().strip() or "?"
        self.do_info.config(text=f"DO({suck}, ON)  เปิดดูด   DO({suck}, OFF) ปิดดูด\n"
                                 f"DO({blow}, ON)  เปิดพ่น   DO({blow}, OFF) ปิดพ่น")

    def read_number(self, entry, name):
        text = entry.get().strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError:
            raise ValueError(f"{name}: '{text}' ไม่ใช่ตัวเลข")

    def read_point(self, key, i):
        """คืน [x, y(, z)] หรือ None ถ้าว่างทั้งแถว แจ้งผิดพลาดถ้ากรอกไม่ครบ"""
        name = POINT_NAMES[key] + ("" if key in SINGLE_POINTS else f" {i + 1}")
        values = [self.read_number(e, name) for e in self.points[key][i]]
        if all(v is None for v in values):
            return None
        if any(v is None for v in values):
            raise ValueError(f"{name} กรอกไม่ครบทุกแกน")
        return values

    def collect_settings(self):
        cfg = {}
        for key, count in POINT_LISTS.items():
            pts = [self.read_point(key, i) for i in range(count)]
            cfg[key] = None if all(p is None for p in pts) else pts
        for key in SINGLE_POINTS:
            cfg[key] = self.read_point(key, 0)
        ints = ("speed_percent", "gripper_do", "release_do", "dashboard_port", "move_port")
        for key, e in self.fields.items():
            v = self.read_number(e, key)
            if v is None:
                raise ValueError(f"{key} ต้องมีค่า")
            cfg[key] = int(v) if key in ints and v is not None else v
        if not 0 <= cfg["suck_pulse_s"] <= 10:
            raise ValueError("เปิดดูดนานต้องอยู่ระหว่าง 0–10 วินาที")
        if not 0 <= cfg["gripper_wait_s"] <= 10:
            raise ValueError("เปิดพ่นนานต้องอยู่ระหว่าง 0–10 วินาที")
        if not 1 <= cfg["speed_percent"] <= 100:
            raise ValueError("ความเร็วต้องอยู่ระหว่าง 1–100 %")
        return cfg

    def cal_say(self, text, error=False):
        self.cal_status.config(text=text, fg=DANGER if error else TEXT)

    def save_settings(self):
        if self.busy:
            self.cal_say("รอให้หุ่นยนต์ทำงานเสร็จก่อนจึงบันทึกได้", True)
            return
        try:
            new = self.collect_settings()
        except ValueError as e:
            self.cal_say(f"ยังไม่ได้บันทึก: {e}", True)
            return
        try:
            ipaddress.ip_address(self.ip.get().strip())
            new["ip"] = self.ip.get().strip()
        except ValueError:
            pass
        self.mg400.cfg.update(new)
        if self.locked_markers is not None:   # จำตำแหน่งกระดานตอนสอบเทียบ ไว้เตือนเมื่อกระดานถูกขยับ
            self.mg400.cfg["calib_board"] = {
                "frame": list(self.frame_size),
                "markers": [[round(float(x), 1), round(float(y), 1)] for x, y in self.locked_markers]}
            self.set_board_moved(False)
        had_file = os.path.exists(CONFIG_FILE)
        try:
            save_config(self.mg400.cfg)
        except OSError as e:
            self.cal_say(f"เขียนไฟล์ไม่ได้: {e}", True)
            return
        missing = self.mg400.missing_config()
        far = out_of_reach(self.mg400.cfg)
        text = "บันทึกลง robot_config.json แล้ว"
        if had_file:
            text += " (ค่าก่อนหน้าเก็บไว้ที่ robot_config.backup.json)"
        text += f"\nยังขาด: {', '.join(missing)} (เล่นจริงได้เมื่อครบ)" if missing else "\nสอบเทียบครบแล้ว"
        if far:
            text += f"\n⚠ อาจเกินระยะเอื้อม {MAX_REACH_MM} มม.: {', '.join(far)} · ตรวจพิกัดอีกครั้ง"
        cfg = self.mg400.cfg
        too_low = low_points(cfg, MIN_APPROACH_MM)   # จุดที่ Z Approach สูงกว่าไม่พอ อาจลากหมาก/ชน
        if too_low:
            text += (f"\n⚠ Z Approach ต้องสูงกว่า {', '.join(too_low)} อย่างน้อย {MIN_APPROACH_MM} มม. "
                     "· เพิ่ม Z Approach หรือตรวจ Z ของจุดนั้น")
        cells = [(i, p[2]) for i, p in enumerate(cfg.get("cell_xyz") or []) if p]
        odd = []
        if len(cells) >= 3:     # กระดานเรียบ: Z ทุกช่องควรใกล้กัน · ช่องที่ต่างจากค่ากลางมาก = สอนพลาด
            mid = sorted(z for _, z in cells)[len(cells) // 2]
            odd = [str(i + 1) for i, z in cells if abs(z - mid) > CELL_Z_SPREAD_MM]
        if odd:
            text += (f"\n⚠ Z ของช่อง {', '.join(odd)} ต่างจากช่องอื่นเกิน {CELL_Z_SPREAD_MM} มม. "
                     "· กระดานเรียบ Z ควรใกล้กัน ตรวจหรือสอนช่องนั้นใหม่")
            too_low = too_low or odd
        if self.locked_markers is not None:
            text += "\nจำตำแหน่งกระดานตอนนี้ไว้แล้ว · ถ้ากระดานถูกขยับ จะมีคำเตือน"
        self.cal_say(text, error=bool(far) or bool(too_low))
        self.add_log("บันทึกการตั้งค่าหุ่นยนต์แล้ว", "ok")

    def reload_settings(self):
        try:
            cfg = load_config()
        except (OSError, ValueError) as e:
            self.cal_say(f"อ่านไฟล์ไม่ได้: {e}", True)
            return
        self.mg400.cfg.update(cfg)
        self.fill_settings(self.mg400.cfg)
        self.update_progress()
        self.cal_say("โหลดค่าจาก robot_config.json แล้ว")

    def fill_grid(self):
        try:
            corners = [self.read_point("cell_xyz", i) for i in (0, 2, 6)]
        except ValueError as e:
            self.cal_say(str(e), True)
            return
        if any(p is None for p in corners):
            self.cal_say("ต้องกรอกช่อง 1, 3 และ 7 ก่อน", True)
            return
        for i, point in enumerate(grid_from_corners(*corners)):
            for e, v in zip(self.points["cell_xyz"][i], point):
                self.set_entry(e, self.fmt(v))
        self.update_progress()
        self.cal_say("คำนวณครบ 9 ช่องแล้ว (รวมความสูง Z) · ใช้ปุ่ม Move/ลง ตรวจทีละช่อง "
                     "แล้วกด บันทึกการตั้งค่า")

    def robot_free(self):
        if not self.mg400.is_connected:
            self.cal_say("ยังไม่ได้เชื่อมต่อหุ่นยนต์ (เชื่อมต่อที่หน้าหลัก)", True)
            return False
        if self.busy:
            self.cal_say("หุ่นยนต์กำลังทำงานอยู่ รอสักครู่", True)
            return False
        return True

    def read_pose(self, key, i):
        if key == "cell_xyz":
            self.select_cell(i)
        if self.robot_free():
            self.background(lambda: ((key, i), self.mg400.get_pose()), "pose_read", "calib_fail")

    def show_read_pose(self, target, pose):
        key, i = target
        x, y, z = pose[:3]
        if key in self.fields:
            self.set_entry(self.fields[key], self.fmt(round(z, 2)))
            self.update_progress()
            self.cal_say(f"ตั้ง Z Approach = {z:.2f} แล้ว (อย่าลืมกด บันทึกการตั้งค่า)")
            return
        entries = self.points[key][i]
        for e, v in zip(entries, (x, y, z)):
            self.set_entry(e, self.fmt(round(v, 2)))
        self.update_progress()
        name = POINT_NAMES[key] + ("" if key in SINGLE_POINTS else f" {i + 1}")
        self.cal_say(f"สอน{name}แล้ว: X {x:.2f}  Y {y:.2f}  Z {z:.2f}  (อย่าลืมกด บันทึกการตั้งค่า)")

    def tour_bar(self, parent, key, text, count):
        """แถวปุ่ม: [Move ทุกจุด] [■ หยุด]"""
        bar = tk.Frame(parent, bg=CARD)
        self.button(bar, text, lambda: self.move_points(key, list(range(count))), "success",
                    small=True).pack(side="left", fill="x", expand=True, padx=(0, 6))
        self.button(bar, "■ หยุด", self.stop_tour, "danger", small=True).pack(side="left")
        return bar

    def approach_z(self):
        """ค่า Z Approach จากช่องกรอก (ต้องมีค่า)"""
        az = self.read_number(self.fields["approach_z"], "Z Approach")
        if az is None:
            raise ValueError("ตั้งค่า Z Approach (การ์ด ③) ก่อน")
        return az

    def goto_approach_z(self):
        """ยกแขนขึ้นตรง ๆ ไปที่ความสูง Approach (X, Y เดิม) ใช้ตรวจว่าสูงพ้นของหรือยัง"""
        if not self.robot_free():
            return
        try:
            az = self.approach_z()
        except ValueError as e:
            self.cal_say(str(e), True)
            return
        dry = self.mg400.dry_run
        if not dry and not messagebox.askyesno(
                "ยืนยันการเคลื่อนที่", f"ยกแขนขึ้น/ลงตรง ๆ ไปที่ Z {az:.1f} (X, Y เดิม) ?\n\n"
                                      "ตรวจว่าไม่มีสิ่งกีดขวาง และเอื้อมถึงปุ่ม E-Stop ได้"):
            return
        self.busy = True

        def run():
            x, y, _, _ = self.mg400.get_pose()
            self.mg400.calib_move(x, y, az)
            return (f"[dry run] ไม่ได้ขยับจริง: {self.mg400.movj_command(x, y, az)}" if dry
                    else f"อยู่ที่ Z Approach {az:.1f} แล้ว · ตรวจว่าหัวจับสูงพ้นหมากและของบนโต๊ะ")
        self.background(run, "calib_done", "calib_fail")

    def move_points(self, key, indices):
        """Move = ทดสอบจุดแบบเดียวกับตอนเล่นจริง: ลอยที่ Z Approach → ลง → ค้างให้ดู → ยกกลับ"""
        if not self.robot_free():
            return
        try:
            az = self.approach_z()
            points = []
            for i in indices:
                p = self.read_point(key, i)
                if p is not None:
                    points.append((POINT_NAMES[key] + f" {i + 1}", p))
        except ValueError as e:
            self.cal_say(str(e), True)
            return
        if not points:
            self.cal_say("ยังไม่มีจุดที่สอนไว้ให้ทดสอบ (กด บันทึก ก่อน)", True)
            return
        dry = self.mg400.dry_run
        names = ", ".join(n for n, _ in points) if len(points) <= 3 else f"{len(points)} จุด"
        if not dry and not messagebox.askyesno(
                "ยืนยัน Move ทดสอบจุด",
                f"แขนจะไปทดสอบ {names}\nแต่ละจุด: ลอยที่ Z {az:g} → ลงที่จุด → ยกกลับขึ้น Z {az:g}\n\n"
                "ตรวจว่าไม่มีสิ่งกีดขวาง และเอื้อมถึงปุ่ม E-Stop ได้ (ปุ่ม หยุด จะหยุดระหว่างท่า)"):
            return
        if key == "cell_xyz" and len(indices) == 1:
            self.select_cell(indices[0])
        self.tour_stop = threading.Event()
        self.busy = True
        say = lambda m: self.events.put(("cal_progress", m, {}))
        done = (f"[dry run] ทดสอบ {names} เสร็จ (ไม่ได้ขยับจริง)" if dry
                else f"Move ทดสอบ {names} เสร็จ · ถ้าจุดไหนไม่ตรง ขยับแขนแล้วกด บันทึก ใหม่")

        def run():
            finished = self.mg400.visit_points(points, az, say, self.tour_stop)
            return done if finished else "หยุดการทดสอบแล้ว"
        self.background(run, "calib_done", "calib_fail")

    def stop_tour(self):
        if getattr(self, "tour_stop", None) is not None:
            self.tour_stop.set()
            self.cal_say("กำลังหยุด ... (แขนจะหยุดหลังจบท่าที่ทำอยู่ · หยุดทันทีใช้ปุ่ม E-Stop)", True)

    def goto_point(self, key, i, down=False):
        """ลง/ไป = ไปที่จุดจริง (down=False = ลอยเหนือจุดที่ Z Approach)
        ทุกแบบยกขึ้น Z Approach ก่อนข้ามไปจุดอื่น (ไม่ลากหัวจับไปตามโต๊ะ)"""
        if key == "cell_xyz":
            self.select_cell(i)
        if not self.robot_free():
            return
        try:
            point = self.read_point(key, i)
            if point is None:
                raise ValueError("จุดนี้ยังไม่ได้สอน (กด บันทึก ก่อน)")
            az = self.read_number(self.fields["approach_z"], "Z Approach")
            if az is None and not down:
                raise ValueError("ตั้งค่า Z Approach (การ์ด ③) ก่อน")
        except ValueError as e:
            self.cal_say(str(e), True)
            return
        x, y, z = point
        tz = z if down else az
        dry = self.mg400.dry_run
        route = f"(ยกขึ้น Z {az:g} ก่อนข้ามไป)\n" if az is not None else ""
        if not dry and not messagebox.askyesno(
                "ยืนยันการเคลื่อนที่", f"ขยับแขนไปที่ X {x:.1f}  Y {y:.1f}  Z {tz:.1f} ?\n{route}\n"
                                      "ตรวจว่าไม่มีสิ่งกีดขวาง และเอื้อมถึงปุ่ม E-Stop ได้"):
            return
        self.busy = True
        self.cal_say("กำลังเคลื่อนที่ ..." if not dry else "")
        done = (f"[dry run] ไม่ได้ขยับจริง: {self.mg400.movj_command(x, y, tz)}" if dry
                else f"ไปถึง X {x:.1f}  Y {y:.1f}  Z {tz:.1f} แล้ว")
        self.background(lambda: (self.mg400.safe_goto(x, y, z, az, down=down), done)[1],
                        "calib_done", "calib_fail")

    def test_gripper(self, on):
        """ทดสอบหัวดูดด้วยเลขขาที่แสดงอยู่บนหน้าจอ (ไม่ต้องกดบันทึกก่อน) แล้วบอกว่าส่งคำสั่งอะไรไป"""
        if not self.robot_free():
            return
        try:
            suck = self.read_number(self.fields["gripper_do"], "ดูด · ขา D")
            blow = self.read_number(self.fields["release_do"], "พ่นออก · ขา D")
            if suck is None or blow is None:
                raise ValueError("ใส่เลขขา ดูด และ พ่นออก ก่อน")
        except ValueError as e:
            self.cal_say(str(e), True)
            return
        self.mg400.cfg.update(gripper_do=int(suck), release_do=int(blow))
        suck, blow = int(suck), int(blow)
        pulse = self.read_number(self.fields["suck_pulse_s"], "เปิดดูดนาน") or 0
        self.mg400.cfg["suck_pulse_s"] = pulse
        sent = (f"DO({suck}, ON) เปิดดูด {pulse:g} วิ → DO({suck}, OFF)" if on and pulse
                else f"DO({blow}, OFF) → DO({suck}, ON) เปิดดูดค้าง" if on
                else f"DO({suck}, OFF) ปิดดูด → DO({blow}, ON) พ่น → DO({blow}, OFF)")
        if self.mg400.dry_run:
            self.cal_say(f"Dry run เปิดอยู่ หัวดูดจึงไม่ทำงาน ({sent}) · เอาติ๊ก Dry run ออกที่หน้าหลักก่อน",
                         True)
            return
        self.busy = True

        def run():
            mode = self.mg400.robot_mode()
            self.mg400.gripper(on)
            note = "" if mode == 5 else " · ⚠ แขนยังไม่ Enable ถ้าหัวดูดไม่ทำงาน ให้กด Enable ก่อน"
            return f"ส่งแล้ว: {sent}{note}"
        self.background(run, "calib_done", "calib_fail")

    # ================= แผงหุ่นยนต์ =================
    def using_mg400(self):
        return self.mode.get() == "mg400"

    def set_lamp(self, state, text):
        self.lamp.itemconfig(self.lamp_dot, fill=LAMP[state])
        self.conn_text.config(text=text)
        self.chip_dot.itemconfig(self.chip_dot_item, fill=LAMP[state])
        # แถบหัวเรื่องแสดงแบบสั้น เช่น "เชื่อมต่อแล้ว" หรือ "พร้อมเล่น"
        self.chip_conn.config(text=text.split(": ")[-1].split(" · ")[0])

    def refresh_robot_panel(self):
        real = self.using_mg400()
        connected = self.mg400.is_connected
        self.ip.config(state="normal" if real else "disabled")
        self.enable_button(self.btn_connect, real)
        for b in (self.btn_disconnect, self.btn_enable, self.btn_disable):
            self.enable_button(b, real and connected)
        if not real or not connected:
            self.pose_text.config(text="")
            self.pose_now.config(text="ยังไม่ได้เชื่อมต่อ (เลือก MG400 จริง แล้วกด เชื่อมต่อ ที่หน้าหลัก)")
        if not real:
            self.set_lamp("ok", "โหมดจำลอง: พร้อมเล่น")
        elif not connected:
            self.set_lamp("off", "ยังไม่ได้เชื่อมต่อ")
        self.update_header()

    def on_mode_change(self):
        if self.busy:
            self.mode.set("mg400" if self.mode.get() == "sim" else "sim")
            self.error.config(text="รอให้หุ่นยนต์ทำงานเสร็จก่อนจึงเปลี่ยนโหมดได้")
            return
        self.ctrl.robot = self.mg400 if self.using_mg400() else self.sim_robot
        self.conn_error.config(text="")
        self.refresh_robot_panel()
        self.set_view(camera=self.using_mg400())
        self.show_idle("เปลี่ยนโหมดแล้ว")

    def on_dry_change(self):
        if not self.dry.get():
            ok = messagebox.askyesno(
                "ยืนยันปิด Dry run",
                "แขนกลจะเคลื่อนที่จริงตามพิกัดใน robot_config.json\n\n"
                "- สอบเทียบพิกัดครบและถูกต้องแล้ว\n- ไม่มีคนหรือสิ่งของในพื้นที่ทำงาน\n"
                "- เอื้อมถึงปุ่ม E-Stop ได้ทันที\n\nยืนยันหรือไม่?")
            if not ok:
                self.dry.set(True)
        self.mg400.dry_run = self.dry.get()
        self.update_header()
        if self.mg400.dry_run:
            self.add_log("Dry run: เปิด")
        else:
            self.add_log("Dry run: ปิด (แขนจะเคลื่อนที่จริง)", "error")

    def background(self, func, ok_kind, fail_kind="conn_fail"):
        def work():
            try:
                self.events.put((ok_kind, "", {"result": func()}))
            except (MG400Error, OSError, ValueError) as e:
                self.events.put((fail_kind, str(e), {}))
        threading.Thread(target=work, daemon=True).start()

    def on_connect(self):
        ip = self.ip.get().strip()
        try:
            ipaddress.ip_address(ip)
        except ValueError:
            self.conn_error.config(text="รูปแบบ IP ไม่ถูกต้อง เช่น 192.168.1.6")
            return
        self.conn_error.config(text="")
        if self.mg400.is_connected and self.mg400.cfg.get("ip") == ip:
            self.add_log("เชื่อมต่ออยู่แล้ว · อ่านสถานะใหม่ (ไม่ตัดแล้วต่อใหม่ เพราะเครื่องจะปฏิเสธชั่วคราว)")
            self.background(self.mg400.status, "robot_status")
            return
        self.set_lamp("busy", f"กำลังเชื่อมต่อ {ip} ... (ถ้าเพิ่งตัดการเชื่อมต่อ อาจใช้เวลาถึง 20 วิ)")
        self.enable_button(self.btn_connect, False)
        self.background(lambda: self.mg400.connect(ip), "conn_ok")

    def on_disconnect(self):
        if self.busy:
            self.conn_error.config(text="รอให้หุ่นยนต์ทำงานเสร็จก่อน")
            return
        self.mg400.disconnect()
        self.add_log("ตัดการเชื่อมต่อหุ่นยนต์แล้ว")
        self.refresh_robot_panel()

    def on_enable(self):
        ok = messagebox.askyesno("ยืนยัน Enable",
                                 "เปิดมอเตอร์หุ่นยนต์?\n\nตรวจว่าไม่มีคนหรือสิ่งของในพื้นที่ทำงาน "
                                 "และเอื้อมถึงปุ่ม E-Stop ได้")
        if ok:
            self.set_lamp("busy", "กำลัง Enable ...")
            self.background(lambda: (self.mg400.enable(), self.mg400.status())[1], "robot_status")

    def on_disable(self):
        self.background(lambda: (self.mg400.disable(), self.mg400.status())[1], "robot_status")

    def show_robot_status(self, st):
        ready = st["mode"] == 5
        self.set_lamp("ok" if ready else ("error" if st["mode"] == 9 else "busy"),
                      f"เชื่อมต่อแล้ว · {st['mode_text']}")
        x, y, z, r = st["pose"]
        text = f"X {x:7.1f}  Y {y:7.1f}  Z {z:6.1f}  R {r:6.1f}"
        self.pose_text.config(text=text)
        self.pose_now.config(text=f"ตำแหน่งแขน  {text}   · {st['mode_text']}")

    def poll_robot_status(self):
        """อ่านสถานะหุ่นยนต์ทุก 2 วินาที (เฉพาะตอนเชื่อมต่อและว่างอยู่)"""
        if self.using_mg400() and self.mg400.is_connected and not self.busy and not self.polling:
            self.polling = True
            self.background(self.mg400.status, "robot_status", "conn_lost")
        self.root.after(2000, self.poll_robot_status)

    # ================= ภาพกล้อง =================
    def set_view(self, camera):
        """เปิด/ปิดกล้อง (MG400 จริงเปิดเอง · โหมดจำลองปิด แต่กดปุ่มเปิดดูได้)"""
        self.camera_view = camera
        self.warp_photo = None
        self.warp_canvas.itemconfig(self.warp_img, image="")
        self.markers, self.markers_seen = None, 0
        self.warp_card.note.config(text="")
        if camera and self.locked_markers is not None:
            self.set_board_note("🔒 ล็อกตำแหน่งกระดานแล้ว", SUCCESS)
        self.camera_view = camera
        self.clear_live()
        if camera:
            self.cam_msg_text("กำลังเปิดกล้อง ...")
            self.cam_status.config(text="กำลังค้นหากล้อง ...")
            self.start_camera()
        else:
            self.cam.stop()
            self.cam_msg_text("กล้องปิดอยู่ · กด เปิดกล้อง หรือเลือก MG400 จริง")
            self.cam_status.config(text="")
        self.cam_toggle.config(text="ปิดกล้อง" if camera else "เปิดกล้อง")
        self.draw_board(self.highlight)
        self.draw_preview()

    def toggle_camera(self):
        self.set_view(not self.camera_view)

    def cam_msg_text(self, text):
        """ข้อความกลางภาพ 'กระดานจากกล้อง' (ตอนยังไม่มีภาพ)"""
        self.warp_canvas.itemconfig(self.warp_msg, text=text)

    def start_camera(self):
        choice = self.cam_choice.get()
        self.cam.start(None if choice == CAM_AUTO else int(choice))

    def on_camera_change(self):
        choice = self.cam_choice.get()     # จำกล้องที่เลือกไว้ เปิดโปรแกรมครั้งหน้าใช้กล้องเดิม
        self.mg400.cfg["camera_index"] = None if choice == CAM_AUTO else int(choice)
        try:
            save_config(self.mg400.cfg)
        except OSError:
            pass
        self.relock()                      # เปลี่ยนกล้อง = มุมมองใหม่ ต้องหาตำแหน่งกระดานใหม่
        if self.camera_view:
            self.cam_msg_text("กำลังเปิดกล้อง ...")
            self.cam_status.config(text="")
            self.start_camera()

    def update_camera(self):
        """นำภาพล่าสุดจากกล้องมาแสดง ประมาณ 30 ภาพต่อวินาที"""
        if self.camera_view:
            frame = self.cam.latest()
            if frame is not None:
                h, w = frame.shape[:2]
                self.frame_size = (w, h)
                if self.locked_markers is None and self.mg400.cfg.get("board_lock"):
                    self.load_board_lock()
                self.cam_status.config(text=f"● ใช้กล้อง {self.cam.index} · {w}×{h}", fg=SUCCESS)
                # หากระดานทุก 5 เฟรมใน thread แยก (ไม่ให้ภาพสดกระตุก)
                self.frame_count += 1
                # ยังไม่ล็อก: หาทุก 5 เฟรม · ล็อกแล้ว: ตรวจทุก ~2 วินาทีว่ากระดานถูกขยับหรือไม่
                every = 5 if self.locked_markers is None else 60
                if not self.detecting and self.frame_count % every == 0:
                    self.detect_async(frame.copy())
                if self.frame_count % 6 == 0:          # ~5 ครั้ง/วินาที
                    self.live_tick(frame)
                if self.warp_card.winfo_ismapped():
                    self.draw_warp(frame)
                if self.settings_visible():
                    self.draw_preview(frame)
            elif self.cam.error:
                self.cam_msg_text(self.cam.error)
                self.cam_status.config(text="", fg=MUTED)
        self.root.after(33, self.update_camera)

    # ================= ตรวจหมากจากกล้อง =================
    def vision_active(self):
        """ใช้กล้องอ่านกระดานจริงไหม: MG400 จริง + เปิดกล้อง + มีภาพ + เปิดตรวจจับอัตโนมัติ"""
        return (self.auto_detect.get() and self.using_mg400() and self.camera_view
                and self.cam.latest() is not None)

    def sync_camera_source(self):
        """เลือกว่า controller อ่านกระดานจากกล้องจริง หรือจากกระดานจำลอง (คลิกบนจอ)"""
        if not self.busy:
            self.ctrl.camera = self.vcam if self.vision_active() else self.sim_camera

    def on_detect_change(self):
        self.auto_cand, self.auto_count = None, 0
        self.sync_camera_source()
        self.add_log("กล้องตรวจหมากอัตโนมัติ: " + ("เปิด" if self.auto_detect.get() else
                                                   "ปิด (คลิกบอกช่องบนจอ)"))

    def waiting_for_player(self):
        return (not self.busy and self.ctrl.result is None and not self.waiting_next
                and (self.ctrl.player_first or self.ctrl.board.count(EMPTY) < 9))

    def on_colors_change(self):
        """เปิด/ปิด การแยก X/O จากสีบล็อก แล้วจำไว้ในไฟล์ตั้งค่า"""
        on = bool(self.piece_colors.get())
        self.mg400.cfg["piece_colors"] = on
        self.vcam.use_colors = on
        try:
            save_config(self.mg400.cfg)
        except OSError:
            pass
        self.add_log("แยก X/O จากสีบล็อก (ฟ้า = X, แดง = O)" if on
                     else "แยก X/O จากตาเดิน · ใช้บล็อกสีอะไรก็ได้")
        self.live_hist.clear()

    def live_tick(self, frame):
        """อ่านกระดานจริงจากกล้องแบบเรียลไทม์: ป้าย X/O บนบล็อก + กระดานเกมแสดงสิ่งที่กล้องเห็น"""
        top = warp_board(frame, self.current_quad(), 150)
        gray = cv2.GaussianBlur(cv2.cvtColor(top, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        if self.prev_gray is not None and self.prev_gray.shape == gray.shape:
            motion = float(cv2.absdiff(gray, self.prev_gray).mean())
            self.still_ticks = self.still_ticks + 1 if motion < MOTION_STILL else 0
        self.prev_gray = gray
        hand_before = self.hand_present()
        self.hand_seen = hand_in_view(top)
        occ, marks, centers = occupied_cells(top), cell_marks(top), piece_centers(top)
        if self.vcam.use_colors:
            reading = [marks[i] or ("?" if occ[i] else None) for i in range(9)]
        else:
            # ไม่ดูสี: ช่องที่ยืนยันแล้วแสดงตามเกม · หมากใหม่ตอนตาผู้เล่น = X · อื่น ๆ = ?
            occ = [bool(occ[i] or marks[i]) for i in range(9)]
            marks = [None] * 9
            new_mark = PLAYER if self.waiting_for_player() else "?"
            reading = [(self.ctrl.board[i] if self.ctrl.board[i] != EMPTY else new_mark)
                       if occ[i] else None for i in range(9)]
        self.live_hist.append(reading)
        # โหวต 3 ครั้งล่าสุด กันป้ายกะพริบตอนมือผ่าน
        smoothed = []
        for i in range(9):
            vals = [h[i] for h in self.live_hist]
            best = max(set(vals), key=vals.count)
            smoothed.append(best if vals.count(best) * 2 > len(vals) else self.live_marks[i])
        self.live_centers = centers
        if smoothed != self.live_marks or self.hand_present() != hand_before:
            self.live_marks = smoothed
            self.paint_grid(self.canvas, cam=False)
            self.update_live_note()
        self.auto_detect_tick(occ, marks)

    def piece_quota(self):
        """จำนวนหมากที่แต่ละฝ่ายต้องลงทั้งเกม: ฝ่ายที่เริ่มก่อนลง 5 ตัว อีกฝ่ายลง 4 ตัว"""
        first = self.ctrl.player_first
        return {PLAYER: 5 if first else 4, ROBOT: 4 if first else 5}

    def turn_info(self):
        """ข้อความจำว่าใครเริ่มก่อน และลงไปแล้วกี่ตัว เช่น 'X เริ่มก่อน · X 2/5 · O 1/4'"""
        if self.ctrl.result == "WAIT":
            return ""
        quota, board = self.piece_quota(), self.ctrl.board
        starter = PLAYER if self.ctrl.player_first else ROBOT
        return (f"{starter} เริ่มก่อน · X {board.count(PLAYER)}/{quota[PLAYER]} "
                f"· O {board.count(ROBOT)}/{quota[ROBOT]}")

    def hand_present(self):
        """ยังมีมืออยู่บนกระดาน: เห็นสีผิว หรือภาพยังขยับอยู่ (ถือว่ายังวางไม่เสร็จ)"""
        return self.hand_seen or self.still_ticks < STILL_TICKS

    def update_live_note(self):
        info = self.turn_info()
        if not self.camera_view:
            self.board_card.note.config(text=info or "คลิกช่องที่วางหมาก X", fg=TEXT if info else MUTED)
            return
        nx, no = self.live_marks.count(PLAYER), self.live_marks.count(ROBOT)
        seen = f"กล้องเห็น X {nx} · O {no}"
        self.board_card.note.config(text=f"{info}   |   {seen}" if info else seen, fg=TEXT)

    def clear_live(self):
        self.live_marks, self.live_centers = [None] * 9, [None] * 9
        self.live_hist.clear()
        self.update_live_note()

    def live_board(self, occ, marks):
        """กระดานตามที่กล้องเห็นตอนนี้ เทียบกับกระดานที่ยืนยันแล้ว
        ช่องว่างที่มีของ (ไม่ใช่สีแดง) = X ใหม่ · ช่องที่มีหมากแล้วแต่กล้องเห็นว่าง = หมากหาย"""
        old = self.ctrl.board
        new = list(old)
        for i in range(9):
            seen = marks[i] or ("?" if occ[i] else None)
            if old[i] == EMPTY:
                if seen:
                    new[i] = ROBOT if seen == ROBOT else PLAYER
            elif seen is None:
                new[i] = EMPTY
            elif seen in (PLAYER, ROBOT):
                new[i] = seen
        return new

    def auto_detect_tick(self, occ, marks):
        """ระหว่างรอผู้เล่น: หมากใหม่ช่องเดียวนิ่ง ๆ ~1 วินาที = ผู้เล่นวางเสร็จ
        ถ้าเห็นการผิดกติกานิ่ง ๆ ~1.6 วินาที = แจ้งเตือนสั้น ๆ ว่าผิดอะไร"""
        if not (self.vision_active() and self.waiting_for_player()):
            self.auto_cand, self.auto_count = None, 0
            self.rule_cand, self.rule_count = None, 0
            return
        if not self.vcam.use_colors:       # ไม่ดูสี: มีของในช่อง = มีหมาก (สีอะไรก็ได้)
            occ = [bool(o or m) for o, m in zip(occ, marks)]
            marks = [None] * 9
        old = self.ctrl.board
        new = self.live_board(occ, marks)
        if new != old and self.hand_present():
            # มือยังขยับอยู่เหนือกระดาน: ยังไม่นับ ยังไม่เตือน รอให้วางเสร็จและเอามือออกก่อน
            self.auto_cand, self.auto_count = None, 0
            self.rule_cand, self.rule_count = None, 0
            self.set_status("รอให้เอามือออกจากกระดาน ...")
            return
        if new == old:                  # ยังไม่มีอะไรเปลี่ยน (หรือแก้กลับแล้ว)
            self.auto_cand, self.auto_count = None, 0
            self.rule_cand, self.rule_count = None, 0
            if self.rule_msg:
                self.clear_rule()
                self.set_status("แก้แล้ว · ตาผู้เล่น: วางหมาก X บนกระดานได้เลย")
            return
        err = rule_violation(old, new, PLAYER)
        if err:
            self.auto_cand, self.auto_count = None, 0
            if err != self.rule_cand:
                self.rule_cand, self.rule_count = err, 0
            self.rule_count += 1
            if self.rule_count >= RULE_STABLE_TICKS and err != self.rule_msg:
                self.show_rule(err)
            return
        self.rule_cand, self.rule_count = None, 0
        if self.rule_msg:
            self.clear_rule()
        cell = next(i for i in range(9) if new[i] != old[i])
        if cell != self.auto_cand:
            self.auto_cand, self.auto_count = cell, 0
        self.auto_count += 1
        self.set_status(f"กล้องเห็นหมากที่ช่อง {cell + 1} · กำลังยืนยัน ...")
        if self.auto_count >= AUTO_STABLE_TICKS:
            self.auto_cand, self.auto_count = None, 0
            self.add_log(f"กล้องตรวจพบหมาก X ที่ช่อง {cell + 1}"
                         + (" (บล็อกสีฟ้า)" if marks[cell] == PLAYER else ""))
            self.sync_camera_source()
            self.run_in_background(self.ctrl.player_done)

    # ---------- แจ้งเตือนผิดกติกา ----------
    def show_rule(self, msg):
        """ป้ายแดงบนกระดานเกม + แถบสถานะ บอกสั้น ๆ ว่าผิดอะไร (หายเองเมื่อแก้บนกระดานแล้ว)"""
        self.rule_msg = msg
        self.set_step(0)
        self.set_status(f"⚠ ผิดกติกา: {msg}", "error")
        self.error.config(text="แก้บนกระดานให้ถูกต้อง แล้วเกมจะเล่นต่อเอง")
        self.add_log(f"ผิดกติกา: {msg}", "error")
        self.add_history("ผิดกติกา", "", msg, "error")
        self.root.bell()
        self.paint_grid(self.canvas, cam=False)

    def clear_rule(self):
        if self.rule_msg is None:
            return
        self.rule_msg = None
        self.rule_cand, self.rule_count = None, 0
        self.error.config(text="")
        self.paint_grid(self.canvas, cam=False)

    def paint_rule(self, c, x0, y0, cell, tag):
        """ป้ายแดงกลางกระดาน: ⚠ ผิดกติกา + ข้อความสั้น"""
        if not self.rule_msg:
            return
        size = max(10, int(cell * 0.1))
        cx, cy = x0 + cell * 1.5, y0 + cell * 1.5
        head = c.create_text(cx, cy - 4, text="⚠ ผิดกติกา", fill="white", anchor="s",
                             font=(self.font_family, size + 4, "bold"), tags=tag)
        body = c.create_text(cx, cy + 2, text=self.rule_msg, fill="white", anchor="n",
                             width=cell * 2.8, justify="center",
                             font=(self.font_family, size, "bold"), tags=tag)
        hx0, hy0, hx1, _ = c.bbox(head)
        bx0, _, bx1, by1 = c.bbox(body)
        box = c.create_rectangle(min(hx0, bx0) - 16, hy0 - 10, max(hx1, bx1) + 16, by1 + 10,
                                 fill=DANGER, outline="white", width=3, tags=tag)
        c.tag_raise(head, box)
        c.tag_raise(body, box)

    def board_has_leftovers(self):
        frame = self.cam.latest()
        if frame is None:
            return []
        top = warp_board(frame, self.current_quad(), 150)
        occ, marks = occupied_cells(top), cell_marks(top)
        return [i + 1 for i in range(9) if occ[i] or marks[i]]

    # ================= กระดานจากกล้อง (vision.py) =================
    def detect_async(self, frame):
        self.detecting = True

        def work():
            try:
                markers, seen = find_markers_info(frame)
            except cv2.error:
                markers, seen = None, 0
            self.events.put(("markers", "", {"markers": markers, "seen": seen}))
        threading.Thread(target=work, daemon=True).start()

    def on_markers(self, markers, seen):
        self.detecting = False
        if not self.camera_view:
            return
        if self.locked_markers is not None:
            self.check_board_moved(markers, seen)
            return
        if markers is not None and seen == 4:     # ใช้เฉพาะครั้งที่เห็นครบ 4 จุดจริง (ไม่เดามุม)
            self.markers, self.markers_seen, self.markers_time = markers, seen, time.time()
            self.try_lock(markers)
        elif markers is not None:
            self.lock_candidates = []              # เห็นไม่ครบ: เริ่มนับใหม่
        if self.locked_markers is not None:
            return
        if self.markers_fresh():
            text, fg = "● พบครบ 4 จุด · กำลังล็อก ...", SUCCESS
        elif markers is not None:
            text, fg = f"● เห็นจุดมุม {seen}/4 · รอให้เห็นครบ 4 จุด (เอาแขน/มือออกจากกระดาน)", WARN
        else:
            text, fg = "● กำลังหาจุดมุม 4 จุดของแผ่นสนาม ...", WARN
        self.set_board_note(text, fg)

    def check_board_moved(self, markers, seen):
        """เทียบจุดมุมกระดานตอนนี้กับตอนกด บันทึกการตั้งค่า (calib_board)
        เตือนเมื่อเห็นกระดานเลื่อน "ไปที่เดียวกัน" ติดกัน 3 ครั้ง (~6 วิ) และขนาดกระดานเท่าเดิม
        (กันเตือนผิดจากการจับมุมผิดตัว เช่น เงาบล็อก/มือ ซึ่งได้กรอบขนาดเพี้ยนหรือกระโดดไปมา)"""
        ref = self.mg400.cfg.get("calib_board")
        if (not ref or markers is None or seen != 4
                or tuple(ref.get("frame", ())) != tuple(self.frame_size)):
            return     # ไม่มีค่าอ้างอิง หรือมือ/แขนบังมุม: ข้ามรอบนี้
        now, base = np.float32(markers), np.float32(ref["markers"])
        sides = lambda q: np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)
        if np.any(np.abs(sides(now) / np.maximum(sides(base), 1) - 1) > 0.1):
            return     # กรอบขนาดไม่เท่าเดิม = จับมุมผิดตัว ไม่ใช่กระดานเลื่อน: ข้ามรอบนี้
        shift = float(np.abs(now - base).max())
        moved = shift > BOARD_MOVED_PX
        last = getattr(self, "move_last", None)
        same_place = last is not None and float(np.abs(now - last).max()) <= 8
        self.move_last = now if moved else None
        self.move_checks = (self.move_checks + 1 if same_place else 1) if moved else 0
        if moved and self.move_checks >= 3 and not self.board_moved:
            msg = ("⚠ กระดานในภาพเลื่อน · ถ้าแค่กล้องขยับ กด 'กล้องขยับ · ใช้ตำแหน่งใหม่' "
                   "· ถ้ากระดานขยับ วางกลับที่เดิม หรือสอนช่อง 1, 3, 7 ใหม่")
            self.set_board_moved(True)
            self.set_board_note("⚠ กระดานในภาพเลื่อนจากตอนสอบเทียบ", DANGER)
            self.error.config(text=msg)
            self.add_log(f"{msg} (เลื่อนไป ~{shift:.0f} พิกเซล)", "error")
            self.root.bell()
        elif not moved and self.board_moved:
            self.set_board_moved(False)
            self.set_board_note("🔒 กระดานกลับมาอยู่ตำแหน่งเดิมแล้ว", SUCCESS)
            self.error.config(text="")
            self.add_log("กระดานกลับมาอยู่ตำแหน่งเดิมแล้ว", "ok")

    def set_board_note(self, text, fg):
        self.warp_card.note.config(text=text, fg=fg)
        self.preview_card.note.config(text=text, fg=fg)

    def try_lock(self, markers):
        """ล็อกตำแหน่งกระดานเมื่อหาจุดมุมได้ตรงกัน LOCK_SAMPLES ครั้งติด (ขยับไม่เกิน LOCK_TOL_PX)
        หลังล็อกแล้ว ตารางจะอยู่ที่เดิมเสมอ แขนบัง/แสงเปลี่ยน/โฟกัสพลาด ก็ไม่ทำให้ภาพกระโดด"""
        self.lock_candidates.append(np.float32(markers))
        self.lock_candidates = self.lock_candidates[-LOCK_SAMPLES:]
        if len(self.lock_candidates) < LOCK_SAMPLES:
            return
        mean = np.mean(self.lock_candidates, axis=0)
        if max(np.abs(c - mean).max() for c in self.lock_candidates) <= LOCK_TOL_PX:
            self.locked_markers = mean
            if self.accept_new_view:          # กล้องขยับ แต่กระดาน/แขนอยู่ที่เดิม: ใช้มุมมองนี้เป็นค่าอ้างอิงใหม่
                self.accept_new_view = False
                self.mg400.cfg["calib_board"] = {
                    "frame": list(self.frame_size),
                    "markers": [[round(float(x), 1), round(float(y), 1)] for x, y in mean]}
                self.set_board_moved(False)
                self.add_log("ใช้มุมกล้องใหม่แล้ว · พิกัดแขนเดิมยังใช้ได้", "ok")
            self.save_board_lock()
            self.set_board_note("🔒 ล็อกตำแหน่งกระดานแล้ว", SUCCESS)
            self.add_log("🔒 ล็อกกระดานแล้ว")

    def accept_camera_moved(self):
        """แค่กล้องขยับ (กระดานและแขนอยู่ที่เดิม): หาตำแหน่งกระดานในภาพใหม่ แล้วใช้เป็นค่าอ้างอิงใหม่
        พิกัดที่สอนแขนไว้ยังใช้ได้ ไม่ต้องสอนใหม่"""
        self.accept_new_view = True
        self.relock()
        self.set_board_note("● กำลังหาตำแหน่งกระดานในมุมกล้องใหม่ ... (เอามือ/แขนออกจากกระดาน)", WARN)

    def set_board_moved(self, moved):
        """เปิด/ปิด สถานะ "กระดานถูกขยับ" และปุ่ม "กล้องขยับ · ใช้ตำแหน่งใหม่" """
        self.board_moved = moved
        if moved:
            self.cam_moved_btn.pack(side="right", padx=(0, 6))
        else:
            self.cam_moved_btn.pack_forget()
            self.error.config(text="")

    def save_board_lock(self):
        """จำตำแหน่งที่ล็อกไว้ในไฟล์ตั้งค่า (ผูกกับขนาดภาพกล้อง) ใช้ต่อครั้งหน้าได้ทันที"""
        cfg = self.mg400.cfg
        cfg["board_lock"] = None if self.locked_markers is None else {
            "frame": list(self.frame_size),
            "markers": [[round(float(x), 1), round(float(y), 1)] for x, y in self.locked_markers]}
        try:
            save_config(cfg)
        except OSError:
            pass

    def load_board_lock(self):
        """ใช้ตำแหน่งที่ล็อกไว้ครั้งก่อน ถ้าขนาดภาพกล้องตรงกัน"""
        lock = self.mg400.cfg.get("board_lock")
        if (self.locked_markers is None and lock and
                tuple(lock.get("frame", ())) == tuple(self.frame_size)):
            self.locked_markers = np.float32(lock["markers"])
            self.set_board_note("🔒 ล็อกตำแหน่งกระดานแล้ว (จากครั้งก่อน)", SUCCESS)

    def relock(self):
        """ลืมตำแหน่งที่ล็อกไว้ แล้วหาจุดมุมใหม่ (ใช้เมื่อขยับกล้องหรือแผ่นสนาม)"""
        had = self.locked_markers is not None or self.mg400.cfg.get("board_lock")
        self.locked_markers, self.lock_candidates = None, []
        self.markers, self.markers_seen = None, 0
        if had:
            self.save_board_lock()
        self.set_board_note("● กำลังหาจุดมุม 4 จุดของแผ่นสนาม ...", WARN)

    def markers_fresh(self):
        if self.locked_markers is not None:
            return True
        return self.markers is not None and time.time() - self.markers_time < MARKER_HOLD_S

    def current_quad(self):
        """มุมทั้ง 4 ของตารางในภาพกล้อง: ตำแหน่งที่ล็อกไว้ > จุดมุมที่เพิ่งหาได้ > กรอบสีเขียว"""
        if self.locked_markers is not None:
            markers = self.locked_markers
        else:
            markers = self.markers if self.markers_fresh() else None
        return grid_quad(self.frame_size, markers, self.mg400.cfg.get("board_rotation", 0))

    def on_warp_resize(self, event):
        self.warp_w, self.warp_h = event.width, event.height
        self.warp_canvas.coords(self.warp_img, self.warp_w // 2, self.warp_h // 2)
        self.warp_canvas.coords(self.warp_msg, self.warp_w // 2, self.warp_h // 2)
        self.warp_canvas.itemconfig(self.warp_msg, width=max(100, self.warp_w - 30))
        self.paint_warp()

    def warp_geometry(self):
        s = max(30, min(self.warp_w, self.warp_h) - 8)
        return (self.warp_w - s) // 2, (self.warp_h - s) // 2, s / 3

    def draw_warp(self, frame):
        x0, y0, cell = self.warp_geometry()
        size = int(cell * 3)
        top = warp_board(frame, self.current_quad(), size)
        ok, buf = cv2.imencode(".ppm", top)
        if ok:
            self.warp_photo = tk.PhotoImage(data=buf.tobytes(), format="PPM")
            self.warp_canvas.itemconfig(self.warp_img, image=self.warp_photo)
            self.warp_canvas.itemconfig(self.warp_msg, text="")
        self.paint_warp()

    def paint_warp(self):
        """เส้นตาราง เลขช่อง และหมากที่โปรแกรมรู้ ทับบนภาพกระดานจากกล้อง"""
        c = self.warp_canvas
        c.delete("grid")
        if self.warp_photo is None:
            return
        x0, y0, cell = self.warp_geometry()
        line = "#22D3EE"
        if self.highlight is not None:
            r, col = divmod(self.highlight, 3)
            c.create_rectangle(x0 + col * cell, y0 + r * cell, x0 + (col + 1) * cell,
                               y0 + (r + 1) * cell, outline="#60A5FA", width=4, tags="grid")
        c.create_rectangle(x0, y0, x0 + cell * 3, y0 + cell * 3, outline=line, width=2, tags="grid")
        for k in (1, 2):
            c.create_line(x0 + k * cell, y0, x0 + k * cell, y0 + cell * 3, fill=line, width=1,
                          dash=(4, 3), tags="grid")
            c.create_line(x0, y0 + k * cell, x0 + cell * 3, y0 + k * cell, fill=line, width=1,
                          dash=(4, 3), tags="grid")
        for i in range(9):
            r, col = divmod(i, 3)
            c.create_text(x0 + col * cell + 6, y0 + r * cell + 4, text=str(i + 1), fill=line,
                          font=self.f_small, anchor="nw", tags="grid")
        # ป้าย X / O ทับบนตัวบล็อกจริงที่กล้องเห็น (ตามสีบล็อก: ฟ้า = X, แดง = O)
        size = cell * 3
        label_font = (self.font_family, max(12, int(cell * 0.32)), "bold")
        for i, mk in enumerate(self.live_marks):
            if not mk:
                continue
            r, col = divmod(i, 3)
            u, v = self.live_centers[i] or ((col + 0.5) / 3, (r + 0.5) / 3)
            x, y = x0 + u * size, y0 + v * size
            color = COLOR_X if mk == PLAYER else COLOR_O if mk == ROBOT else "#64748B"
            rad = cell * 0.24
            c.create_oval(x - rad, y - rad, x + rad, y + rad, fill="white", outline=color, width=3,
                          tags="grid")
            c.create_text(x, y, text=mk, fill=color, font=label_font, tags="grid")
        self.paint_result(c, x0, y0, cell, "grid")

    def cell_at(self, view, x, y):
        """แปลงตำแหน่งที่คลิกเป็นเลขช่อง 0-8 (หรือ None ถ้าคลิกนอกตาราง)"""
        if view == "warp":
            x0, y0, cell = self.warp_geometry()
            col, row = (x - x0) / cell, (y - y0) / cell
        else:
            x0, y0, cell = self.board_geometry()
            col, row = (x - x0) / cell, (y - y0) / cell
        if not (0 <= col < 3 and 0 <= row < 3):
            return None
        return int(row) * 3 + int(col)

    # ================= กระดาน =================
    def on_canvas_resize(self, event):
        self.cw, self.ch = event.width, event.height
        self.draw_board(self.highlight)

    def board_geometry(self, camera=False):
        """ตำแหน่งมุมซ้ายบนของตาราง และขนาดต่อช่อง บนกระดานเกม"""
        return grid_geometry(self.cw, self.ch, self.frame_size, False)

    def draw_board(self, highlight=None):
        """วาดกระดานเกม และเส้นตาราง/หมากบนภาพกระดานจากกล้อง"""
        self.highlight = highlight
        self.paint_grid(self.canvas, cam=False)
        self.paint_warp()
        self.update_live_note()

    def paint_grid(self, c, cam):
        highlight = self.highlight
        c.delete("overlay")
        x0, y0, cell = self.board_geometry(cam)
        line = COLOR_GUIDE if cam else "#334155"
        piece_font = (self.font_family, max(12, int(40 * cell / CELL)), "bold")

        if highlight is not None:
            r, col = divmod(highlight, 3)
            box = (x0 + col * cell, y0 + r * cell, x0 + (col + 1) * cell, y0 + (r + 1) * cell)
            if cam:
                c.create_rectangle(*box, outline="#60A5FA", width=4, tags="overlay")
            else:
                c.create_rectangle(*box, fill=PRIMARY_SOFT, outline="", tags="overlay")
        if cam:
            c.create_rectangle(x0, y0, x0 + cell * 3, y0 + cell * 3, outline=line, width=2,
                               tags="overlay")
        for k in (1, 2):
            c.create_line(x0 + k * cell, y0, x0 + k * cell, y0 + cell * 3,
                          width=2 if cam else 4, fill=line, capstyle="round", tags="overlay")
            c.create_line(x0, y0 + k * cell, x0 + cell * 3, y0 + k * cell,
                          width=2 if cam else 4, fill=line, capstyle="round", tags="overlay")
        for i, mark in enumerate(self.world.board):
            r, col = divmod(i, 3)
            x, y = x0 + col * cell + cell / 2, y0 + r * cell + cell / 2
            c.create_text(x0 + col * cell + 8, y0 + r * cell + 6, text=str(i + 1),
                          fill=line if cam else "#94A3B8", font=self.f_small,
                          anchor="nw", tags="overlay")
            rad = cell * 0.33
            live = self.live_marks[i] if self.camera_view else None
            if mark != EMPTY and self.camera_view and not live:
                if self.ctrl.result is not None:
                    continue             # จบเกมแล้ว และยกหมากออกไปแล้ว: ไม่ต้องแสดง (ตรงกับกล้อง)
                # กำลังเล่น แต่กล้องไม่เห็นหมากที่เกมจำไว้ (ถูกบัง/ถูกยกออก): แสดงจาง ๆ
                color = COLOR_X if mark == PLAYER else COLOR_O
                c.create_oval(x - rad, y - rad, x + rad, y + rad, fill="#F1F5F9", outline=color,
                              width=2, dash=(3, 3), tags="overlay")
                c.create_text(x, y, text=mark, fill=color, font=piece_font, tags="overlay")
                c.create_text(x, y + rad + 10, text="กล้องไม่เห็น", fill=MUTED, font=self.f_small,
                              tags="overlay")
            elif mark != EMPTY:          # หมากที่เกมยืนยันแล้ว: สีทึบ
                color = COLOR_X if mark == PLAYER else COLOR_O
                c.create_oval(x - rad, y - rad, x + rad, y + rad, fill=color, outline="",
                              tags="overlay")
                c.create_text(x, y, text=mark, fill="white", font=piece_font, tags="overlay")
            elif live and not self.hand_present():   # กล้องเห็นแต่เกมยังไม่ยืนยัน (มือออกแล้ว): เส้นประ
                color = COLOR_X if live == PLAYER else COLOR_O if live == ROBOT else "#94A3B8"
                soft = {PLAYER: "#FDE3D6", ROBOT: "#D3F2E6"}.get(live, "#EEF1F5")
                c.create_oval(x - rad, y - rad, x + rad, y + rad, fill=soft, outline=color,
                              width=3, dash=(6, 4), tags="overlay")
                c.create_text(x, y, text=live, fill=color, font=piece_font, tags="overlay")
                c.create_text(x, y + rad + 10, text="กล้องเห็น", fill=color, font=self.f_small,
                              tags="overlay")
        self.paint_result(c, x0, y0, cell, "overlay")
        if not cam:
            self.paint_rule(c, x0, y0, cell, "overlay")

    def display_board(self):
        """กระดานที่แสดงอยู่: หมากที่เกมยืนยันแล้ว + หมากที่กล้องเห็นบนกระดานจริง"""
        board = list(self.world.board)
        if self.camera_view:
            for i, mk in enumerate(self.live_marks):
                if board[i] == EMPTY and mk in (PLAYER, ROBOT) and not self.hand_present():
                    board[i] = mk
                elif board[i] != EMPTY and not mk and self.ctrl.result is not None:
                    board[i] = EMPTY      # จบเกมแล้ว ยกหมากออก: ไม่แสดงเส้นชนะ/ป้ายจบเกมค้าง
        return board

    def paint_result(self, c, x0, y0, cell, tag):
        """ถ้ามีฝ่ายชนะ: ขีดเส้นตรงผ่าน 3 ช่องที่ชนะ + ป้าย 'จบเกม' · ถ้าเต็มกระดานไม่มีใครชนะ: ป้ายเสมอ"""
        board = self.display_board()
        win = winning_line(board)
        if win is None and EMPTY in board:
            return
        if win:
            mark, (a, _, b) = win
            color = COLOR_X if mark == PLAYER else COLOR_O
            (ra, ca), (rb, cb) = divmod(a, 3), divmod(b, 3)
            xa, ya = x0 + (ca + 0.5) * cell, y0 + (ra + 0.5) * cell
            xb, yb = x0 + (cb + 0.5) * cell, y0 + (rb + 0.5) * cell
            dx, dy = (xb - xa) * 0.18, (yb - ya) * 0.18          # ยื่นเลยช่องปลายออกไปนิดหน่อย
            c.create_line(xa - dx, ya - dy, xb + dx, yb + dy, fill="white", width=16,
                          capstyle="round", tags=tag)
            c.create_line(xa - dx, ya - dy, xb + dx, yb + dy, fill=color, width=10,
                          capstyle="round", tags=tag)
            text = "จบเกม · " + ("ผู้เล่น (X) ชนะ" if mark == PLAYER else "หุ่นยนต์ (O) ชนะ")
        else:
            color, text = MUTED, "จบเกม · เสมอ"
        # ป้ายเล็ก ๆ ชิดขอบล่างด้านในกระดาน (บังหมากให้น้อยที่สุด และไม่ตกขอบภาพ)
        font = (self.font_family, max(10, int(cell * 0.1)), "bold")
        label = c.create_text(x0 + cell * 1.5, y0 + cell * 3 - 8, text=text, fill="white", font=font,
                              anchor="s", tags=tag)
        bx0, by0, bx1, by1 = c.bbox(label)
        box = c.create_rectangle(bx0 - 10, by0 - 3, bx1 + 10, by1 + 3, fill=color, outline="white",
                                 width=2, tags=tag)
        c.tag_raise(label, box)

    def set_step(self, index):
        for i, (dot, circle, num, lb) in enumerate(self.step_items):
            on = i == index
            dot.itemconfig(circle, fill=PRIMARY if on else "#E2E8F0")
            dot.itemconfig(num, fill="white" if on else MUTED)
            lb.config(fg=PRIMARY if on else MUTED,
                      font=(self.font_family, 9, "bold") if on else self.f_small)

    def set_status(self, text, tone="info"):
        self.status.config(text=text, fg=TONE[tone] if tone != "info" else TEXT)
        self.status_bar.config(bg=TONE[tone])

    def add_log(self, text, tag=None):
        self.log.config(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S  "), "time")
        self.log.insert("end", text + "\n", tag or ())
        self.log.see("end")
        self.log.config(state="disabled")

    def update_score(self):
        for key, lb in self.score_vals.items():
            lb.config(text=str(self.score[key]))

    # ================= เกม =================
    def robot_ready_or_warn(self):
        if self.using_mg400() and not self.mg400.is_connected:
            self.error.config(text="ยังไม่ได้เชื่อมต่อหุ่นยนต์ กดปุ่ม เชื่อมต่อ ก่อน")
            return False
        return True

    def run_in_background(self, func):
        self.busy = True
        threading.Thread(target=func, daemon=True).start()

    def on_start_button(self):
        if self.waiting_next:
            self.start_game()      # เกมถัดไปในแมตช์เดิม
        else:
            self.new_game()

    def on_play_mode_change(self):
        if self.busy:
            self.play_mode.set("match" if self.play_mode.get() == "single" else "single")
            self.error.config(text="รอให้หุ่นยนต์ทำงานเสร็จก่อนจึงเปลี่ยนรูปแบบได้")
            return
        self.show_idle("เปลี่ยนรูปแบบการเล่นแล้ว")

    def show_idle(self, reason=""):
        """ยังไม่เริ่มเกม: กระดานว่าง ห้ามคลิก/ตรวจจับ จนกว่าจะกดปุ่มเริ่มเกม"""
        if self.busy:
            return
        self.cancel_next()
        self.waiting_next = False
        self.clear_rule()
        self.match = None
        self.world.clear()
        self.ctrl.new_game()
        self.ctrl.result = "WAIT"       # กันไม่ให้คลิก/ตรวจจับจนกว่าจะกดเริ่มเกม
        self.draw_board()
        self.set_step(-1)
        self.update_match_view()
        self.update_start_button()
        self.error.config(text="")
        self.set_status((reason + " · " if reason else "") + f"พร้อมแล้ว กด {self.start_btn.cget('text')} "
                        "เพื่อเริ่มเล่น")

    def new_game(self):
        """เริ่มใหม่ทั้งหมด: เกมเดียว หรือแมตช์ใหม่ (เกมที่ 1)"""
        if self.busy:
            self.error.config(text="รอให้หุ่นยนต์ทำงานเสร็จก่อนจึงเริ่มเกมใหม่ได้")
            return
        self.match = Match() if self.play_mode.get() == "match" else None
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")
        self.history.delete(*self.history.get_children())
        self.start_game()

    def start_game(self):
        if self.busy:
            self.error.config(text="รอให้หุ่นยนต์ทำงานเสร็จก่อนจึงเริ่มเกมใหม่ได้")
            return
        self.cancel_next()
        self.waiting_next = False
        self.clear_rule()
        self.error.config(text="")

        self.sync_camera_source()
        if self.ctrl.camera is self.vcam:
            left = self.board_has_leftovers()
            if left:   # หมากค้างบนกระดาน กล้องจะเข้าใจผิดว่าเป็นตาเดินใหม่
                self.set_status("เก็บหมากออกจากกระดานให้หมดก่อน แล้วกดเริ่มเกมใหม่", "warn")
                self.error.config(text=f"กล้องเห็นของในช่อง {', '.join(map(str, left))} · "
                                       "ถ้าไม่มีอะไรจริง ให้ตรวจแสง หรือปิด 'กล้องตรวจหมาก' แล้วคลิกบอกเอง")
                self.world.clear()
                self.ctrl.new_game()
                self.ctrl.result = "WAIT"       # กันไม่ให้คลิก/ตรวจจับจนกว่าจะเริ่มใหม่
                self.draw_board()
                self.set_step(-1)
                return

        if self.board_moved and self.using_mg400():
            self.set_status("กระดานในภาพเลื่อนจากตอนสอบเทียบ · ยังเริ่มเกมไม่ได้", "warn")
            self.error.config(text="ถ้าแค่กล้องขยับ: กดปุ่ม 'กล้องขยับ · ใช้ตำแหน่งใหม่' ใต้ภาพกล้อง · "
                                   "ถ้ากระดานขยับ: วางกลับที่เดิม หรือสอนช่อง 1, 3, 7 ใหม่")
            return

        choice = self.first.get()
        player_first = random.choice((True, False)) if choice == "rand" else choice == "p"
        self.world.clear()
        self.ctrl.new_game(player_first=player_first)
        self.draw_board()
        self.set_step(-1)
        self.update_match_view()
        self.update_start_button()

        title = f"เกมที่ {self.match.game_number}" if self.match else "เริ่มเกมใหม่"
        who = "ผู้เล่น (X)" if player_first else "หุ่นยนต์ (O)"
        mode = "MG400 จริง" + (" (dry run)" if self.mg400.dry_run else "") if self.using_mg400() else "จำลอง"
        self.add_log(f"{title} · โหมด{mode}")
        self.add_log(("สุ่มได้: " if choice == "rand" else "") + f"{who} เริ่มก่อน")
        self.turn_no = 0
        self.add_history("เริ่มเกม", "", ("สุ่มได้: " if choice == "rand" else "") + f"{who} เริ่มก่อน",
                         "end")
        quota = self.piece_quota()
        self.add_log(f"{who} เริ่มก่อน ลง {quota[PLAYER if player_first else ROBOT]} ตัว · "
                     f"อีกฝ่ายลง {quota[ROBOT if player_first else PLAYER]} ตัว")
        need = (f" · วางบล็อกแดง {quota[ROBOT]} ก้อนที่จุดหยิบ" if self.using_mg400() else "")
        self.set_status(f"{title} · {who}เริ่มก่อน ลง 5 ตัว{need}")
        if not player_first and not self.robot_ready_or_warn():
            self.set_status("รอเชื่อมต่อหุ่นยนต์", "warn")
            return
        self.run_in_background(self.ctrl.start)

    # ---------- แมตช์ 2–3 เกม ----------
    def update_start_button(self):
        if self.waiting_next:
            text = f"เริ่มเกมที่ {self.match.game_number}"
        else:
            fresh = "" if self.ctrl.result == "WAIT" else "ใหม่"   # ยังไม่เคยเริ่ม = "เริ่มเกม"
            text = ("▶ เริ่มแมตช์" if self.play_mode.get() == "match" else "▶ เริ่มเกม") + fresh
        self.start_btn.config(text=text)

    def update_match_view(self):
        m = self.match
        self.layout_game_card(force=True)
        if m is None:
            return
        x, o = m.wins(PLAYER), m.wins("O")
        head = "จบแมตช์" if m.finished else "แมตช์"
        self.match_title.config(text=f"{head}  ผู้เล่น {x} – {o} หุ่นยนต์",
                                fg=TEXT if m.finished else MUTED)
        for i, lb in enumerate(self.match_pills):
            name = f"เกม {i + 1}"
            if i < len(m.results):
                r = m.results[i]
                lb.config(text=f"{name} · {RESULT_TEXT[r]}", fg=RESULT_COLOR[r], bg=SOFT)
            elif m.finished:
                lb.config(text=f"{name} · ไม่ต้องเล่น", fg="#CBD5E1", bg=SOFT)
            elif i == len(m.results):
                lb.config(text=f"{name} · {'รอเริ่ม' if self.waiting_next else 'กำลังเล่น'}",
                          fg=PRIMARY, bg=PRIMARY_SOFT)
            else:
                lb.config(text=f"{name} · {'เล่นถ้าเสมอ' if i == 2 else 'รอเล่น'}", fg=MUTED, bg=SOFT)

    def finish_match_game(self, result):
        """เรียกเมื่อจบหนึ่งเกมในแมตช์"""
        m = self.match
        m.add(result)
        if m.finished:
            text = f"จบแมตช์: {MATCH_TEXT[m.winner]} ({m.wins(PLAYER)} – {m.wins('O')})"
            self.set_status(text, "ok")
            self.add_log(text, "ok")
        else:
            self.waiting_next = True
            done = f"จบเกมที่ {len(m.results)}: {RESULT_TEXT[result]}"
            if self.using_mg400():
                # หุ่นยนต์จริง: ต้องให้คนเก็บหมากออกจากกระดานก่อน จึงไม่เริ่มเองอัตโนมัติ
                self.set_status(f"{done} · เก็บหมากออกจากกระดาน วางหมาก O ที่จุดหยิบ แล้วกด "
                                f"เริ่มเกมที่ {m.game_number}", "warn")
            else:
                self.countdown(done, NEXT_GAME_DELAY_S)
        self.update_match_view()
        self.update_start_button()

    def countdown(self, done, seconds):
        if seconds <= 0:
            self.next_job = None
            self.start_game()
            return
        self.set_status(f"{done} · เกมที่ {self.match.game_number} เริ่มใน {seconds} วินาที", "ok")
        self.next_job = self.root.after(1000, lambda: self.countdown(done, seconds - 1))

    def cancel_next(self):
        if self.next_job is not None:
            self.root.after_cancel(self.next_job)
            self.next_job = None

    def on_click(self, event, view="board"):
        self.error.config(text="")
        if self.ctrl.result is not None:
            if self.ctrl.result == "WAIT":
                text = f"ยังไม่เริ่มเกม กด {self.start_btn.cget('text')} ก่อน"
            else:
                text = "เกมนี้จบแล้ว" if self.waiting_next else "เกมจบแล้ว กด เริ่มเกมใหม่"
            self.error.config(text=text)
            return
        if self.busy:
            self.error.config(text="รอหุ่นยนต์ทำงานให้เสร็จก่อน")
            return
        if not self.robot_ready_or_warn():
            return
        if not self.ctrl.player_first and self.world.board.count(EMPTY) == 9:
            self.error.config(text="หุ่นยนต์ต้องเดินก่อน กด เริ่มเกมใหม่")
            return
        cell = self.cell_at(view, event.x, event.y)
        if cell is None:
            return
        if self.world.board[cell] != EMPTY:
            self.error.config(text=f"ช่อง {cell + 1} มีหมากแล้ว เลือกช่องอื่น")
            return
        self.world.place(cell, PLAYER)
        self.draw_board()
        self.sync_camera_source()
        if self.ctrl.camera is self.vcam:      # คลิกบอกเองระหว่างใช้กล้อง: เชื่อตามที่คลิก
            self.vcam.override = (cell, PLAYER)
        self.run_in_background(self.ctrl.player_done)

    # ================= รับเหตุการณ์จาก thread อื่น =================
    def poll_events(self):
        try:
            while True:
                self.handle_event(*self.events.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self.poll_events)

    def handle_event(self, kind, msg, data):
        if kind == "step":
            self.set_step(data["index"])
            self.set_status(msg)
        elif kind in ("log", "player_moved", "ai_move", "robot_moved"):
            self.add_log(("  - " if kind == "log" and not msg.startswith("    ") else "") + msg)
            if kind in ("ai_move", "robot_moved"):
                self.draw_board(highlight=data["cell"])
            if kind == "player_moved":
                self.clear_rule()
                if self.world.board[data["cell"]] == EMPTY:     # กล้องตรวจพบเอง: อัปเดตกระดานบนจอ
                    self.world.place(data["cell"], PLAYER)
                    self.draw_board(highlight=data["cell"])
                self.add_history("คน (X)", data["cell"] + 1,
                                 "กล้องตรวจพบ" if self.ctrl.camera is self.vcam else "วางหมาก", "X")
            elif kind == "robot_moved":
                self.add_history("AI / หุ่นยนต์ (O)", data["cell"] + 1,
                                 "หยิบและวาง · กล้องยืนยันแล้ว", "O")
        elif kind == "wait_player":
            self.busy = False
            self.sync_camera_source()
            if self.ctrl.camera is self.vcam:
                self.set_status("ตาผู้เล่น: วางหมาก X บนกระดานได้เลย กล้องจะตรวจเอง "
                                "· ตรวจว่ามีหมาก O รอที่จุดหยิบ")
            elif self.using_mg400():   # หุ่นยนต์หยิบที่เดิมทุกครั้ง ต้องมีหมากรออยู่ก่อนถึงตาหุ่นยนต์
                self.set_status("ตาผู้เล่น: วางหมาก X แล้วคลิกช่อง · ตรวจว่ามีหมาก O รอที่จุดหยิบ")
        elif kind == "game_over":
            self.busy = False
            self.set_step(-1)
            self.set_status(msg, "ok")
            self.add_log(msg, "ok")
            self.add_history("จบเกม", "", msg, "end")
            self.score[data["result"]] += 1
            self.update_score()
            self.draw_board()
            if self.match is not None:
                self.finish_match_game(data["result"])
        elif kind == "rule":       # ผู้เล่นผิดกติกา: เกมไม่หยุด รอให้แก้บนกระดาน
            self.busy = False
            if self.ctrl.camera is not self.vcam:   # โหมดคลิก: เอาหมากที่คลิกผิดออกจากกระดานจำลอง
                self.world.board[:] = list(self.ctrl.board)
            self.show_rule(msg)
        elif kind == "error":
            self.busy = False
            self.set_step(-1)
            self.set_status("หยุดทำงาน: ต้องตรวจสอบ", "error")
            self.error.config(text=msg)
            self.add_log("ผิดพลาด: " + msg, "error")
            self.add_history("ระบบ", "", "หยุดทำงาน: " + msg, "error")
        elif kind == "conn_ok":
            self.add_log(f"เชื่อมต่อ {self.mg400.cfg['ip']} สำเร็จ · ส่งคำสั่งเคลื่อนที่ทางพอร์ต "
                         f"{self.mg400.move_channel_name}", "ok")
            self.refresh_robot_panel()
            self.show_robot_status(data["result"])
            missing = self.mg400.missing_config()
            if missing:
                self.add_log("ยังไม่ได้สอบเทียบ: " + ", ".join(missing) + " (ใช้ได้เฉพาะ dry run)")
        elif kind == "conn_fail":
            self.refresh_robot_panel()
            if self.mg400.is_connected:
                self.set_lamp("error", "คำสั่งไม่สำเร็จ")
            self.conn_error.config(text=msg)
            self.add_log(f"ผิดพลาด: {msg}", "error")
        elif kind == "robot_status":
            self.polling = False
            self.poll_failures = 0
            self.show_robot_status(data["result"])
        elif kind == "markers":
            self.on_markers(data["markers"], data["seen"])
        elif kind == "pose_read":
            self.show_read_pose(*data["result"])
        elif kind == "jog_done":
            self.busy = False
            x, y, z, r = data["result"]
            self.pose_now.config(text=f"ตำแหน่งแขน  X {x:7.1f}  Y {y:7.1f}  Z {z:6.1f}  R {r:6.1f}")
            self.cal_say(f"ขยับแล้ว · ตอนนี้ X {x:.1f}  Y {y:.1f}  Z {z:.1f}")
        elif kind == "cal_progress":
            self.cal_say(msg)
        elif kind == "calib_done":
            self.busy = False
            self.cal_say(data["result"])
        elif kind == "calib_fail":
            self.busy = False
            self.cal_say(msg, True)
        elif kind == "conn_lost":
            # อ่านสถานะพลาดครั้งเดียวยังไม่ตัด (อาจเป็นแค่คำตอบช้า) ตัดเมื่อพลาด 3 ครั้งติด
            # หรือเมื่อเครื่องปิดการเชื่อมต่อเอง และบันทึกสาเหตุไว้ในบันทึกการทำงานเสมอ
            self.polling = False
            self.poll_failures += 1
            closed = ("ปิดการเชื่อมต่อจากฝั่งเครื่อง" in msg or "WinError" in msg
                      or "พอร์ต Move" in msg)
            if closed or self.poll_failures >= 3:
                self.mg400.disconnect()
                self.refresh_robot_panel()
                self.set_lamp("error", "การเชื่อมต่อหลุด")
                self.conn_error.config(text=msg)
                self.add_log(f"การเชื่อมต่อหลุด: {msg}", "error")
                self.poll_failures = 0
            else:
                self.set_lamp("busy", f"อ่านสถานะไม่สำเร็จ ครั้งที่ {self.poll_failures} · กำลังลองใหม่")
                self.add_log(f"อ่านสถานะไม่สำเร็จ ({self.poll_failures}/3): {msg}", "error")


if __name__ == "__main__":
    try:   # ให้ตัวอักษรคมชัดบน Windows ที่ตั้งขยายหน้าจอ 125–150%
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass
    root = tk.Tk()
    XOApp(root)
    root.mainloop()
