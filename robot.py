"""robot.py — ควบคุม Dobot MG400 ผ่าน TCP/IP (ใช้แทน SimRobot เมื่อมีเครื่องจริง)

ความปลอดภัย
- connect() แค่เปิดการเชื่อมต่อและอ่านสถานะ ไม่ทำให้แขนขยับ
- แขนจะขยับได้ก็ต่อเมื่อ: กด Enable แล้ว + ปิดโหมด dry_run + ใส่พิกัดใน robot_config.json ครบ
- dry_run=True (ค่าเริ่มต้น) จะแสดงคำสั่งที่ "จะส่ง" แต่ไม่ส่งคำสั่งเคลื่อนที่/กริปเปอร์จริง
- ปุ่ม E-Stop ที่ตัวเครื่องคือระบบหยุดหลักเสมอ ปุ่มในโปรแกรมเป็นแค่ตัวเสริม

ชื่อคำสั่ง พอร์ต และความหมายของ RobotMode อ้างอิงจาก TCP/IP protocol ของ MG400
ต้องตรวจกับเอกสารของเวอร์ชันเฟิร์มแวร์ที่ใช้จริงก่อนใช้งาน
"""

import json
import math
import os
import re
import shutil
import socket
import sys
import struct
import threading
import time

from game import ROBOT
from hardware import RobotBase

# ไฟล์ตั้งค่าอยู่ข้างโปรแกรม: รันจาก .py = โฟลเดอร์โค้ด · รันจาก .exe (PyInstaller) = โฟลเดอร์ของไฟล์ .exe
# (ถ้าใช้ __file__ ตอนเป็น .exe ไฟล์ตั้งค่าจะไปอยู่ในโฟลเดอร์ชั่วคราว แล้วหายเมื่อปิดโปรแกรม)
APP_DIR = (os.path.dirname(os.path.abspath(sys.executable)) if getattr(sys, "frozen", False)
           else os.path.dirname(os.path.abspath(__file__)))
CONFIG_FILE = os.path.join(APP_DIR, "robot_config.json")
MOVJ_FORMATS = {   # รูปแบบคำสั่งเคลื่อนที่ของเฟิร์มแวร์แต่ละรุ่น ({cmd} = MovL หรือ MovJ)
    "plain": "{cmd}({x:.2f},{y:.2f},{z:.2f},{r:.2f})",          # โปรโตคอล MG400 รุ่นเดิม (V3)
    "pose": "{cmd}(pose={{{x:.2f},{y:.2f},{z:.2f},{r:.2f}}})",   # โปรโตคอลรุ่นใหม่ (V4)
    "brace": "{cmd}({{{x:.2f},{y:.2f},{z:.2f},{r:.2f}}})",
}
MOVE_TYPES = ("MovL", "MovJ")   # MovL = หัวจับวิ่งเป็นเส้นตรง · MovJ = ข้อต่อหมุนพร้อมกัน (ทางโค้ง เร็วกว่า)
MAX_REACH_MM = 440   # ระยะเอื้อมสูงสุดของ MG400 จากแกนฐาน (ตามสเปกผู้ผลิต)
ARRIVE_MM = 1.5      # (พอร์ตที่ส่งสถานะตลอด) แขนห่างเป้าหมายไม่เกินเท่านี้ = ถึงแล้ว
# ข้อมูลสถานะที่พอร์ต 30003 ส่งมาทุก ~8 มิลลิวินาที (แพ็กเก็ตละ 1440 ไบต์ ตามเอกสาร Dobot TCP/IP)
FEED_SIZE = 1440
FEED_TEST = 0x0123456789ABCDEF      # ค่าตรวจสอบที่ไบต์ 48 ของทุกแพ็กเก็ต
FEED_MODE_AT = 24                   # robot_mode (uint64)
FEED_POSE_AT = 624                  # tool_vector_actual: X, Y, Z, R (double)
_TEXT_REPLY = re.compile(rb"-?\d+\s*,\s*\{[^{}]*\}\s*,\s*[A-Za-z]\w*\([^;]*\);")


def feed_packet(buf, i=0):
    """ถ้าไบต์ที่ตำแหน่ง i เป็นแพ็กเก็ตสถานะที่ถูกต้อง คืน (โหมด, [x, y, z, r]) ไม่ใช่คืน None"""
    if len(buf) - i < FEED_SIZE:
        return None
    if (struct.unpack_from("<H", buf, i)[0] != FEED_SIZE
            or struct.unpack_from("<Q", buf, i + 48)[0] != FEED_TEST):
        return None
    mode = struct.unpack_from("<Q", buf, i + FEED_MODE_AT)[0]
    return int(mode), list(struct.unpack_from("<4d", buf, i + FEED_POSE_AT))


def feed_bytes(mode, pose):
    """สร้างแพ็กเก็ตสถานะ (ใช้ในเทสต์)"""
    pkt = bytearray(FEED_SIZE)
    struct.pack_into("<H", pkt, 0, FEED_SIZE)
    struct.pack_into("<Q", pkt, 48, FEED_TEST)
    struct.pack_into("<Q", pkt, FEED_MODE_AT, mode)
    struct.pack_into("<4d", pkt, FEED_POSE_AT, *pose)
    return bytes(pkt)

DEFAULT_CONFIG = {
    "ip": "192.168.1.6", "dashboard_port": 29999, "move_port": 30003,
    "move_channel": "auto",   # auto / dashboard / move_port (ดู MG400Robot._open_move_channel)
    "cmd_terminator": "",     # ตัวปิดท้ายคำสั่ง Dashboard ("" หรือ "\n") · เครื่องในแล็บใช้ ""
    "move_terminator": "",    # ตัวปิดท้าย MovJ ทางพอร์ต Move · หาค่าที่ใช้ได้ด้วย check_move.py
    "movj_format": "plain",   # plain / pose / brace (ดู MOVJ_FORMATS · หาค่าที่ใช้ได้ด้วย check_move.py)
    "speed_percent": 20,
    "move_type": "MovL",      # ท่าลง/ขึ้น/ไปจุดจอด: MovL = เส้นตรง (ตรงดิ่ง แม่นยำ)
    "travel_move": "MovJ",    # ท่าข้ามไปมาที่ความสูง Approach: MovJ = ข้อต่อหมุนพร้อมกัน เร็วกว่า MovL มาก
    "suck_pulse_s": 2.0,      # ตอนหยิบ: เปิดดูดนานเท่านี้แล้วปิด (วินาที) · 0 = เปิดดูดค้างจนถึงตอนปล่อย
    "gripper_wait_s": 2.0,    # ตอนปล่อย: เปิดพ่นนานเท่านี้แล้วปิด (วินาที)
    # ---- จุดที่สอนไว้ (แต่ละจุดเก็บ X, Y, Z ของตัวเอง = ตำแหน่งหัวจับตอนจับ/ปล่อยหมากพอดี) ----
    "cell_xyz": None,     # [[x, y, z], ...] จุดวาง 9 ช่อง เรียงช่อง 1-9
    "pick_xyz": None,     # [[x, y, z], ...] จุดหยิบหมาก O สูงสุด 5 จุด หยิบเรียง 1→5 (ว่างได้บางจุด)
    "park_xyz": None,     # [x, y, z] จุดจอดนอกมุมกล้อง
    # ความสูง Approach (Safe Z) ค่าเดียว ใช้ทุกจุด: ลอยที่ Z นี้ → ลงตรงไปที่จุด → ยกกลับขึ้น Z นี้
    # เคลื่อนที่ข้ามจุดที่ความสูงนี้เสมอ (ต้องสูงพ้นหมากและของบนโต๊ะ)
    "approach_z": None,
    "r": 0,
    "board_rotation": 0,  # หมุนเลขช่องบนภาพกล้อง 0/90/180/270 (ปุ่ม "หมุนเลขช่อง" ในหน้าตั้งค่า)
    "board_lock": None,   # ตำแหน่งจุดมุมกระดานที่ล็อกไว้ {frame, markers} (ปุ่ม หาตำแหน่งกระดานใหม่ ล้างค่า)
    "calib_board": None,
    "camera_index": None, # หมายเลขกล้องที่เลือกไว้ (None = หาเองอัตโนมัติ) · จำไว้ใช้ครั้งหน้า
    "piece_colors": False,  # True = แยก X/O จากสีบล็อก (ฟ้า = X, แดง = O) · False = ตามตาเดิน (สีอะไรก็ได้)
    # หัวดูด: ขา DO ดูด (gripper_do) และขา DO ปล่อย/เป่าลม (release_do) · gripper_on_value = ค่าที่สั่งให้ขาทำงาน
    "gripper_do": 1, "release_do": 2, "gripper_on_value": 1,     # D1 = ดูด · D2 = ปล่อย (พ่นออก)
    "move_timeout_s": 60,     # Sync() รอแขนวิ่งเสร็จได้นานสุดเท่านี้ (วินาที)
}

ROBOT_MODES = {
    1: "เริ่มต้นระบบ", 2: "เบรกเปิด", 4: "ปิดมอเตอร์ (Disabled)", 5: "พร้อมทำงาน (Enabled)",
    6: "Backdrive", 7: "กำลังเคลื่อนที่", 8: "บันทึก", 9: "ผิดพลาด (Error)",
    10: "หยุดชั่วคราว", 11: "Jog",
}
MODE_READY = 5


class MG400Error(RuntimeError):
    pass


def load_config(path=CONFIG_FILE):
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            cfg.update(json.load(f))
    return migrate_config(cfg)


def migrate_config(cfg):
    """แปลงไฟล์ตั้งค่ารุ่นเก่า (X,Y + ความสูงรวม z_pick/z_place) เป็นแบบจุดละ X,Y,Z"""
    def with_z(p, z):
        return [p[0], p[1], z] if p and z is not None else None
    old_cells, z_place = cfg.pop("cell_xy", None), cfg.pop("z_place", None)
    old_pick, z_pick = cfg.pop("pick_xy", None), cfg.pop("z_pick", None)
    tray = cfg.pop("tray_xy", None)
    z_safe = cfg.pop("z_safe", None)
    if cfg.get("cell_xyz") is None and old_cells:
        cfg["cell_xyz"] = [with_z(p, z_place) for p in old_cells]
    if cfg.get("pick_xyz") is None and (old_pick or tray):
        points = tray if tray else [old_pick]
        cfg["pick_xyz"] = [with_z(p, z_pick) for p in points]
    # ระบบ Approach รุ่นเก่า (ยกเหนือแต่ละจุด approach_dz / จุด ★) → ความสูง Approach ค่าเดียว
    dz = cfg.pop("approach_dz", None)
    star = cfg.pop("board_approach_xyz", None)
    cfg.pop("pick_approach_xyz", None)
    for old in ("smooth_motion", "accel_percent", "cp_ratio", "wait_mode", "arrive_tolerance_mm"):   # ไม่ใช้แล้ว (เปลี่ยนเป็น MovL + Sync)
        cfg.pop(old, None)
    if cfg.get("approach_z") is None:
        if z_safe is not None:
            cfg["approach_z"] = z_safe
        elif star:
            cfg["approach_z"] = star[2]
        else:
            zs = [p[2] for k in ("cell_xyz", "pick_xyz") for p in (cfg.get(k) or []) if p]
            if zs:
                cfg["approach_z"] = round(max(zs) + float(dz or 40), 1)
    return cfg


def save_config(cfg, path=CONFIG_FILE):
    """บันทึกค่าลงไฟล์ ให้พิกัดแต่ละจุดอยู่บรรทัดละจุด อ่านและแก้ด้วยมือได้ง่าย"""
    keys = list(DEFAULT_CONFIG) + [k for k in cfg if k not in DEFAULT_CONFIG]
    lines = []
    for k in keys:
        v = cfg.get(k)
        if isinstance(v, list) and v and isinstance(v[0], list):
            text = "[\n    " + ",\n    ".join(json.dumps(p) for p in v) + "\n  ]"
        else:
            text = json.dumps(v, ensure_ascii=False)
        lines.append(f"  {json.dumps(k)}: {text}")
    if os.path.exists(path):   # เก็บค่าก่อนหน้าไว้ 1 ชุด เผื่อบันทึกผิด
        shutil.copyfile(path, backup_path(path))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("{\n" + ",\n".join(lines) + "\n}\n")
    os.replace(tmp, path)   # เขียนไฟล์ชั่วคราวก่อน ถ้าเขียนไม่สำเร็จไฟล์เดิมจะไม่เสีย


def backup_path(path=CONFIG_FILE):
    return os.path.splitext(path)[0] + ".backup.json"


def out_of_reach(cfg):
    """จุดที่อยู่ไกลจากฐานเกินระยะเอื้อมของ MG400 (ใช้เตือน ไม่ได้ห้าม)"""
    far = []
    for key, name in (("cell_xyz", "ช่อง"), ("pick_xyz", "จุดหยิบ")):
        for i, p in enumerate(cfg.get(key) or []):
            if p and math.hypot(p[0], p[1]) > MAX_REACH_MM:
                far.append(f"{name} {i + 1}")
    park = cfg.get("park_xyz")
    if park and math.hypot(park[0], park[1]) > MAX_REACH_MM:
        far.append("จุดจอด")
    return far


def low_points(cfg, margin):
    """จุดหยิบ/วางที่สูงเกินไปเมื่อเทียบกับความสูง Approach (ห่างไม่ถึง margin มม.)"""
    az = cfg.get("approach_z")
    if az is None:
        return []
    out = []
    for key, name in (("cell_xyz", "ช่อง"), ("pick_xyz", "จุดหยิบ")):
        for i, p in enumerate(cfg.get(key) or []):
            if p and az - p[2] < margin:
                out.append(f"{name} {i + 1}")
    return out


def grid_from_corners(p1, p3, p7):
    """คำนวณกึ่งกลางทั้ง 9 ช่อง จากช่อง 1, ช่อง 3 และช่อง 7 (ใช้ได้ทั้งแบบ X,Y และ X,Y,Z)"""
    out = []
    for i in range(9):
        r, c = divmod(i, 3)
        out.append([round(p1[k] + (p3[k] - p1[k]) * c / 2 + (p7[k] - p1[k]) * r / 2, 2)
                    for k in range(len(p1))])
    return out


_FULL_REPLY = re.compile(r"(-?\d+)\s*,\s*\{([^{}]*)\}\s*,\s*([A-Za-z]\w*)")   # 0,{5},RobotMode();


def dobot_studio_running():
    """DobotStudio Pro / DobotLink เปิดอยู่ในเครื่องนี้ไหม (ถ้าเปิด หุ่นยนต์จะปฏิเสธโปรแกรมอื่น)"""
    try:
        import subprocess
        out = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True, text=True,
                             timeout=5, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout
        return "DobotStudio Pro" in out or "DobotLink" in out
    except (OSError, subprocess.SubprocessError):
        return False


def _wire(cmd, term=""):
    """ข้อความที่ส่งจริง = คำสั่ง + ตัวปิดท้าย (term)
    MG400 ในแล็บใช้ ')' เป็นตัวจบคำสั่ง ถ้าต่อ '\\n' ท้าย ตัว '\\n' จะไปติดหน้าคำสั่งถัดไปจนอ่านค่าพัง
    (ทดสอบจริง: 'GetPose()' ได้ค่า · 'GetPose()\\n' ได้แค่ '\\nGetPose()') จึงใช้ "" เป็นค่าเริ่มต้น
    ตั้งใน robot_config.json ได้: cmd_terminator (Dashboard) / move_terminator (พอร์ต Move)"""
    return (cmd.strip() + term).encode("utf-8")


def _cmd_name(cmd):
    return cmd.split("(", 1)[0].strip()


def _strip_echo(cmd, text):
    """ตัดส่วนที่เครื่องสะท้อนชื่อคำสั่งกลับมา เช่น 'GetPose()' ออก เหลือแต่ค่า"""
    return re.sub(re.escape(_cmd_name(cmd)) + r"\s*\([^)]*\)\s*;?", " ", text)


def _has_value(cmd, text):
    rest = _strip_echo(cmd, text)
    return "{" in rest or bool(re.search(r"\d", rest))


def _reply_complete(cmd, text):
    """คำตอบแบบไม่มี ErrorID ครบแล้วหรือยัง (จะได้ไม่ต้องรอเครื่องเงียบ 0.15 วิ ทุกคำสั่ง)
    '{…}' ปิดวงเล็บแล้ว · ค่าเปล่าของ RobotMode เช่น '5' · คำสั่งที่ไม่มีค่าคืนสะท้อนชื่อกลับมาครบแล้ว"""
    rest = _strip_echo(cmd, text).strip()
    if re.match(r"-?\d+\s*,\s*\{", rest):
        return False                 # รูปแบบเต็ม 'ErrorID,{…},คำสั่ง;' ยังมาไม่ครบ ให้รอ ';'
    name = _cmd_name(cmd)
    if name == "RobotMode" or name.startswith("Get"):
        if "{" in rest:
            return "}" in rest.split("{", 1)[1]
        if name == "RobotMode":
            return bool(re.fullmatch(r"-?\d+", rest))
        # GetPose แบบไม่มีวงเล็บ: ครบเมื่อได้ 6 ค่า (X,Y,Z,R,0,0)
        return name == "GetPose" and bool(re.fullmatch(r"(-?\d+(\.\d+)?\s*,\s*){5}-?\d+(\.\d+)?", rest))
    return not rest and name in text   # คำสั่งที่ไม่มีค่าคืน: สะท้อนชื่อคำสั่งกลับมาแล้ว


def parse_reply(cmd, resp):
    """แยกค่าจากคำตอบ Dashboard คืนข้อความในวงเล็บปีกกา
    รองรับหลายรูปแบบ (เฟิร์มแวร์ต่างรุ่นตอบไม่เหมือนกัน):
      'ErrorID,{ค่า},คำสั่ง;'  ตามเอกสาร (ErrorID ไม่ใช่ 0 = ผิดพลาด)
      '4' หรือ '{4}'          ค่าเปล่า ๆ
      'GetPose()' + '{…}'     สะท้อนชื่อคำสั่งก่อน แล้วตามด้วยค่า (อาจมาเป็นหลายท่อน)
      'ClearError()'          สะท้อนชื่อคำสั่งอย่างเดียว สำหรับคำสั่งที่ไม่มีค่าคืน (คืน "")
    แบบที่ไม่มี ErrorID ไม่บอกว่าสำเร็จหรือไม่ จึงต้องตรวจผลด้วยการอ่านสถานะซ้ำ (ดู enable())"""
    name = _cmd_name(cmd)
    fulls = _FULL_REPLY.findall(resp)
    for err, value, reply_name in fulls:
        if reply_name == name or len(fulls) == 1:
            if err != "0":
                raise MG400Error(f"{cmd} ตอบกลับรหัสผิดพลาด {err}: {resp[:120]}")
            return value
    rest = _strip_echo(cmd, resp)
    m = re.search(r"\{([^{}]*)\}", rest)
    if m:
        return m.group(1).strip()
    bare = rest.strip(" \t\r\n;")
    if bare and re.fullmatch(r"[-+\d.eE,\s]+", bare):
        return re.sub(r"\s+", "", bare)
    if name and name in resp:
        return ""
    short = repr(resp[:80]) + (" …" if len(resp) > 80 else "")
    if "\x00" in resp:
        raise MG400Error(f"{cmd} ได้ข้อมูลไบนารี {len(resp)} ไบต์แทนคำตอบ "
                         "(พอร์ตนี้ส่งข้อมูลสถานะ ไม่ใช่พอร์ตรับคำสั่ง)")
    raise MG400Error(f"{cmd} ได้คำตอบที่อ่านไม่ออก: {short}")


class MG400Robot(RobotBase):
    def __init__(self, config=None, dry_run=True, sim_world=None, log=print):
        self.cfg = config or load_config()
        self.dry_run = dry_run
        self.sim_world = sim_world   # ใช้ตอนยังไม่มีกล้อง: วางเสร็จแล้วอัปเดตกระดานจำลองด้วย
        self.log = log
        self.dash = None
        self.move = None
        self.lock = threading.Lock()
        self.slot = 0
        self.last_reply = ("", "")
        self.move_streaming = False   # พอร์ต Move ส่งข้อมูลสถานะตลอด (ไม่ตอบเป็นข้อความ)
        self.feed_ok = False          # อ่านตำแหน่งจากข้อมูลสถานะนั้นได้ (ไม่ต้องถามเครื่อง)
        self._feed = None
        self._sync_seen = threading.Event()

    # ---------------- การเชื่อมต่อ ----------------
    def connect(self, ip=None):
        if ip:
            self.cfg["ip"] = ip
        self.disconnect()
        ip = self.cfg["ip"]
        try:
            self.dash = self._open(ip, self.cfg["dashboard_port"], "Dashboard", log=self.log)
            self.move = self._open_move_channel(ip)
        except MG400Error:
            self.disconnect()
            raise
        return self.status()

    def _open_move_channel(self, ip):
        """พอร์ตส่งคำสั่งเคลื่อนที่ (แบบ mg400_vision.py ของเพื่อน ซึ่งใช้ได้กับเครื่องนี้)
        move_channel = "auto" / "move_port" : พอร์ต Move (30003) · ส่ง MovL แล้วอ่านคำตอบ แล้วรอด้วย Sync()
                                            ("auto" เปิดพอร์ต Move ไม่ได้ จะใช้ Dashboard แทน)
        move_channel = "dashboard" : ส่งทาง Dashboard (เฟิร์มแวร์โปรโตคอลรุ่นใหม่ V4)"""
        channel = self.cfg.get("move_channel", "auto")
        self.move_streaming = self.feed_ok = False
        if channel == "dashboard":
            self.log("ส่งคำสั่งเคลื่อนที่ทางพอร์ต Dashboard (ตั้งค่า move_channel)")
            return self.dash
        port = self.cfg["move_port"]
        try:
            sock = self._open(ip, port, "Move", retries=8 if channel == "move_port" else 3,
                              log=self.log)
        except MG400Error:
            if channel == "move_port":
                raise
            self.log(f"เปิดพอร์ต {port} ไม่ได้ · ส่งคำสั่งเคลื่อนที่ทางพอร์ต Dashboard แทน")
            return self.dash
        if self._streams_data(sock):
            # เครื่องแบบนี้ (MG400 ในแล็บ) ส่งสถานะตลอด และไม่ตอบ MovL เป็นข้อความ:
            # ส่ง MovL อย่างเดียว แล้วดูตำแหน่งแขนจากข้อมูลสถานะที่ส่งมา (ไม่ต้องถามเครื่อง)
            self.move_streaming = True
            self._start_drain(sock)
            self.feed_ok = self._check_feed()
            self.log(f"พอร์ต {port}: ส่งสถานะตลอด · "
                     + ("ดูตำแหน่งแขนจากข้อมูลสถานะ" if self.feed_ok else "ถามตำแหน่งแขนทุก 0.3 วิ"))
        else:
            self.log(f"พอร์ต {port}: MovL + Sync()")
        return sock

    @staticmethod
    def _streams_data(sock, wait_s=0.5):
        """พอร์ตส่งข้อมูลมาเองโดยไม่ได้ถาม = พอร์ตส่งสถานะตลอด"""
        old = sock.gettimeout()
        sock.settimeout(wait_s)
        try:
            return bool(sock.recv(64))
        except OSError:
            return False
        finally:
            sock.settimeout(old)

    def _start_drain(self, sock):
        """อ่านข้อมูลสถานะที่เครื่องส่งมาตลอด (ไม่ให้บัฟเฟอร์เต็ม) และจำตำแหน่ง/โหมดล่าสุดไว้
        ถ้ามีข้อความคำตอบปนมา (เช่น Sync()) จะจดไว้ด้วย"""
        self._feed = None
        self._sync_seen.clear()
        marker = struct.pack("<Q", FEED_TEST)

        def drain():
            buf, noted = b"", False
            try:
                sock.settimeout(None)
                while True:
                    data = sock.recv(65536)
                    if not data:
                        break
                    window = buf[-256:] + data
                    if b"Sync(" in window:
                        self._sync_seen.set()
                    if not noted:                     # บันทึกครั้งเดียวว่าเครื่องตอบข้อความแบบไหน
                        m = _TEXT_REPLY.search(window)
                        if m:
                            noted = True
                            self.log(f"พอร์ต Move ตอบ: {m.group(0)[:60].decode('ascii', 'replace')}")
                    buf += data
                    while len(buf) >= FEED_SIZE:
                        got = feed_packet(buf)
                        if got:
                            self._feed = (time.time(), got[0], got[1])
                            buf = buf[FEED_SIZE:]
                            continue
                        k = buf.find(marker, 49)      # หาจุดเริ่มแพ็กเก็ตถัดไป
                        if k < 0:
                            buf = buf[-(FEED_SIZE + 48):]
                            break
                        buf = buf[k - 48:]
            except OSError:
                pass
        threading.Thread(target=drain, daemon=True).start()

    def _check_feed(self, wait_s=1.0):
        """ข้อมูลสถานะแกะได้ และตำแหน่งตรงกับ GetPose (±2 มม.) = เชื่อถือได้"""
        deadline = time.time() + wait_s
        while time.time() < deadline and not self._feed:
            time.sleep(0.02)
        if not self._feed:
            return False
        try:
            asked = self.get_pose(ask=True)
        except (MG400Error, OSError):
            return False
        return all(abs(a - b) <= 2.0 for a, b in zip(self._feed[2][:3], asked[:3]))

    def _live(self):
        """(โหมด, ตำแหน่ง) จากข้อมูลสถานะที่เพิ่งได้รับ (ไม่เกิน 0.5 วิ) · ไม่มีคืน None"""
        feed = self._feed
        if self.feed_ok and feed and time.time() - feed[0] < 0.5:
            return feed[1], feed[2]
        return None

    @property
    def move_channel_name(self):
        if self.move is None:
            return "-"
        return "Dashboard" if self.move is self.dash else "Move"

    @staticmethod
    def _open(ip, port, name, retries=8, log=None):
        """เปิดการเชื่อมต่อ 1 พอร์ต · ถ้าถูกปฏิเสธจะลองใหม่ได้นานถึง ~20 วิ
        (MG400 ใช้เวลาสักพักกว่าจะปล่อยการเชื่อมต่อเก่า หลังตัด/ต่อใหม่ หรือปิดแล้วเปิดโปรแกรมใหม่เร็ว ๆ)"""
        for attempt in range(retries):
            try:
                sock = socket.create_connection((ip, port), timeout=3)
                # ส่งทุกคำสั่งออกทันที ไม่ให้ระบบรวมคำสั่งสั้น ๆ หลายคำสั่งเป็นแพ็กเก็ตเดียว
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                return sock
            except ConnectionRefusedError:
                if attempt == retries - 1:
                    if dobot_studio_running():
                        raise MG400Error(
                            f"พอร์ต {name} ({ip}:{port}) ปฏิเสธการเชื่อมต่อ เพราะ DobotStudio Pro เปิดอยู่ในเครื่องนี้ "
                            "· ปิด DobotStudio Pro ให้หมด (รวม DobotLink ที่มุมขวาล่างของ Taskbar) "
                            "รอ 30 วินาที แล้วกดเชื่อมต่อใหม่")
                    raise MG400Error(
                        f"พอร์ต {name} ({ip}:{port}) ปฏิเสธการเชื่อมต่อ · มีโปรแกรมอื่นเชื่อมต่อพอร์ตนี้อยู่ "
                        "(DobotStudio Pro หรือโปรแกรม XO ที่ค้าง) หรือการเชื่อมต่อเก่ายังไม่ถูกปล่อย "
                        "· ปิดโปรแกรมอื่น รอ 10–30 วินาที แล้วลองใหม่")
                if log:
                    log(f"พอร์ต {name} ยังไม่ว่าง · ลองใหม่ {attempt + 1}/{retries - 1}")
                time.sleep(2.5)
            except OSError as e:
                raise MG400Error(f"เชื่อมต่อพอร์ต {name} ({ip}:{port}) ไม่ได้: {e} "
                                 "· ตรวจสาย LAN และ IP ของคอมให้อยู่วงเดียวกัน เช่น 192.168.1.100")

    def disconnect(self):
        """ปิดการเชื่อมต่อให้เรียบร้อย (แจ้งเครื่องว่าเลิกใช้แล้ว ก่อนปิด) เครื่องจะได้ปล่อยพอร์ตเร็ว"""
        self.move_streaming = self.feed_ok = False
        self._feed = None
        socks = {id(s): s for s in (self.dash, self.move) if s}.values()   # อาจเป็นตัวเดียวกัน
        self.dash = self.move = None
        for s in socks:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                s.close()
            except OSError:
                pass

    @property
    def is_connected(self):
        return self.dash is not None and self.move is not None

    def _send(self, sock, cmd):
        """ส่งคำสั่งแล้วคืนค่าในวงเล็บปีกกาของคำตอบ"""
        if sock is None:
            raise MG400Error("ยังไม่ได้เชื่อมต่อหุ่นยนต์")
        t0 = time.time()
        name = _cmd_name(cmd)
        query = name == "RobotMode" or name.startswith("Get")    # อ่านค่าอย่างเดียว ถามซ้ำได้ปลอดภัย
        slow = name in ("EnableRobot", "DisableRobot", "ClearError")   # เปิด/ปิดมอเตอร์ใช้เวลาหลายวินาที
        with self.lock:
            old_timeout = sock.gettimeout()
            if slow:
                sock.settimeout(max(old_timeout or 0, 15))
            try:
                resp = self._send_locked(sock, cmd, name, query)
            finally:
                sock.settimeout(old_timeout)
        took = time.time() - t0
        if took > 1.0 and name not in ("EnableRobot", "ClearError", "DisableRobot", "Sync"):
            self.log(f"⏱ {name} ช้า {took:.1f} วิ")         # จดเฉพาะคำสั่งที่ช้าผิดปกติ
        self.last_reply = (cmd, resp)       # เก็บไว้ดูตอนแก้ปัญหา
        return parse_reply(cmd, resp)

    def _send_locked(self, sock, cmd, name, query):
        """ส่ง-รับ 1 คำสั่ง (เรียกขณะถือ lock) · คำสั่งอ่านค่าที่ไม่ตอบ ถามใหม่ 1 ครั้ง"""
        for attempt in (1, 2):
            try:
                self._flush(sock)         # ทิ้งคำตอบเก่าที่ค้าง ไม่ให้คำตอบเลื่อนไม่ตรงคำสั่ง
                sock.sendall(_wire(cmd, self.cfg.get("cmd_terminator", "")))
                if sock is self.move and self.move is not self.dash and not self.move_streaming:
                    resp = self._recv_move_reply(sock, cmd)    # พอร์ต Move: อ่านจนเจอคำตอบของคำสั่งนี้
                else:
                    resp = self._recv_reply(sock, cmd)
                return resp
            except socket.timeout:
                if query and attempt == 1:
                    self.log(f"⏱ {name} ไม่ตอบ · ถามใหม่")
                    continue
                raise MG400Error(f"หุ่นยนต์ไม่ตอบคำสั่ง {cmd} ภายใน {sock.gettimeout():g} วิ "
                                 "(เครื่องตอบช้า หรือสาย LAN หลวม)")

    @staticmethod
    def _recv_move_reply(sock, cmd):
        """พอร์ต Move (แบบ mg400_vision.py): อ่านจนเจอคำตอบของคำสั่งนี้ เช่น '0,{},MovL(...);'
        ถ้าพอร์ตนี้ส่งข้อมูลสถานะ (ไบนารี) ปนมาด้วย จะข้ามส่วนนั้นไป หาเฉพาะข้อความคำตอบ
        รอได้ไม่เกินเวลา timeout ของ socket (MovL 3 วิ · Sync ตาม move_timeout_s)"""
        name = re.escape(_cmd_name(cmd)).encode()
        full = re.compile(rb"(-?\d+)\s*,\s*\{[^{}]*\}\s*,\s*" + name + rb"\([^;]*\);")
        echo = re.compile(name + rb"\([^)]*\)")
        deadline = time.time() + (sock.gettimeout() or 3)
        buf = b""
        while True:
            if time.time() > deadline:
                raise socket.timeout()
            chunk = sock.recv(65536)
            if not chunk:
                raise MG400Error("หุ่นยนต์ปิดการเชื่อมต่อพอร์ต Move")
            buf = (buf + chunk)[-65536:]
            m = full.search(buf)
            if m:
                return m.group(0).decode("ascii", errors="replace")
            text_only = b"\x00" not in buf
            if text_only and echo.search(buf):          # เครื่องที่ตอบแค่สะท้อนชื่อคำสั่ง
                return buf.decode("ascii", errors="replace").strip()

    @staticmethod
    def _flush(sock):
        old = sock.gettimeout()
        sock.setblocking(False)
        try:
            while sock.recv(4096):
                pass
        except (BlockingIOError, OSError):
            pass
        finally:
            sock.settimeout(old)

    @staticmethod
    def _recv_reply(sock, cmd="", quiet_s=0.15, max_s=1.5):
        """อ่านคำตอบให้ครบทุกท่อน: อ่านต่อจนเครื่องเงียบ (quiet_s) หรือครบเวลา max_s
        คำสั่งที่ต้องคืนค่า (RobotMode / Get…) จะรอจนกว่าจะได้ตัวค่า ไม่ใช่แค่ชื่อคำสั่งที่สะท้อนกลับมา"""
        data = sock.recv(4096).decode("utf-8", errors="replace")
        if not data:
            raise MG400Error("หุ่นยนต์ปิดการเชื่อมต่อจากฝั่งเครื่อง (มักเกิดเมื่อโปรแกรมอื่น "
                             "เช่น DobotStudio Pro เชื่อมต่อเข้ามาแทน)")
        name = _cmd_name(cmd)
        need_value = name == "RobotMode" or name.startswith("Get")
        deadline = time.time() + max_s
        old = sock.gettimeout()
        try:
            while time.time() < deadline:
                stripped = data.rstrip()
                if stripped.endswith(";") and _FULL_REPLY.search(data):
                    break                                   # คำตอบเต็มตามเอกสาร ครบแล้ว
                if _reply_complete(cmd, data):
                    break                                   # ค่าเปล่า/สะท้อนชื่อ ครบแล้ว ไม่ต้องรอเงียบ
                sock.settimeout(quiet_s)
                try:
                    more = sock.recv(4096).decode("utf-8", errors="replace")
                except socket.timeout:
                    if not need_value or _has_value(cmd, data):
                        break                               # เงียบแล้ว และได้สิ่งที่ต้องการแล้ว
                    continue
                if not more:
                    break
                data += more
        finally:
            sock.settimeout(old)
        return data.strip()

    # ---------------- สถานะ ----------------
    def robot_mode(self):
        live = self._live()
        if live:
            return live[0]
        return int(float(self._send(self.dash, "RobotMode()")))

    def get_pose(self, ask=False):
        """[x, y, z, r] · ถ้ามีข้อมูลสถานะจากพอร์ต Move ใช้อันนั้น (ไม่ต้องถามเครื่อง) · ask=True ถามเสมอ"""
        live = None if ask else self._live()
        if live:
            return list(live[1])
        values = self._send(self.dash, "GetPose()").split(",")
        if len(values) < 4:
            raise MG400Error(f"GetPose() ได้ค่าไม่ครบ: {self.last_reply[1]!r}")
        return [float(v) for v in values[:4]]

    def status(self):
        mode = self.robot_mode()
        return {"mode": mode, "mode_text": ROBOT_MODES.get(mode, f"ไม่ทราบ ({mode})"),
                "pose": self.get_pose()}

    def enable(self, timeout_s=15):
        """ClearError → EnableRobot → รอโหมด 5 → SpeedFactor (เหมือน mg400_vision.py)"""
        self._send(self.dash, "ClearError()")
        self._send(self.dash, "EnableRobot()")
        deadline = time.time() + timeout_s
        mode = self.robot_mode()
        while mode != MODE_READY and time.time() < deadline:
            time.sleep(0.5)
            mode = self.robot_mode()
        if mode != MODE_READY:
            raise MG400Error(f"Enable ไม่สำเร็จ: หุ่นยนต์ยังอยู่โหมด {ROBOT_MODES.get(mode, mode)} "
                             "· ตรวจปุ่ม E-Stop / ไฟ Error ที่ตัวเครื่อง / ปิด DobotStudio Pro")
        self._send(self.dash, f"SpeedFactor({int(self.cfg['speed_percent'])})")
        for cmd in ("SpeedJ(100)", "SpeedL(100)", "AccJ(100)", "AccL(100)"):
            try:     # ค่าเริ่มต้นของเครื่องอาจต่ำ (ออกตัว/เบรกช้า) · เครื่องที่ไม่มีคำสั่งนี้ก็ข้าม
                self._send(self.dash, cmd)
            except MG400Error:
                pass

    def disable(self):
        self._send(self.dash, "DisableRobot()")

    # ---------------- การเคลื่อนที่ (แบบเดียวกับ mg400_vision.py) ----------------
    #   1 ท่า = MovL(...) → อ่านคำตอบ (เครื่องปฏิเสธ = แจ้งทันที) → Sync() → เครื่องตอบเมื่อวิ่งเสร็จ
    #   ระหว่างแขนวิ่ง ไม่ถามตำแหน่ง/สถานะเครื่องเลย
    def movj_command(self, x, y, z, travel=False):
        """คำสั่งเคลื่อนที่ไป (x, y, z) · travel=True = ท่าข้ามที่ความสูง Approach (ใช้ travel_move)
        ท่าอื่น (ลง/ขึ้น/จุดจอด) ใช้ move_type"""
        fmt = MOVJ_FORMATS.get(self.cfg.get("movj_format", "plain"), MOVJ_FORMATS["plain"])
        cmd = self.cfg.get("travel_move", "MovJ") if travel else self.cfg.get("move_type", "MovL")
        return fmt.format(cmd=cmd if cmd in MOVE_TYPES else "MovL", x=x, y=y, z=z, r=self.cfg["r"])

    def _sync(self):
        """Sync(): เครื่องตอบกลับเมื่อวิ่งครบทุกท่าที่ส่งไปแล้ว (รอได้ไม่เกิน move_timeout_s)"""
        timeout = float(self.cfg.get("move_timeout_s") or 60)
        old = self.move.gettimeout()
        self.move.settimeout(timeout)
        try:
            self._send(self.move, "Sync()")
        except MG400Error as e:
            raise MG400Error(f"แขนวิ่งไม่เสร็จภายใน {timeout:g} วิ ({e}) "
                             "· ตรวจปุ่ม E-Stop / ไฟ Error ที่ตัวเครื่อง")
        finally:
            self.move.settimeout(old)

    def _send_move(self, cmd):
        """ส่งคำสั่งเคลื่อนที่: พอร์ตตอบข้อความ = รอคำตอบ · พอร์ตส่งสถานะตลอด = ส่งอย่างเดียว"""
        if self.move_streaming:
            self.move.sendall(_wire(cmd, self.cfg.get("move_terminator", "")))
            time.sleep(0.05)   # เว้นจังหวะ ไม่ให้ 2 คำสั่งติดกันในแพ็กเก็ตเดียว
        else:
            self._send(self.move, cmd)

    def _wait_done(self, target):
        """รอแขนวิ่งเสร็จ · พอร์ตตอบข้อความ: Sync() · พอร์ตส่งสถานะตลอด: ดูตำแหน่งจนถึงเป้าหมาย"""
        if not self.move_streaming:
            self._sync()
            return
        self._sync_seen.clear()
        self._send_move("Sync()")      # ถ้าเครื่องตอบ Sync() ปนมา จะรู้เร็วขึ้น
        x, y, z = target
        timeout = float(self.cfg.get("move_timeout_s") or 60)
        deadline = time.time() + timeout
        gap = 0.02 if self.feed_ok else 0.3   # ข้อมูลสถานะดูได้ถี่ · ถามเครื่องต้องห่าง ๆ
        pose = None
        while time.time() < deadline:
            live = self._live()
            mode, pose = live if live else (None, self.get_pose())
            if mode == 9:
                raise MG400Error("หุ่นยนต์เข้าสู่สถานะ Error ระหว่างเคลื่อนที่ (ตรวจ E-Stop / ไฟ Error)")
            if all(abs(a - b) <= ARRIVE_MM for a, b in zip(pose[:3], (x, y, z))):
                return
            if self._sync_seen.wait(gap):
                self._sync_seen.clear()
        at = "" if pose is None else f" · แขนอยู่ที่ ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f})"
        raise MG400Error(f"แขนไปไม่ถึง ({x:.1f}, {y:.1f}, {z:.1f}) ภายใน {timeout:g} วิ{at} "
                         "· ตรวจ E-Stop / ไฟ Error / ตำแหน่งเกินระยะเอื้อม")

    def move_to(self, x, y, z):
        """เคลื่อนที่ 1 ท่า แล้วรอจนวิ่งเสร็จ"""
        self.move_path([(x, y, z)])

    def move_path(self, points, stops=()):
        """ส่ง MovL ทีละจุด และ Sync() (รอวิ่งเสร็จ) ที่จุดใน stops และจุดสุดท้าย
        จุดที่ไม่อยู่ใน stops จะส่งต่อกันไปเลย เครื่องวิ่งต่อไม่ต้องรอโปรแกรม"""
        # จุด (x, y, z) = ท่าลง/ขึ้น (MovL) · (x, y, z, "J") = ท่าข้ามที่ความสูง Approach (MovJ)
        travel = [len(p) > 3 and p[3] == "J" for p in points]
        points = [tuple(p[:3]) for p in points]
        if self.dry_run:
            for p, t in zip(points, travel):
                self.log(f"    [dry run] {self.movj_command(*p, travel=t)}")
            return True
        t0, last = time.time(), len(points) - 1
        for i, (x, y, z) in enumerate(points):
            self._send_move(self.movj_command(x, y, z, travel=travel[i]))
            if i == last or i in stops:
                self._wait_done((x, y, z))
        self.log(f"⏱ วิ่ง {len(points)} ท่า {time.time() - t0:.1f} วิ")
        return True

    def gripper(self, on):
        """ดูด (on=True):  ปิดขาพ่น → เปิดขาดูด → รอ suck_pulse_s → ปิดขาดูด (หัวดูดจับค้างเอง)
                          (suck_pulse_s = 0: เปิดดูดค้างไว้จนถึงตอนปล่อย)
        ปล่อย (on=False): ปิดขาดูด → เว้นจังหวะ → เปิดขาพ่น → รอ gripper_wait_s → ปิดขาพ่น"""
        act = int(self.cfg["gripper_on_value"])
        suck, blow = self.cfg["gripper_do"], self.cfg.get("release_do")
        wait = max(0.0, float(self.cfg.get("gripper_wait_s", 2.0)))
        pulse = max(0.0, float(self.cfg.get("suck_pulse_s", 2.0)))
        if on:
            steps = ([(f"DO({blow},{1 - act})", "ปิดพ่น", 0)] if blow is not None else [])
            if pulse > 0:
                steps += [(f"DO({suck},{act})", f"เปิดดูด {pulse:g} วิ", pulse),
                          (f"DO({suck},{1 - act})", "ปิดดูด", 0)]
            else:
                steps += [(f"DO({suck},{act})", "เปิดดูด (ค้างไว้)", wait)]
        else:
            steps = [(f"DO({suck},{1 - act})", "ปิดดูด", 0.2 if blow is not None else wait)]  # หยุดดูดก่อนพ่น
            if blow is not None:
                steps += [(f"DO({blow},{act})", "เปิดพ่น", wait), (f"DO({blow},{1 - act})", "ปิดพ่น", 0)]
        for cmd, meaning, pause in steps:
            pin, value = cmd[3:-1].split(",")                  # เขียนแบบ DobotStudio: DO(0, ON)
            shown = f"DO({pin}, {'ON' if value == str(act) else 'OFF'})"
            self.log(f"{'[dry run] ' if self.dry_run else ''}{shown} {meaning}")   # เห็นในบันทึกการทำงาน
            if self.dry_run:
                continue
            self._send(self.dash, cmd)
            time.sleep(pause)

    def calib_move(self, x, y, z):
        """ใช้ตอนสอบเทียบ: ขยับไปยังจุดเดียว (ไม่ต้องตั้งค่าครบทุกจุด)"""
        if not self.is_connected:
            raise MG400Error("ยังไม่ได้เชื่อมต่อหุ่นยนต์")
        if not self.dry_run:
            mode = self.robot_mode()
            if mode != MODE_READY:
                raise MG400Error(f"หุ่นยนต์ยังไม่พร้อม: {ROBOT_MODES.get(mode, mode)} (กด Enable ก่อน)")
        self.move_to(x, y, z)

    def safe_goto(self, x, y, z, approach_z=None, down=True):
        """ไปจุดแบบปลอดภัย (ใช้ตอนสอบเทียบ): ยกขึ้นความสูง Approach ที่ตำแหน่งเดิม → ข้ามไปเหนือจุด
        → ลงที่จุด (down=False หยุดเหนือจุด) · ถ้าอยู่ตรงจุดนั้นอยู่แล้ว ขยับขึ้น/ลงตรง ๆ อย่างเดียว"""
        if approach_z is None:
            self.calib_move(x, y, z)
            return
        az = float(approach_z)
        path = []
        if self.is_connected:
            cx, cy, cz, _ = self.get_pose()
            if math.hypot(cx - x, cy - y) > 0.5 and cz < az - 0.5:
                path.append((cx, cy, az))
        path.append((x, y, az, "J"))
        if down:
            path.append((x, y, z))
        if not self.is_connected:
            raise MG400Error("ยังไม่ได้เชื่อมต่อหุ่นยนต์")
        if not self.dry_run:
            mode = self.robot_mode()
            if mode != MODE_READY:
                raise MG400Error(f"หุ่นยนต์ยังไม่พร้อม: {ROBOT_MODES.get(mode, mode)} (กด Enable ก่อน)")
        # ยกขึ้นตรงดิ่งก่อน (ถ้ามี) และถึงเหนือจุดตรงเป๊ะก่อนลง
        self.move_path(path, stops=tuple(range(len(path) - 1)))

    def visit_points(self, points, approach_z, progress=None, stop=None, dwell_s=0.0):
        """ทดสอบจุดก่อนเล่น เส้นทางเดียวกับตอนเล่นจริง ทีละจุด:
        เหนือจุด (Z Approach) → ลงตรงดิ่ง → ยกขึ้นตรงดิ่ง
        points = [(ชื่อจุด, [x, y, z]), ...] · stop = threading.Event (หยุดหลังจบจุดที่ทำอยู่
        แขนจะอยู่เหนือจุดเสมอ ไม่ค้างต่ำ) · คืน True เมื่อครบทุกจุด, False เมื่อถูกสั่งหยุด"""
        say = progress or (lambda m: None)
        az = float(approach_z)
        if not self.is_connected:
            raise MG400Error("ยังไม่ได้เชื่อมต่อหุ่นยนต์")
        if not self.dry_run:
            mode = self.robot_mode()
            if mode != MODE_READY:
                raise MG400Error(f"หุ่นยนต์ยังไม่พร้อม: {ROBOT_MODES.get(mode, mode)} (กด Enable ก่อน)")
        for k, (name, (x, y, z)) in enumerate(points):
            if stop is not None and stop.is_set():
                say("หยุดการทดสอบแล้ว")
                return False
            say(f"{name} ({k + 1}/{len(points)})")
            self.move_path([(x, y, az, "J"), (x, y, z), (x, y, az)], stops=(0, 1))
            if dwell_s and not self.dry_run:
                time.sleep(dwell_s)
        return True

    # ---------------- ตามสัญญา RobotBase ----------------
    def missing_config(self):
        """ค่าที่ยังไม่ครบ: ช่องวางต้องครบ 9 · จุดหยิบต้องมีอย่างน้อย 1 · จุดจอด · ความสูง Approach"""
        c = self.cfg
        missing = []
        cells = c.get("cell_xyz")
        if not cells or len(cells) < 9 or any(p is None for p in cells[:9]):
            missing.append("cell_xyz")
        if not self.pick_points():
            missing.append("pick_xyz")
        if c.get("park_xyz") is None:
            missing.append("park_xyz")
        if c.get("approach_z") is None:
            missing.append("approach_z")
        return missing

    def pick_points(self):
        """จุดหยิบที่สอนไว้แล้ว (ข้ามช่องที่ยังว่าง) · คืน [(เลขจุด 1-5, [x, y, z]), ...]"""
        return [(i + 1, p) for i, p in enumerate(self.cfg.get("pick_xyz") or []) if p]

    def check_ready(self):
        if not self.is_connected:
            raise MG400Error("ยังไม่ได้เชื่อมต่อหุ่นยนต์")
        if self.dry_run:
            return
        missing = self.missing_config()
        if missing:
            raise MG400Error("ยังไม่ได้สอบเทียบใน robot_config.json: " + ", ".join(missing))
        mode = self.robot_mode()
        if mode != MODE_READY:
            raise MG400Error(f"หุ่นยนต์ยังไม่พร้อม: {ROBOT_MODES.get(mode, mode)} (กด Enable ก่อน)")

    def reset_tray(self):
        self.slot = 0     # เริ่มเกมใหม่: กลับไปหยิบจากจุดหยิบที่ 1

    def _point(self, key, index=None, default=(0.0, 0.0, 0.0)):
        point = self.cfg.get(key)
        if index is not None:
            point = point[index] if point and index < len(point) else None
        return list(point) if point else list(default)

    def place_piece(self, cell, progress=None):
        """ตาหุ่นยนต์: หยิบจากจุดหยิบลำดับถัดไป (1→5 วนใหม่ถ้าหมด) แล้ววางที่ช่อง cell"""
        say = progress or (lambda m: None)
        self.check_ready()
        picks = self.pick_points() or [(1, None)]
        number, point = picks[self.slot % len(picks)]
        self._pick_and_place(cell, say, pick=point, pick_no=number)
        self.slot += 1
        if self.sim_world is not None:
            self.sim_world.place(cell, ROBOT)

    def test_pick_place(self, cell, slot=0, progress=None):
        """ใช้ตอนสอบเทียบ: หยิบจากจุดหยิบที่ slot+1 ไปวางช่อง cell หนึ่งรอบ (ไม่นับเป็นตาเดินในเกม)"""
        missing = self.missing_config()
        if missing:
            raise MG400Error("ต้องบันทึกค่าให้ครบก่อนทดสอบ ยังขาด: " + ", ".join(missing))
        point = self._point("pick_xyz", slot, default=())
        if not point:
            raise MG400Error(f"จุดหยิบที่ {slot + 1} ยังไม่ได้บันทึก")
        self.check_ready()
        self._pick_and_place(cell, progress or (lambda m: None), pick=point, pick_no=slot + 1)

    def jog(self, dx, dy, dz):
        """ขยับจากตำแหน่งปัจจุบันไปทีละนิด (ใช้ตอนสอบเทียบ)"""
        x, y, z, _ = self.get_pose()
        self.calib_move(x + dx, y + dy, z + dz)
        return self.get_pose()

    def _pick_and_place(self, cell, say, pick=None, pick_no=1):
        """หยิบ → วาง → จอด · ทุกจุดลง-ขึ้นตรง ๆ จากความสูง Approach เดียวกัน
        วิ่งต่อเนื่องเป็น 3 ช่วง หยุดรอเฉพาะตอนจับ ตอนปล่อย และถึงจุดจอด:
          ① เหนือจุดหยิบ → ลง [จับ]  ② ขึ้น → เหนือช่อง → ลง [ปล่อย]  ③ ขึ้น → จุดจอด"""
        px, py, pz = pick if pick else self._point("pick_xyz", 0)
        cx, cy, cz = self._point("cell_xyz", cell)
        az = self.cfg.get("approach_z")
        az_p = float(az) if az is not None else pz + 40   # ยังไม่ตั้ง (ใช้ได้แค่ dry run): ยกเหนือจุด 40 มม.
        az_c = float(az) if az is not None else cz + 40
        say(f"ไปหยิบหมากที่จุดหยิบ {pick_no} (เหนือจุดที่ Z {az_p:g} แล้วลงตรงดิ่ง)")
        self.move_path([(px, py, az_p, "J"), (px, py, pz)], stops=(0,))
        say("หยิบหมาก")
        self.gripper(True)
        say(f"ยกขึ้นตรงดิ่งถึง Z Approach แล้วไปวางช่อง {cell + 1}")
        # ยกขึ้นตรงดิ่งจนถึง Z Approach ก่อน (ไม่ลากหมากเฉียงออกจากถาด) → ข้ามไปเหนือช่องตรงเป๊ะ → ลงตรงดิ่ง
        self.move_path([(px, py, az_p), (cx, cy, az_c, "J"), (cx, cy, cz)], stops=(0, 1))
        say("ปล่อยหมาก")
        self.gripper(False)
        park = self.cfg.get("park_xyz")
        if park:
            say("กลับจุดจอดนอกมุมกล้อง (ตรงไปเลย)")
            self.move_to(*park)
        else:
            say("ยกขึ้นที่ความสูง Approach (ยังไม่ได้ตั้งจุดจอด)")
            self.move_to(cx, cy, az_c)

    def go_park(self, progress=None):
        """ไปจุดจอดนอกมุมกล้องตรง ๆ จากที่ไหนก็ได้ (ตอนเริ่ม/จบเกม)
        ข้ามไป (คืน False) ถ้ายังไม่ได้ตั้งจุดจอด ยังไม่เชื่อมต่อ หรือยังไม่ Enable"""
        park = self.cfg.get("park_xyz")
        if not park or not self.is_connected:
            return False
        if not self.dry_run and self.robot_mode() != MODE_READY:
            return False
        (progress or (lambda m: None))("ไปจุดจอดนอกมุมกล้อง")
        self.move_to(*park)
        return True
