"""test_robot.py — ทดสอบ robot.py กับ "MG400 ปลอม" ที่รันในเครื่อง (ไม่ต้องมีหุ่นยนต์)
รันด้วย: python -m unittest test_robot -v
"""

import os
import re
import socket
import tempfile
import threading
import time
import unittest

from hardware import SimWorld
from robot import (MG400Robot, MG400Error, DEFAULT_CONFIG, parse_reply,
                   save_config, load_config, grid_from_corners, backup_path, out_of_reach,
                   low_points, feed_bytes, feed_packet)


class FakeMG400:
    """เซิร์ฟเวอร์จำลองที่ตอบกลับรูปแบบเดียวกับ MG400: 'ErrorID,{ค่า},คำสั่ง;'
    style="bare"  = ตอบแค่ค่า เช่น '4' (พบในบางเฟิร์มแวร์)
    style="split" = ส่งคำตอบเป็น 2 ท่อน (TCP แบ่งข้อมูล)
    stream_move=True = พอร์ต Move ส่งข้อมูลสถานะไบนารีออกมาเองตลอด และรับ MovJ ทาง Dashboard
                       (แบบเดียวกับ MG400 ที่ใช้จริงในโปรเจกต์นี้)"""

    def __init__(self, style="full", stream_move=False):
        self.style = style
        self.stream_move = stream_move
        self.estop = False     # True = กด E-Stop ค้าง: สั่ง Enable แล้วโหมดไม่เปลี่ยน
        self.mode = 4
        self.pose = [300.0, 0.0, 50.0, 0.0]
        self.commands = []
        self.mute = 0            # ไม่ตอบคำสั่ง GetPose อีกกี่ครั้ง (จำลองเครื่องตอบช้า/ไม่ตอบ)
        self.close_move = False  # True = เครื่องปิดพอร์ต Move เอง
        self.mute_sync = False   # True = ไม่ตอบ Sync() (จำลองแขนวิ่งไม่เสร็จ / ค้าง)
        self.feed = False        # True = พอร์ต Move ส่งแพ็กเก็ตสถานะจริง (มีตำแหน่ง/โหมด)
        self.stream_replies = True   # False = พอร์ตที่ส่งสถานะตลอด ไม่ตอบ MovL/Sync เป็นข้อความ (เครื่องในแล็บ)
        self.dash = self._listen(self._dash_reply)
        self.move = self._listen(self._move_reply, stream=stream_move)

    def _listen(self, reply, stream=False):
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)

        def serve():
            conn, _ = srv.accept()
            with conn:
                if stream:   # ส่งแพ็กเก็ตสถานะไบนารีไม่หยุด
                    if self.stream_move == "accepts":   # เฟิร์มแวร์รุ่นเก่า: รับ MovJ ทางพอร์ตนี้ด้วย
                        def read_cmds():
                            try:
                                while True:
                                    data = conn.recv(1024)
                                    if not data:
                                        return
                                    self.commands.append(data.decode())
                                    reply = self._move_reply(data.decode())
                                    # คำตอบข้อความปนมากับข้อมูลสถานะไบนารี
                                    if self.stream_replies and not (self.mute_sync and data.startswith(b"Sync")):
                                        conn.sendall(reply.encode())
                            except OSError:
                                return
                        threading.Thread(target=read_cmds, daemon=True).start()
                    try:
                        while not self.close_move:
                            if self.feed:     # แพ็กเก็ตสถานะ 1440 ไบต์แบบเครื่องจริง (มีตำแหน่ง/โหมด)
                                conn.sendall(feed_bytes(self.mode, (list(self.pose) + [0] * 4)[:4]))
                            else:
                                conn.sendall(b"\x00" * 64 + b"@\x18\xab" + b"\x00" * 61)
                            time.sleep(0.01)
                        conn.shutdown(socket.SHUT_RDWR)   # เครื่องปิดพอร์ตเอง
                    except OSError:
                        pass
                    return
                while True:
                    data = conn.recv(1024)
                    if not data:
                        return
                    cmd = data.decode()
                    self.commands.append(cmd)
                    if self.mute and cmd.startswith("GetPose"):
                        self.mute -= 1
                        continue
                    if self.mute_sync and cmd.startswith("Sync"):
                        continue
                    text = reply(cmd)
                    if self.style == "bare":   # ค่าเปล่า หรือสะท้อนชื่อคำสั่งถ้าไม่มีค่า (เหมือนเครื่องจริง)
                        text = re.search(r"\{(.*?)\}", text).group(1) or cmd
                    if self.style == "split":
                        conn.sendall(text[:3].encode())
                        time.sleep(0.05)
                        text = text[3:]
                    if self.style == "echo":   # สะท้อนชื่อคำสั่งก่อน แล้วค่อยส่งค่า (ถ้ามี) ตามมาทีหลัง
                        value = re.search(r"\{(.*?)\}", text).group(1)
                        conn.sendall(cmd.strip().encode())
                        if not value:
                            continue
                        time.sleep(0.08)
                        text = "{" + value + "}"
                    conn.sendall(text.encode())
        threading.Thread(target=serve, daemon=True).start()
        return srv.getsockname()[1]

    def _dash_reply(self, cmd):
        if "\n" in cmd:     # เหมือนเครื่องจริง: ต่อ \n ท้ายคำสั่งแล้วอ่านคำสั่งพัง ไม่ได้ค่า ไม่ทำอะไร
            return f"0,{{}},{cmd.strip()};"
        if cmd.startswith("RobotMode"):
            return f"0,{{{self.mode}}},{cmd};"
        if cmd.startswith("GetPose"):
            return "0,{%s,0,0},%s;" % (",".join(str(v) for v in self.pose), cmd)
        if cmd.startswith("EnableRobot") and not self.estop:
            self.mode = 5
        if cmd.startswith("DisableRobot"):
            self.mode = 4
        if cmd.startswith(("MovJ", "MovL")) and self.stream_move is True:
            return self._move_reply(cmd)
        return f"0,{{}},{cmd};"

    def _move_reply(self, cmd):
        if cmd.startswith("Sync"):            # เครื่องจริงตอบเมื่อวิ่งเสร็จ (ตัวจำลองวิ่งเสร็จทันที)
            self.syncs = getattr(self, "syncs", 0) + 1
            return f"0,{{}},{cmd.strip()};"
        nums = [float(v) for v in re.findall(r"-?\d+\.?\d*", cmd)]
        self.pose = nums[:4]
        return f"0,{{}},{cmd.strip()};"


def make_config(fake, calibrated=True):
    cfg = dict(DEFAULT_CONFIG, ip="127.0.0.1",
               dashboard_port=fake.dash, move_port=fake.move, move_timeout_s=2,
               gripper_wait_s=0, suck_pulse_s=0.01)
    if calibrated:
        cfg.update(cell_xyz=[[250 + 35 * (i % 3), 100 - 35 * (i // 3), 12] for i in range(9)],
                   pick_xyz=[[350, -100, 10]],
                   park_xyz=[200, -200, 80], approach_z=60)      # ทุกจุดลอยที่ Z 60 ก่อนลง
    return cfg


class TestMG400(unittest.TestCase):
    def test_connect_reads_status(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), log=lambda m: None)
        st = robot.connect()
        self.assertTrue(robot.is_connected)
        self.assertEqual(st["mode"], 4)
        self.assertEqual(st["pose"], [300.0, 0.0, 50.0, 0.0])
        self.assertFalse(any("Enable" in c or "Mov" in c for c in fake.commands))  # เชื่อมต่อแล้วต้องไม่ขยับ

    def test_reply_styles(self):
        """อ่านสถานะได้ทั้งคำตอบเต็ม คำตอบค่าเปล่า ('4') และคำตอบที่มาเป็นหลายท่อน"""
        for style in ("full", "bare", "split", "echo"):
            fake = FakeMG400(style)
            robot = MG400Robot(make_config(fake), log=lambda m: None)
            st = robot.connect()
            self.assertEqual(st["mode"], 4, style)
            self.assertEqual(st["pose"], [300.0, 0.0, 50.0, 0.0], style)
            robot.disconnect()

    def test_enable_with_echo_replies(self):
        """เฟิร์มแวร์ที่ตอบ 'ClearError()' กลับมา: Enable ต้องสำเร็จ และยืนยันด้วยโหมด 5"""
        fake = FakeMG400("bare")
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        self.assertEqual(robot.robot_mode(), 5)
        robot.place_piece(4)                       # เคลื่อนที่ได้ครบรอบกับคำตอบรูปแบบนี้
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])

    def test_dashboard_channel_for_v4_firmware(self):
        """เฟิร์มแวร์ที่รับ MovJ ทาง Dashboard เท่านั้น: ตั้ง move_channel = dashboard"""
        fake = FakeMG400("bare", stream_move=True)
        robot = MG400Robot(dict(make_config(fake), move_channel="dashboard"), dry_run=False,
                           log=lambda m: None)
        robot.connect()
        self.assertEqual(robot.move_channel_name, "Dashboard")
        robot.enable()
        robot.place_piece(4)
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])   # จบที่จุดจอด
        robot.disconnect()

    def test_move_port_with_status_stream(self):
        """พอร์ต 30003 ส่งข้อมูลสถานะไบนารีปนมา: ยังหาคำตอบ MovL/Sync เจอ และหยิบ-วางได้ครบ"""
        fake = FakeMG400("bare", stream_move="accepts")
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        robot.place_piece(4)
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])
        self.assertEqual(fake.syncs, 6)
        robot.disconnect()

    def test_lab_robot_streams_and_never_replies(self):
        """แบบ MG400 ในแล็บ: พอร์ต 30003 ส่งสถานะตลอด ไม่ตอบ MovL/Sync เป็นข้อความ
        ต้องหยิบ-วางได้ครบ โดยดูตำแหน่งจากข้อมูลสถานะ ไม่ถาม GetPose/RobotMode ระหว่างวิ่ง"""
        fake = FakeMG400("bare", stream_move="accepts")
        fake.feed, fake.stream_replies = True, False
        said = []
        robot = MG400Robot(make_config(fake), dry_run=False, log=said.append)
        robot.connect()
        self.assertTrue(robot.move_streaming and robot.feed_ok)
        self.assertTrue(any("ดูตำแหน่งแขนจากข้อมูลสถานะ" in m for m in said))
        robot.enable()
        n0 = len(fake.commands)
        t0 = time.time()
        robot.place_piece(4)
        self.assertLess(time.time() - t0, 5)
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])
        cmds = fake.commands[n0:]
        first = next(i for i, c in enumerate(cmds) if c.startswith(("MovL", "MovJ")))
        self.assertEqual([c for c in cmds[first:] if c.startswith(("GetPose", "RobotMode"))], [])
        robot.disconnect()

    def test_feed_packet_parsing(self):
        pkt = feed_bytes(5, [301.5, -20.25, -44.0, 0.0])
        self.assertEqual(feed_packet(pkt), (5, [301.5, -20.25, -44.0, 0.0]))
        self.assertIsNone(feed_packet(b"\x00" * 1440))

    def test_sync_timeout_message(self):
        """Sync() ไม่ตอบภายในเวลาที่ตั้ง (แขนค้าง/E-Stop): แจ้งข้อความชัดเจน"""
        fake = FakeMG400()
        fake.mute_sync = True
        robot = MG400Robot(dict(make_config(fake), move_timeout_s=0.5), dry_run=False,
                           log=lambda m: None)
        robot.connect()
        robot.enable()
        with self.assertRaises(MG400Error) as ctx:
            robot.calib_move(300, 0, 80)
        self.assertIn("แขนวิ่งไม่เสร็จภายใน", str(ctx.exception))

    def test_normal_move_port_still_used(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), log=lambda m: None)
        robot.connect()
        self.assertEqual(robot.move_channel_name, "Move")

    def test_binary_reply_message_is_short(self):
        with self.assertRaises(MG400Error) as ctx:
            parse_reply("MovJ(1,2,3,0)", "\x00" * 2000)
        self.assertLess(len(str(ctx.exception)), 400)

    def test_enable_fails_when_mode_stays_disabled(self):
        fake = FakeMG400("bare")
        fake.estop = True
        robot = MG400Robot(make_config(fake), log=lambda m: None)
        robot.connect()
        with self.assertRaises(MG400Error):
            robot.enable(timeout_s=0.5)

    def test_echo_then_value_full_game_move(self):
        """เครื่องสะท้อนชื่อคำสั่งก่อนแล้วส่งค่าตามมา (แบบที่เจอหลังใส่ \\n): ต้องไม่เลื่อนคำตอบผิดคำสั่ง"""
        fake = FakeMG400("echo")
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        st = robot.connect()
        self.assertEqual(st["mode"], 4)
        self.assertEqual(st["pose"], [300.0, 0.0, 50.0, 0.0])
        robot.enable()
        for _ in range(3):                       # อ่านซ้ำหลายรอบ คำตอบต้องไม่เลื่อน
            self.assertEqual(robot.robot_mode(), 5)
            self.assertEqual(robot.get_pose()[:3], [300.0, 0.0, 50.0])
        robot.place_piece(4)
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])
        robot.disconnect()

    def test_parse_reply(self):
        self.assertEqual(parse_reply("GetPose()", "GetPose(){1.5,2,3,4,0,0}"), "1.5,2,3,4,0,0")
        self.assertEqual(parse_reply("RobotMode()", "RobotMode()\n5"), "5")
        self.assertEqual(parse_reply("GetPose()", "0,{1,2,3,4},GetPose();"), "1,2,3,4")
        self.assertEqual(parse_reply("ClearError()", "ClearError()"), "")
        self.assertEqual(parse_reply("MovJ(1,2,3,0)", "MovJ(1,2,3,0);"), "")
        self.assertEqual(parse_reply("RobotMode()", "0,{5},RobotMode();"), "5")
        self.assertEqual(parse_reply("RobotMode()", "4"), "4")
        self.assertEqual(parse_reply("RobotMode()", "{4}"), "4")
        with self.assertRaises(MG400Error):
            parse_reply("EnableRobot()", "-1,{},EnableRobot();")   # รหัสผิดพลาดจริง
        with self.assertRaises(MG400Error):
            parse_reply("RobotMode()", "")

    def test_connect_fail(self):
        cfg = dict(DEFAULT_CONFIG, ip="127.0.0.1", dashboard_port=1, move_port=1)
        with self.assertRaises(MG400Error):
            MG400Robot(cfg).connect()

    def test_dry_run_sends_no_motion(self):
        fake = FakeMG400()
        world = SimWorld()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=True,
                           sim_world=world, log=lambda m: None)
        robot.connect()
        robot.place_piece(4)
        self.assertFalse(any(c.startswith(("MovL", "MovJ", "DO")) for c in fake.commands))
        self.assertEqual(world.board[4], "O")

    def test_real_move_requires_enable(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False)
        robot.connect()
        with self.assertRaises(MG400Error):
            robot.place_piece(0)

    def test_real_move_requires_calibration(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False)
        robot.connect()
        robot.enable()
        with self.assertRaises(MG400Error):
            robot.place_piece(0)

    def test_full_place_sequence(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False)
        robot.connect()
        robot.enable()
        robot.place_piece(2)
        moves = [c for c in fake.commands if c.startswith(("MovL", "MovJ"))]
        self.assertEqual(len(moves), 6)    # จับ 2 · ไปวาง 3 · ตรงไปจุดจอด 1
        self.assertEqual(moves[3], "MovJ(320.00,100.00,60.00,0.00)")   # ข้ามไปเหนือช่อง 3 ด้วย MovJ
        # ดูด: ปิด D2 → เปิด D1 · ปล่อย: ปิด D1 ก่อน → เปิด D2 พ่นออก → ปิด D2
        self.assertEqual([c for c in fake.commands if c.startswith("DO")],
                         ["DO(2,0)", "DO(1,1)", "DO(1,0)", "DO(1,0)", "DO(2,1)", "DO(2,0)"])
        self.assertEqual(fake.pose[:3], [200.0, -200.0, 80.0])          # จบที่จุดจอด

    def test_newline_terminator_breaks_dashboard_reads(self):
        """ยืนยันพฤติกรรมเครื่องจริง: ตั้ง cmd_terminator เป็น \\n แล้วอ่าน GetPose ไม่ได้"""
        fake = FakeMG400("bare")
        robot = MG400Robot(dict(make_config(fake), cmd_terminator="\n"), log=lambda m: None)
        with self.assertRaises((MG400Error, ValueError)):
            robot.connect()

    def test_calib_move_requires_enable(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False)
        robot.connect()
        with self.assertRaises(MG400Error):
            robot.calib_move(300, 0, 60)
        robot.enable()
        robot.calib_move(300, 0, 60)
        self.assertEqual(fake.pose[:3], [300.0, 0.0, 60.0])

    def test_jog_moves_relative(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False)
        robot.connect()
        robot.enable()
        robot.jog(5, 0, -10)
        self.assertEqual(fake.pose[:3], [305.0, 0.0, 40.0])

    def test_pick_place_test_does_not_count_as_move(self):
        fake = FakeMG400()
        world = SimWorld()
        robot = MG400Robot(make_config(fake), dry_run=False, sim_world=world)
        robot.connect()
        robot.enable()
        robot.test_pick_place(cell=4)
        self.assertEqual(len([c for c in fake.commands if c.startswith(("MovL", "MovJ"))]), 6)
        self.assertEqual(world.board.count("O"), 0)   # ไม่ไปแก้กระดานในเกม
        self.assertEqual(robot.slot, 0)

    def test_always_picks_from_same_point(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False)
        robot.connect()
        robot.enable()
        for cell in range(6):      # หยิบได้เกิน 5 ชิ้น เพราะคนเติมหมากที่จุดเดิม
            robot.place_piece(cell)
        picks = [c for c in fake.commands if c.startswith("MovL(350.00,-100.00,10.00")]
        self.assertEqual(len(picks), 6)

    def test_five_pick_points_used_in_order(self):
        """มีจุดหยิบ 5 จุด: หุ่นยนต์หยิบเรียงจุดที่ 1 → 5 (ข้ามจุดที่ยังว่าง)"""
        fake = FakeMG400()
        picks = [[400, -100 + 30 * k, 10] for k in range(5)]
        picks[2] = None                                   # จุดที่ 3 ยังไม่ได้สอน
        robot = MG400Robot(dict(make_config(fake), pick_xyz=picks), dry_run=False,
                           log=lambda m: None)
        robot.connect()
        robot.enable()
        for cell in range(4):
            robot.place_piece(cell)
        downs = [c for c in fake.commands if c.startswith("MovL(400.00") and ",10.00," in c]
        self.assertEqual([c.split(",")[1] for c in downs], ["-100.00", "-70.00", "-10.00", "20.00"])

    def test_visit_points_above_down_up(self):
        """Move ทดสอบจุด: แต่ละจุดต้อง ลอยที่ Z Approach → ลงที่จุด → ยกกลับ ตามลำดับ"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        ok = robot.visit_points([("ช่อง 1", [300, 0, -20]), ("ช่อง 2", [300, 40, -30])], 20,
                                dwell_s=0)
        self.assertTrue(ok)
        moves = [c for c in fake.commands if c.startswith(("MovL", "MovJ"))]
        self.assertEqual(moves, ["MovJ(300.00,0.00,20.00,0.00)", "MovL(300.00,0.00,-20.00,0.00)",
                                 "MovL(300.00,0.00,20.00,0.00)", "MovJ(300.00,40.00,20.00,0.00)",
                                 "MovL(300.00,40.00,-30.00,0.00)", "MovL(300.00,40.00,20.00,0.00)"])

    @staticmethod
    def segments(commands):
        """แบ่งคำสั่งเคลื่อนที่เป็นช่วงที่ส่งติดกัน (คั่นด้วยการอ่านตำแหน่ง = รอถึงจุดก่อน)"""
        out, cur = [], []
        for c in commands:
            if c.startswith(("MovL", "MovJ")):
                cur.append(c)
            elif c.startswith(("GetPose", "Sync")) and cur:
                out.append(cur)
                cur = []
        return out + ([cur] if cur else [])

    def test_visit_points_continuous(self):
        """Move ทดสอบ: ถึงเหนือจุดและจุดจริงตรงเป๊ะ (ลงตรงดิ่ง) · ยกขึ้นแล้วไหลไปเหนือจุดถัดไปไม่หยุด"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        n0 = len(fake.commands)
        said = []
        robot.visit_points([("ช่อง 1", [300, 0, -20]), ("ช่อง 2", [300, 40, -30])], 20,
                           progress=said.append)
        self.assertEqual(self.segments(fake.commands[n0:]), [
            ["MovJ(300.00,0.00,20.00,0.00)"],                                     # ข้ามไปเหนือช่อง 1
            ["MovL(300.00,0.00,-20.00,0.00)"],                                    # ลงตรงดิ่ง
            ["MovL(300.00,0.00,20.00,0.00)"],                                     # ยกขึ้นตรงดิ่ง
            ["MovJ(300.00,40.00,20.00,0.00)"],                                    # ข้ามไปเหนือช่อง 2
            ["MovL(300.00,40.00,-30.00,0.00)"],
            ["MovL(300.00,40.00,20.00,0.00)"]])
        self.assertEqual(said, ["ช่อง 1 (1/2)", "ช่อง 2 (2/2)"])

    def test_every_pick_point_goes_above_then_straight_down(self):
        """จุดหยิบทุกจุด (1–5): ไปถึง X,Y เดิมที่ Z Approach ก่อน แล้วจึงลงตรงดิ่งที่ Z ของจุดนั้น
        และช่องวางก็เหมือนกัน"""
        fake = FakeMG400()
        picks = [[400, -100 + 30 * k, 10 + k] for k in range(5)]
        robot = MG400Robot(dict(make_config(fake), pick_xyz=picks), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        for k in range(5):
            n0 = len(fake.commands)
            robot.place_piece(k)
            segs = self.segments(fake.commands[n0:])
            x, y, z = picks[k]
            cx, cy, cz = make_config(fake)["cell_xyz"][k]
            above = f"MovL({x:.2f},{y:.2f},60.00,0.00)"
            self.assertEqual(segs[0], [above.replace("MovL", "MovJ")])                # ข้ามไปเหนือจุดหยิบ
            self.assertEqual(segs[1], [f"MovL({x:.2f},{y:.2f},{z:.2f},0.00)"])        # ลงตรงดิ่ง
            self.assertEqual(segs[2], [above])                                        # ยกขึ้นตรงดิ่งก่อน
            self.assertEqual(segs[3], [f"MovJ({cx:.2f},{cy:.2f},60.00,0.00)"])        # ข้ามไปเหนือช่อง
            self.assertEqual(segs[4], [f"MovL({cx:.2f},{cy:.2f},{cz:.2f},0.00)"])     # ลงตรงดิ่ง

    def test_place_uses_one_approach_height(self):
        """ทุกจุดลอยที่ Z Approach เดียวกัน แม้จุดหยิบกับช่องจะสูงไม่เท่ากัน"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        robot.place_piece(0)                       # จุดหยิบ (350, -100, 10) → ช่อง 1 (250, 100, 12)
        moves = [c for c in fake.commands if c.startswith(("MovL", "MovJ"))]
        self.assertEqual(moves, [
            "MovJ(350.00,-100.00,60.00,0.00)", "MovL(350.00,-100.00,10.00,0.00)",
            "MovL(350.00,-100.00,60.00,0.00)",
            "MovJ(250.00,100.00,60.00,0.00)", "MovL(250.00,100.00,12.00,0.00)",
            "MovL(200.00,-200.00,80.00,0.00)"])                  # ปล่อยแล้วตรงไปจุดจอดเลย                                     # จุดจอด

    def test_safe_goto_lifts_before_crossing(self):
        """ไปจุดอื่นตอนสอบเทียบ: ยกขึ้น Z Approach ก่อน → ข้ามไป → ลง"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        fake.pose = [300.0, 0.0, -20.0, 0.0]
        robot.safe_goto(250, 50, -25, 30)
        moves = [c for c in fake.commands if c.startswith(("MovL", "MovJ"))]
        self.assertEqual(moves, ["MovL(300.00,0.00,30.00,0.00)", "MovJ(250.00,50.00,30.00,0.00)",
                                 "MovL(250.00,50.00,-25.00,0.00)"])

    def test_bare_replies_read_without_waiting(self):
        """เครื่องที่ตอบค่าเปล่า ('5', '{x,y,…}', 'DO(1,1)'): อ่านได้ทันที ไม่ต้องรอเงียบ 0.15 วิ ทุกคำสั่ง"""
        for style in ("bare", "full"):
            fake = FakeMG400(style)
            robot = MG400Robot(make_config(fake), log=lambda m: None)
            robot.connect()
            t0 = time.time()
            for _ in range(5):
                robot.get_pose()
                robot.robot_mode()
                robot._send(robot.dash, "DO(1,1)")
            self.assertLess(time.time() - t0, 0.5, style)     # เดิม ~2.3 วิ (15 คำสั่ง × 0.15)
            robot.disconnect()

    def test_go_park_goes_straight(self):
        """ไปจุดจอดตรง ๆ คำสั่งเดียว จากที่ไหนก็ได้ · ยังไม่ Enable = ข้าม ไม่ขยับ"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        robot.connect()
        self.assertFalse(robot.go_park())                     # ยังไม่ Enable
        self.assertFalse(any(c.startswith("Mov") for c in fake.commands))
        robot.enable()
        fake.pose = [330.0, 40.0, 12.0, 0.0]                  # ค้างอยู่บนกระดาน
        self.assertTrue(robot.go_park())
        self.assertEqual([c for c in fake.commands if c.startswith("Mov")],
                         ["MovL(200.00,-200.00,80.00,0.00)"])

    def test_open_retries_until_robot_releases_port(self):
        """เครื่องยังไม่ปล่อยการเชื่อมต่อเก่า (ถูกปฏิเสธ) → ลองใหม่อัตโนมัติจนต่อได้ ไม่ยอมแพ้ใน 3 วิ"""
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()                                   # ตอนนี้พอร์ตนี้ปฏิเสธการเชื่อมต่อ
        srv = socket.socket()

        def open_later():
            time.sleep(3.5)                             # เครื่องปล่อยพอร์ตหลัง ~3.5 วิ
            srv.bind(("127.0.0.1", port))
            srv.listen(1)
        threading.Thread(target=open_later, daemon=True).start()
        said = []
        sock = MG400Robot._open("127.0.0.1", port, "Dashboard", log=said.append)
        self.assertTrue(said)                           # บอกผู้ใช้ว่ากำลังลองใหม่
        sock.close()
        srv.close()

    def test_slow_reply_is_asked_again(self):
        """เครื่องไม่ตอบ GetPose ครั้งหนึ่ง: ถามใหม่อัตโนมัติ ไม่ทำให้หลุด"""
        fake = FakeMG400("bare")
        said = []
        robot = MG400Robot(make_config(fake), log=said.append)
        robot.connect()
        fake.mute = 1
        self.assertEqual(robot.get_pose(), [300.0, 0.0, 50.0, 0.0])
        self.assertTrue(any("ถามใหม่" in m for m in said))
        fake.mute = 2                                  # ไม่ตอบ 2 ครั้งติด: แจ้งเป็นข้อความชัดเจน
        with self.assertRaises(MG400Error) as err:
            robot.get_pose()
        self.assertIn("ไม่ตอบคำสั่ง GetPose()", str(err.exception))
        robot.disconnect()

    def test_waits_with_sync_like_reference(self):
        """พอร์ต Move ตอบเป็นข้อความ: ส่ง MovL แล้ว Sync() (แบบ mg400_vision.py) ไม่ถามตำแหน่งวนซ้ำ"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        n0 = len(fake.commands)
        robot.place_piece(4)
        cmds = fake.commands[n0:]
        moves = [c for c in cmds if c.startswith(("MovL", "MovJ"))]
        self.assertEqual(len(moves), 6)
        self.assertEqual(fake.syncs, 6)                              # รอวิ่งเสร็จทีละช่วง ด้วย Sync()
        for i, c in enumerate(cmds):                                 # ทุก MovL ตามด้วย Sync() ก่อนท่าถัดไป
            if c.startswith(("MovL", "MovJ")):
                self.assertTrue(cmds[i + 1].startswith(("Sync", "MovL", "MovJ")))
        first_move = next(i for i, c in enumerate(cmds) if c.startswith(("MovL", "MovJ")))
        self.assertEqual([c for c in cmds[first_move:] if c.startswith(("GetPose", "RobotMode"))], [])   # ระหว่างวิ่งไม่ถามเครื่องเลย

    def test_move_type_option(self):
        """ค่าเริ่มต้นเคลื่อนที่แบบ MovL (เส้นตรง) · ตั้ง move_type = MovJ ได้"""
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake), log=lambda m: None)
        self.assertEqual(robot.movj_command(300, 0, 20), "MovL(300.00,0.00,20.00,0.00)")
        robot.cfg["move_type"] = "MovJ"
        self.assertEqual(robot.movj_command(300, 0, 20), "MovJ(300.00,0.00,20.00,0.00)")

    def test_low_points_warning(self):
        cfg = dict(DEFAULT_CONFIG, approach_z=0, cell_xyz=[[300, 0, -35]] * 8 + [[300, 0, -10]],
                   pick_xyz=[[300, 100, -35]])
        self.assertEqual(low_points(cfg, 20), ["ช่อง 9"])

    def test_visit_points_can_stop(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=False, log=lambda m: None)
        robot.connect()
        robot.enable()
        stop = threading.Event()
        stop.set()
        self.assertFalse(robot.visit_points([("ช่อง 1", [300, 0, -20])], 20, stop=stop))
        self.assertFalse(any(c.startswith("MovL") for c in fake.commands))

    def test_pick_place_test_requires_full_config(self):
        fake = FakeMG400()
        robot = MG400Robot(make_config(fake, calibrated=False), dry_run=True)
        robot.connect()
        with self.assertRaises(MG400Error):
            robot.test_pick_place(cell=0)


class TestConfigTools(unittest.TestCase):
    def test_grid_from_corners(self):
        grid = grid_from_corners([300, -40], [300, 40], [220, -40])
        self.assertEqual(grid[0], [300, -40])
        self.assertEqual(grid[4], [260, 0])      # ช่องกลาง
        self.assertEqual(grid[8], [220, 40])     # มุมขวาล่าง

    def test_save_and_load_config(self):
        cfg = dict(DEFAULT_CONFIG, approach_z=60, cell_xyz=[[1.5, 2, -3]] * 9)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "robot_config.json")
            save_config(cfg, path)
            self.assertEqual(load_config(path), cfg)
            save_config(dict(cfg, approach_z=80), path)      # บันทึกครั้งที่ 2 ต้องเก็บค่าเดิมไว้
            self.assertEqual(load_config(backup_path(path))["approach_z"], 60)

    def test_out_of_reach(self):
        cfg = dict(DEFAULT_CONFIG, cell_xyz=[[300, 0, 0]] * 8 + [[500, 0, 0]], park_xyz=[0, -450, 50])
        self.assertEqual(out_of_reach(cfg), ["ช่อง 9", "จุดจอด"])

    def test_old_config_migrates_to_xyz_points(self):
        """ไฟล์รุ่นเก่า (X,Y + ความสูงรวม) ต้องแปลงเป็นจุดละ X,Y,Z อัตโนมัติ"""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "robot_config.json")
            with open(path, "w", encoding="utf-8") as f:
                f.write('{"cell_xy": [[1, 2], [3, 4]], "z_place": -35, "z_safe": 5, '
                        '"pick_xy": [7, 8], "z_pick": -30}')
            cfg = load_config(path)
            self.assertEqual(cfg["cell_xyz"], [[1, 2, -35], [3, 4, -35]])
            self.assertEqual(cfg["pick_xyz"], [[7, 8, -30]])
            self.assertEqual(cfg["approach_z"], 5)          # z_safe เดิม = ความสูง Approach
            for old in ("cell_xy", "pick_xy", "z_safe", "z_pick", "z_place"):
                self.assertNotIn(old, cfg)

    def test_approach_dz_config_migrates_to_one_z(self):
        """ไฟล์ที่ใช้ approach_dz (ยกเหนือแต่ละจุด) → Z Approach = จุดที่สูงสุด + approach_dz"""
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "robot_config.json")
            with open(path, "w", encoding="utf-8") as f:
                f.write('{"cell_xyz": [[1, 2, -35], [3, 4, -30]], "pick_xyz": [[7, 8, -40], null], '
                        '"approach_dz": 40}')
            cfg = load_config(path)
            self.assertEqual(cfg["approach_z"], 10)
            self.assertNotIn("approach_dz", cfg)


if __name__ == "__main__":
    unittest.main()
