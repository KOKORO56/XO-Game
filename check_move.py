"""check_move.py — หารูปแบบคำสั่ง MovJ ที่ MG400 เครื่องนี้รับ (ใช้ตอนขึ้น "เคลื่อนที่ไปไม่ถึง")

รัน:  python check_move.py 192.168.1.6

ทำอะไร:
  ลองส่ง MovJ ทีละรูปแบบ ทีละช่องทาง ให้แขน "ยกขึ้น 10 มม." จากตำแหน่งปัจจุบัน
  แล้วดูว่าแขนขยับจริงไหม (อ่าน GetPose) · ถามก่อนทุกครั้ง กด Ctrl+C ยกเลิกได้ตลอด
ก่อนรัน: ปิดโปรแกรม XO และ DobotStudio Pro · กด Enable ไว้ หรือให้สคริปต์ Enable ให้
         ตรวจว่าเหนือหัวจับว่างอย่างน้อย 30 มม. และเอื้อมถึงปุ่ม E-Stop
"""

import sys
import time

from robot import MG400Robot, MG400Error, MOVJ_FORMATS, MODE_READY, load_config, _wire

LIFT_MM = 10


def moved_to(robot, target, wait_s=4.0):
    deadline = time.time() + wait_s
    pose = robot.get_pose()
    while time.time() < deadline:
        pose = robot.get_pose()
        if all(abs(pose[i] - target[i]) <= 1.5 for i in range(3)):
            return True, pose
        time.sleep(0.1)
    return False, pose


def main():
    cfg = load_config()
    if len(sys.argv) > 1:
        cfg["ip"] = sys.argv[1]
    cfg["move_channel"] = "move_port"     # เปิดพอร์ต Move ไว้ด้วย จะได้ลองทั้งสองช่องทาง
    robot = MG400Robot(cfg, dry_run=False, log=print)
    try:
        robot.connect()
    except MG400Error:
        cfg["move_channel"] = "dashboard"
        robot.connect()
    print("เชื่อมต่อแล้ว · โหมด", robot.robot_mode(), "· ตำแหน่ง", robot.get_pose())

    if robot.robot_mode() != MODE_READY:
        input("หุ่นยนต์ยังไม่ Enable · กด Enter เพื่อ Enable (แขนยังไม่ขยับ) หรือ Ctrl+C ยกเลิก ...")
        robot.enable()
    robot._send(robot.dash, "SpeedFactor(10)")      # ทดสอบที่ความเร็ว 10%

    channels = [("Dashboard 29999", robot.dash, False)]
    if robot.move is not robot.dash:
        # พอร์ต 30003 บางเฟิร์มแวร์ส่งข้อมูลสถานะออกมาตลอด แต่ยังรับคำสั่งได้: ส่งอย่างเดียว ไม่รอคำตอบ
        channels.append(("Move 30003", robot.move, robot.move_streaming))

    tries = [(ch, sock, so, fmt_name, fmt, term)
             for ch, sock, so in channels for fmt_name, fmt in MOVJ_FORMATS.items()
             for term in ("", "\n")]
    for ch_name, sock, send_only, fmt_name, fmt, term in tries:
        if sock is robot.dash and term == "\n":
            continue        # Dashboard ของเครื่องนี้อ่านคำสั่งพังถ้าต่อ \n (ทดสอบแล้ว)
        if True:
            x, y, z, r = robot.get_pose()
            target = (x, y, z + LIFT_MM)
            cmd = fmt.format(cmd=robot.cfg.get("move_type", "MovL"),
                             x=target[0], y=target[1], z=target[2], r=r)
            input(f"\nลอง {ch_name} · รูปแบบ {fmt_name} · ปิดท้าย {term!r}: {cmd}\n"
                  f"แขนจะยกขึ้น {LIFT_MM} มม. · กด Enter เพื่อส่ง ...")
            try:
                sock.sendall(_wire(cmd, term))
                reply = "(ส่งอย่างเดียว พอร์ตนี้ไม่ตอบเป็นข้อความ)" if send_only else robot._recv_reply(sock, cmd)
            except Exception as e:      # แสดงทุกอย่างเพื่อวินิจฉัย
                reply = f"(ส่ง/อ่านไม่สำเร็จ: {e})"
            ok, pose = moved_to(robot, target)
            try:
                robot._send(robot.dash, "GetErrorID()")
                err = robot.last_reply[1][:80]
            except Exception as e:
                err = f"อ่านไม่ได้ ({e})"
            print(f"  คำตอบ: {reply[:80]!r} · GetErrorID: {err!r}")
            print(f"  ผล: {'✅ แขนขยับถึงเป้าหมาย' if ok else '❌ แขนไม่ขยับ'} · ตอนนี้ {[round(v, 1) for v in pose]}")
            if ok:
                channel = "dashboard" if sock is robot.dash else "move_port"
                key = "cmd_terminator" if channel == "dashboard" else "move_terminator"
                term_json = '"\\n"' if term else '""'
                print("\nใช้ได้แล้ว · แก้ robot_config.json ให้เป็น:\n"
                      f'  "move_channel": "{channel}",\n  "movj_format": "{fmt_name}",\n'
                      f'  "{key}": {term_json},')
                robot.disconnect()
                return
    print("\nไม่มีรูปแบบไหนทำให้แขนขยับ · ตรวจว่าเครื่องอยู่โหมด TCP/IP และไม่มี Error/E-Stop ค้าง")
    robot.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nยกเลิกแล้ว")
