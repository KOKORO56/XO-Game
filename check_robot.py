"""check_robot.py — ทดสอบการเชื่อมต่อ MG400 แบบ "อ่านอย่างเดียว"

รัน:  python check_robot.py 192.168.1.6            อ่านสถานะอย่างเดียว
      python check_robot.py 192.168.1.6 --enable   ทดสอบ Enable ด้วย (ถามก่อน ไม่สั่งเคลื่อนที่)

ถ้าไม่ใส่ --enable สคริปต์นี้ไม่สั่ง Enable และไม่สั่งเคลื่อนที่ใด ๆ ทำแค่:
  1) เปิดการเชื่อมต่อ TCP ไปยังพอร์ต Dashboard
  2) ถามโหมดหุ่นยนต์ (RobotMode) และตำแหน่งปัจจุบัน (GetPose)

ก่อนใช้: ตรวจ IP / พอร์ต / ชื่อคำสั่ง กับเอกสาร TCP/IP Protocol ของเฟิร์มแวร์ที่ใช้จริง
"""

import socket
import sys
import time

ROBOT_IP = "192.168.1.6"   # ค่าเริ่มต้นที่พบบ่อยของพอร์ต LAN1 — แก้ให้ตรงเครื่องจริง
DASHBOARD_PORT = 29999
TIMEOUT_S = 3


def send(sock, cmd):
    """คืนคำตอบดิบทั้งหมดที่ได้ภายใน 1 วินาที (แสดงแบบ repr เพื่อเห็นทุกตัวอักษร)"""
    sock.sendall((cmd + "\n").encode("utf-8"))      # ต้องลงท้าย \n เหมือนโปรแกรมหลัก
    data = sock.recv(1024)
    sock.settimeout(1)
    try:
        while True:                                  # อ่านทุกท่อนจนเครื่องเงียบ 1 วินาที
            more = sock.recv(1024)
            if not more:
                break
            data += more
    except socket.timeout:
        pass
    finally:
        sock.settimeout(TIMEOUT_S)
    return repr(data.decode("utf-8", errors="replace"))


def check_enable(s):
    """ทดสอบ Enable: เปิดมอเตอร์อย่างเดียว ไม่สั่งเคลื่อนที่ (แขนจะค้างอยู่ที่เดิม)"""
    input("\nจะสั่ง ClearError() + EnableRobot() เพื่อเปิดมอเตอร์ (แขนไม่เคลื่อนที่) "
          "ตรวจพื้นที่รอบแขนแล้วกด Enter เพื่อทำต่อ หรือ Ctrl+C เพื่อยกเลิก ...")
    for cmd in ("ClearError()", "EnableRobot()"):
        print(f"{cmd:14s} -> {send(s, cmd)}")
    for i in range(10):          # ดูโหมด 5 วินาที: 5 = พร้อมทำงาน (Enabled)
        time.sleep(0.5)
        print(f"  {i * 0.5 + 0.5:3.1f} วินาที  RobotMode() -> {send(s, 'RobotMode()')}")
    print("\nผลที่ควรได้: RobotMode เปลี่ยนเป็น 5 · ถ้ายังเป็น 4 แปลว่าเครื่องไม่รับคำสั่ง Enable")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    ip = args[0] if args else ROBOT_IP
    print(f"กำลังเชื่อมต่อ {ip}:{DASHBOARD_PORT} ...")
    try:
        with socket.create_connection((ip, DASHBOARD_PORT), timeout=TIMEOUT_S) as s:
            print("เชื่อมต่อสำเร็จ")
            for cmd in ("RobotMode()", "GetPose()"):
                try:
                    print(f"{cmd:12s} -> {send(s, cmd)}")
                except socket.timeout:
                    print(f"{cmd:12s} -> ไม่มีการตอบกลับ (ตรวจชื่อคำสั่งกับเวอร์ชันเฟิร์มแวร์)")
            if "--enable" in sys.argv:
                check_enable(s)
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"เชื่อมต่อไม่ได้: {e}")
        from robot import dobot_studio_running
        if dobot_studio_running():
            print("สาเหตุ: DobotStudio Pro / DobotLink เปิดอยู่ในเครื่องนี้ · ปิดให้หมดก่อน "
                  "(รวมไอคอน DobotLink ที่มุมขวาล่างของ Taskbar) รอ 30 วินาที แล้วลองใหม่")
            sys.exit(1)
        print("ตรวจ: สาย LAN, IP ของคอมอยู่วงเดียวกัน (เช่น 192.168.1.100), "
              "ปิดโปรแกรมอื่นที่เชื่อมต่อหุ่นยนต์อยู่, และโหมดการควบคุมเป็น TCP/IP")
        sys.exit(1)


if __name__ == "__main__":
    main()
