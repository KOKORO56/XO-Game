"""camera_test.py — หากล้อง เปิดภาพสด และช่วยจัดตำแหน่งกล้องให้ตรงกระดาน

วิธีใช้:
    python camera_test.py            # สแกนหากล้องอัตโนมัติ
    python camera_test.py 1          # ระบุหมายเลขกล้องเอง

ปุ่มลัดขณะเปิดภาพ:
    g = เปิด/ปิดเส้นกริด 3x3
    s = บันทึกภาพลงโฟลเดอร์ images/ (ใช้ตั้งค่าสีในขั้นตอนถัดไป)
    q = ออก
"""

import os
import sys
import time
import platform
import cv2

WIDTH, HEIGHT = 1280, 720


def open_camera(index):
    # บน Windows ใช้ DirectShow จะเปิดกล้องได้เร็วกว่า
    backend = cv2.CAP_DSHOW if platform.system() == "Windows" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    return cap


def find_cameras(max_index=5):
    found = []
    for i in range(max_index):
        cap = open_camera(i)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(i)
        cap.release()
    return found


def draw_guides(frame):
    h, w = frame.shape[:2]
    # สี่เหลี่ยมจัตุรัสกลางภาพ = พื้นที่ที่กระดานควรอยู่
    size = int(min(h, w) * 0.8)
    x0, y0 = (w - size) // 2, (h - size) // 2
    cv2.rectangle(frame, (x0, y0), (x0 + size, y0 + size), (0, 255, 0), 2)
    for k in (1, 2):
        cv2.line(frame, (x0 + k * size // 3, y0), (x0 + k * size // 3, y0 + size), (0, 255, 0), 1)
        cv2.line(frame, (x0, y0 + k * size // 3), (x0 + size, y0 + k * size // 3), (0, 255, 0), 1)
    cv2.drawMarker(frame, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)


def main():
    if len(sys.argv) > 1:
        index = int(sys.argv[1])
    else:
        cams = find_cameras()
        print("พบกล้องหมายเลข:", cams if cams else "ไม่พบเลย")
        if not cams:
            sys.exit("ตรวจสาย USB / สิทธิ์การเข้าถึงกล้อง / ปิดโปรแกรมอื่นที่ใช้กล้องอยู่")
        index = cams[-1]  # กล้อง USB ภายนอกมักเป็นหมายเลขท้ายสุด (0 มักเป็นกล้องโน้ตบุ๊ก)
        print(f"ใช้กล้องหมายเลข {index} (ถ้าผิดตัว ให้รัน: python camera_test.py <หมายเลข>)")

    cap = open_camera(index)
    if not cap.isOpened():
        sys.exit(f"เปิดกล้องหมายเลข {index} ไม่ได้")

    os.makedirs("images", exist_ok=True)
    show_grid = True
    while True:
        ok, frame = cap.read()
        if not ok:
            print("อ่านภาพจากกล้องไม่ได้")
            break
        view = frame.copy()
        if show_grid:
            draw_guides(view)
        h, w = frame.shape[:2]
        cv2.putText(view, f"cam {index}  {w}x{h}   g=grid  s=save  q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("XO Camera", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("g"):
            show_grid = not show_grid
        if key == ord("s"):
            path = os.path.join("images", f"board_{time.strftime('%Y%m%d_%H%M%S')}.png")
            cv2.imwrite(path, frame)  # บันทึกภาพจริง ไม่มีเส้นกริด
            print("บันทึก:", path)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
