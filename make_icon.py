"""make_icon.py — สร้างไอคอนโปรแกรม (app_icon.ico) รูปกระดาน XO สีเดียวกับในแอป"""
from PIL import Image, ImageDraw

NAVY, LINE, X_COLOR, O_COLOR = "#0F172A", "#94A3B8", "#EA580C", "#059669"


def draw(size=256):
    s = size
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([0, 0, s - 1, s - 1], radius=int(s * 0.2), fill=NAVY)
    m, w = s * 0.17, max(2, int(s * 0.035))            # ขอบในตาราง · ความหนาเส้นตาราง
    cell = (s - 2 * m) / 3
    for k in (1, 2):                                     # เส้นตาราง 3×3
        p = m + k * cell
        d.line([(p, m), (p, s - m)], fill=LINE, width=w)
        d.line([(m, p), (s - m, p)], fill=LINE, width=w)

    def x_mark(r, c):
        pad, t = cell * 0.2, max(2, int(s * 0.055))
        x0, y0 = m + c * cell + pad, m + r * cell + pad
        x1, y1 = m + (c + 1) * cell - pad, m + (r + 1) * cell - pad
        d.line([(x0, y0), (x1, y1)], fill=X_COLOR, width=t)
        d.line([(x0, y1), (x1, y0)], fill=X_COLOR, width=t)

    def o_mark(r, c):
        pad, t = cell * 0.14, max(2, int(s * 0.04))
        d.ellipse([m + c * cell + pad, m + r * cell + pad,
                   m + (c + 1) * cell - pad, m + (r + 1) * cell - pad], outline=O_COLOR, width=t)

    for r, c in ((0, 0), (1, 1), (2, 2)):                # X เรียงแนวทแยง = ชนะ
        x_mark(r, c)
    for r, c in ((0, 2), (2, 0)):
        o_mark(r, c)
    return img


if __name__ == "__main__":
    big = draw(256)
    big.save("app_icon.ico", sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    big.save("app_icon.png")
    print("สร้าง app_icon.ico และ app_icon.png แล้ว")
