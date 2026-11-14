"""Geometry mock of the new assistant screen.

Renders the same coordinates/sizes/colours that main/assistant_ui.c uses, so the
layout can be checked without hardware. This is NOT an LVGL render: it is a
Pillow drawing of the same primitives, used only to verify placement inside the
360x360 round panel (radius 180).
"""
from PIL import Image, ImageDraw, ImageFont

W = H = 360
C_BG = (255, 255, 255)
C_INK = (28, 28, 30)
C_MUTED = (142, 142, 147)
C_ACCENT = (37, 99, 235)
C_TRACK = (217, 217, 222)
C_CHIP = (242, 242, 247)

EYE_CY, EYE_CX_L, EYE_CX_R = 150, 146, 214
MOUTH_CX, MOUTH_CY = 180, 210

FONT_PATH = "fonts_src/NotoSansSC-Regular.otf"


def font(px):
    return ImageFont.truetype(FONT_PATH, px)


def centered(draw, xy, text, f, fill):
    box = draw.textbbox((0, 0), text, font=f)
    draw.text((xy[0] - (box[2] - box[0]) / 2, xy[1] - (box[3] - box[1]) / 2 - box[1]), text, font=f, fill=fill)


def rounded_rect(draw, cx, cy, w, h, color, radius):
    r = min(radius, w // 2, h // 2)
    draw.rounded_rectangle([cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2], radius=r, fill=color)


def ring(draw, cx, cy, d, width, color):
    r = d / 2
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=color, width=width)


def render(state):
    img = Image.new("RGB", (W, H), (0, 0, 0))          # outside of the round panel = black
    d = ImageDraw.Draw(img)
    d.ellipse([0, 0, W - 1, H - 1], fill=C_BG)          # the visible panel
    f16, f20 = font(16), font(20)

    # close chip (top centre, LV_SYMBOL_CLOSE)
    d.ellipse([180 - 17, 8, 180 + 17, 8 + 34], fill=C_CHIP)
    d.line([173, 18, 187, 32], fill=C_MUTED, width=2)
    d.line([187, 18, 173, 32], fill=C_MUTED, width=2)

    status = {"idle": "待机", "listen": "正在聆听…", "think": "思考中…",
              "speak": "正在说话", "error": "连接出错"}[state]
    sub = {"idle": "今天天气怎么样", "listen": "", "think": "今天天气怎么样",
           "speak": "今天晴，最高 24 度", "error": "连接出错"}[state]
    hint = {"idle": "轻触说话 · 右滑返回", "listen": "再次轻触结束", "think": "右滑返回",
            "speak": "右滑返回", "error": "右滑返回"}[state]

    if state == "listen":
        # recording dot at (180, 88)
        d.ellipse([174, 88, 186, 100], fill=C_ACCENT)

    # eyes
    ew, eh = {"idle": (36, 46), "listen": (40, 50), "think": (34, 8),
              "speak": (36, 38), "error": (34, 6)}[state]
    ink = C_MUTED if state == "error" else C_INK
    for cx in (EYE_CX_L, EYE_CX_R):
        rounded_rect(d, cx, EYE_CY, ew, eh, ink, 16)

    # mouth
    if state == "idle":
        # arc: object 120x120 at (120,115), r=(120-2*7)/2=53, angles 25..155
        cx, cy, r = 180, 175, 53
        d.arc([cx - r, cy - r, cx + r, cy + r], start=25, end=155, fill=ink, width=7)
    elif state == "listen":
        ring(d, MOUTH_CX, MOUTH_CY, 26, 6, ink)
    elif state == "think":
        cx, cy, r = MOUTH_CX, MOUTH_CY, 25
        d.arc([cx - r, cy - r, cx + r, cy + r], start=0, end=360, fill=C_TRACK, width=5)
        d.arc([cx - r, cy - r, cx + r, cy + r], start=200, end=290, fill=C_ACCENT, width=5)
    elif state == "speak":
        rounded_rect(d, MOUTH_CX, MOUTH_CY, 44, 22, ink, 22)
    else:
        rounded_rect(d, MOUTH_CX, MOUTH_CY, 30, 6, ink, 16)

    # labels: top at y=236 / 260 / 284
    centered(d, (180, 236 + 10), status, f20, C_INK)
    if sub:
        centered(d, (180, 260 + 8), sub, f16, C_MUTED)
    centered(d, (180, 284 + 8), hint, f16, C_MUTED)

    d.ellipse([0, 0, W - 1, H - 1], outline=(70, 70, 70))
    return img


def main():
    states = ["idle", "listen", "think", "speak", "error"]
    sheet = Image.new("RGB", (W * len(states) + 12 * (len(states) - 1), H), (16, 16, 18))
    for i, s in enumerate(states):
        sheet.paste(render(s), (i * (W + 12), 0))
    sheet.save("docs/assistant_face_mock.png")
    print("saved docs/assistant_face_mock.png", sheet.size)


if __name__ == "__main__":
    main()
