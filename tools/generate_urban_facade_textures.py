#!/usr/bin/env python3
"""Generate deterministic, non-periodic facade textures for Gazebo.

The textures intentionally avoid regular window grids. Repeated, equally
spaced corners can produce locally convincing but geometrically wrong visual
matches. Each asset instead combines multi-scale surface variation with an
asymmetric set of coloured landmarks.
"""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = (
    ROOT
    / "src"
    / "multi_slam_uav_sim"
    / "models"
    / "s_curve_urban_structures"
    / "materials"
    / "textures"
)
SIZE = 1024
RESAMPLING = getattr(Image, "Resampling", Image)


def surface(base, seed):
    rng = np.random.default_rng(seed)
    coarse = rng.normal(0.0, 1.0, (64, 64)).astype(np.float32)
    coarse_image = Image.fromarray(
        np.uint8(np.clip(128.0 + coarse * 34.0, 0.0, 255.0)), mode="L"
    ).resize((SIZE, SIZE), RESAMPLING.BICUBIC)
    coarse_image = coarse_image.filter(ImageFilter.GaussianBlur(5.0))
    coarse_values = np.asarray(coarse_image, dtype=np.float32) - 128.0
    fine = rng.normal(0.0, 4.0, (SIZE, SIZE)).astype(np.float32)
    base_values = np.asarray(base, dtype=np.float32)[None, None, :]
    values = base_values + coarse_values[:, :, None] * 0.18 + fine[:, :, None]
    return Image.fromarray(np.uint8(np.clip(values, 0.0, 255.0)), mode="RGB")


def add_weathering(image, seed, dark=(45, 52, 56), light=(205, 210, 202)):
    rng = np.random.default_rng(seed)
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay, "RGBA")

    for _ in range(34):
        x = int(rng.integers(0, SIZE))
        y = int(rng.integers(0, SIZE))
        radius_x = int(rng.integers(24, 160))
        radius_y = int(rng.integers(18, 130))
        colour = dark if rng.random() < 0.68 else light
        alpha = int(rng.integers(7, 23))
        draw.ellipse(
            (x - radius_x, y - radius_y, x + radius_x, y + radius_y),
            fill=(*colour, alpha),
        )

    for _ in range(22):
        x = int(rng.integers(20, SIZE - 20))
        y = int(rng.integers(10, SIZE - 80))
        length = int(rng.integers(35, 170))
        points = [(x, y)]
        for _ in range(int(rng.integers(2, 5))):
            x += int(rng.integers(-18, 19))
            y += int(rng.integers(12, max(13, length // 2)))
            points.append((x, y))
        draw.line(points, fill=(*dark, int(rng.integers(25, 65))), width=2)

    overlay = overlay.filter(ImageFilter.GaussianBlur(0.8))
    return Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")


def panel(draw, box, fill, border=(30, 36, 40), width=10):
    draw.rounded_rectangle(box, radius=7, fill=fill, outline=border, width=width)
    x0, y0, x1, y1 = box
    inset = max(8, width + 2)
    draw.line(
        (x0 + inset, y0 + inset, x1 - inset, y0 + inset),
        fill=(245, 245, 232),
        width=max(2, width // 3),
    )


def facade_a():
    image = add_weathering(surface((104, 119, 126), 4101), 4102)
    draw = ImageDraw.Draw(image)
    panel(draw, (70, 95, 294, 292), (40, 105, 132), width=12)
    panel(draw, (390, 55, 582, 184), (190, 76, 58), width=9)
    panel(draw, (703, 118, 950, 365), (181, 154, 54), width=13)
    panel(draw, (151, 430, 432, 660), (63, 127, 91), width=11)
    panel(draw, (535, 344, 728, 584), (74, 83, 125), width=8)
    panel(draw, (777, 589, 960, 876), (168, 77, 116), width=12)
    panel(draw, (52, 754, 316, 942), (173, 106, 52), width=10)
    draw.polygon(
        [(410, 720), (690, 655), (716, 752), (441, 824)],
        fill=(54, 122, 132),
        outline=(26, 35, 38),
    )
    draw.line((360, 260, 930, 516), fill=(222, 219, 198), width=16)
    return image


def facade_b():
    image = add_weathering(surface((137, 116, 102), 4201), 4202)
    draw = ImageDraw.Draw(image)
    panel(draw, (45, 72, 242, 338), (42, 75, 91), width=11)
    panel(draw, (327, 116, 618, 281), (76, 119, 106), width=10)
    panel(draw, (712, 58, 944, 230), (165, 83, 55), width=12)
    panel(draw, (96, 455, 376, 648), (174, 143, 54), width=9)
    panel(draw, (488, 374, 684, 690), (61, 84, 130), width=13)
    panel(draw, (773, 396, 962, 594), (132, 67, 104), width=10)
    panel(draw, (166, 773, 472, 940), (43, 111, 123), width=12)
    panel(draw, (620, 745, 894, 930), (180, 93, 57), width=9)
    draw.polygon(
        [(22, 366), (460, 316), (471, 358), (36, 420)],
        fill=(211, 207, 184),
        outline=(46, 43, 41),
    )
    draw.line((541, 39, 967, 342), fill=(201, 188, 83), width=18)
    return image


def tunnel():
    image = add_weathering(
        surface((75, 82, 86), 4301), 4302, dark=(24, 28, 31), light=(164, 169, 164)
    )
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 714, SIZE, 790), fill=(205, 163, 45))
    draw.rectangle((0, 790, SIZE, 820), fill=(36, 43, 47))
    for start in (38, 271, 583, 844):
        draw.polygon(
            [(start, 1010), (start + 112, 1010), (start + 350, 716), (start + 238, 716)],
            fill=(224, 213, 174),
        )
    panel(draw, (80, 98, 310, 330), (39, 93, 121), width=11)
    panel(draw, (417, 162, 605, 415), (156, 65, 54), width=9)
    panel(draw, (733, 73, 958, 292), (62, 116, 77), width=12)
    draw.line((92, 537, 846, 466), fill=(178, 181, 173), width=12)
    return image


def canyon():
    image = add_weathering(surface((112, 119, 111), 4401), 4402)
    draw = ImageDraw.Draw(image)
    panel(draw, (58, 66, 332, 238), (165, 75, 50), width=11)
    panel(draw, (417, 96, 634, 352), (39, 94, 129), width=12)
    panel(draw, (747, 54, 965, 218), (185, 151, 52), width=9)
    panel(draw, (120, 424, 309, 715), (51, 124, 83), width=12)
    panel(draw, (496, 466, 782, 643), (129, 66, 110), width=10)
    panel(draw, (715, 752, 950, 944), (46, 100, 119), width=11)
    draw.polygon(
        [(30, 864), (381, 736), (405, 798), (62, 944)],
        fill=(197, 193, 169),
        outline=(37, 43, 43),
    )
    draw.line((348, 39, 912, 588), fill=(192, 83, 56), width=15)
    return image


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    assets = {
        "facade_a_v2.png": facade_a(),
        "facade_b_v2.png": facade_b(),
        "tunnel_v1.png": tunnel(),
        "canyon_v1.png": canyon(),
    }
    for name, image in assets.items():
        image.save(OUTPUT / name, optimize=True)
        print(OUTPUT / name)


if __name__ == "__main__":
    main()
