#!/usr/bin/env python3
"""在场地贴图上叠加高对比细节，用于给 ORB-SLAM3 提供密集角点。

原贴图是 RoboMaster 场地图，大面积深灰区域对比度低；相机前视时近处地面临近
纹理稀疏，飞行段容易丢跟踪。这里叠加规则网格线 + 随机高对比块 + 圆点，
不改变地面几何，只增加可提取特征。
"""
import os
import random
import sys

from PIL import Image, ImageDraw, ImageEnhance

SRC = "/home/gengyinghao/drone_sim/rm27_sim/models/rmuc_2027/materials/textures/rmuc2027_floor.png"
OUT = "/home/gengyinghao/drone_sim/rm27_sim/models/arena_texture_patches/meshes/floor_detail.png"
COPY = "/home/gengyinghao/drone_sim/rm27_sim/models/arena_texture_patches/materials/textures/floor_detail.png"


def main():
    if not os.path.exists(SRC):
        print("缺少源贴图", SRC)
        return 1
    im = Image.open(SRC).convert("RGB")
    w, h = im.size
    # 底图 77% 的像素亮度低于 110（中位 80），渲染出来是大片暗区，
    # FAST 在暗区提不到角点。整体提亮 + 略提对比，只影响这张视觉层贴图。
    im = ImageEnhance.Brightness(im).enhance(1.65)
    im = ImageEnhance.Contrast(im).enhance(1.12)
    d = ImageDraw.Draw(im)
    random.seed(1111)

    # 规则网格：每 72 px 一格（约 0.78 m），给整幅地面一个均匀的角点骨架。
    # 实测这一版（网格+随机块+圆环）跟踪保持最好；去掉网格、改加立体参照物
    # 反而出现 171 次地图重建，故保留并加密。
    for x in range(0, w, 72):
        d.line([(x, 0), (x, h)], fill=(235, 235, 235), width=2)
    for y in range(0, h, 72):
        d.line([(0, y), (w, y)], fill=(235, 235, 235), width=2)

    # 随机高对比矩形块（多尺度）
    for _ in range(3400):
        x = random.randint(0, w - 30)
        y = random.randint(0, h - 30)
        s = random.choice((6, 9, 14, 22, 30))
        c = random.choice([(250, 250, 250), (12, 12, 12), (235, 50, 50), (40, 110, 235)])
        d.rectangle([x, y, x + s, y + s], fill=c)

    # 随机圆环
    for _ in range(1500):
        x = random.randint(20, w - 20)
        y = random.randint(20, h - 20)
        r = random.randint(5, 14)
        d.ellipse([x - r, y - r, x + r, y + r], outline=(250, 250, 250), width=3)

    # 深色细线，制造局部高频对比（不与随机块方向一致）
    for _ in range(1200):
        x = random.randint(0, w - 34)
        y = random.randint(0, h - 34)
        ln = random.randint(12, 32)
        d.line([(x, y), (x + ln, y + random.randint(-14, 14))], fill=(15, 15, 15), width=2)

    im.save(OUT, quality=92)
    im.save(COPY, quality=92)
    print("已生成", OUT, im.size)
    return 0


if __name__ == "__main__":
    sys.exit(main())
