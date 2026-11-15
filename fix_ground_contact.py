#!/usr/bin/env python3
"""Make the x500 landing gear rest ON the visible arena floor.

Root cause
----------
`arena_ground` is a 100x100 m collision plane at z=0.019, while the visible
arena mesh along the launch / 5 m line corridor is at z≈0.265. The drone
therefore settled with its gear ~0.25 m inside the visible floor, both at
spawn (after the initial drop) and after landing.

Fix
---
Add a purely-collision (NO visual) contact pad over the launch corridor,
top face at z=0.265, exactly like the existing `base_landing_pad` does for
the far platform. Also align the spawn pose and the truth origin so the
initial state is already resting on that surface. No mesh, material, visual,
existing collision or model is modified.

Idempotent: re-running skips already-patched files. Original files are backed
up as *.bak_before_ground_fix_20261111.
"""

import os

ROOT = "/home/gengyinghao/drone_sim"
WORLDS = (
    os.path.join(ROOT, "rm27_sim", "worlds", "rm27_world.sdf"),
    os.path.join(ROOT, "rm27_sim", "worlds", "rm27_nav_world.sdf"),
)
ENV = os.path.join(ROOT, "env.sh")
SUFFIX = ".bak_before_ground_fix_20261111"

# Top face = 0.13 + 0.27/2 = 0.265 m (measured visible floor along the route).
# Coverage x=3..12, y=3..9 includes spawn (5,6) and the 5 m line landing
# point (10.17, 5.79), and avoids the far base platform (x≈24.7).
PAD = """    <!-- Landing-gear contact pad: COLLISION ONLY, no visual.
         The visible arena floor along the launch/landing corridor is z~0.265,
         but arena_ground's collision plane is z=0.019, so the gear used to
         sink ~0.25 m into the floor. This raises the physical contact surface
         to the visible floor without changing any mesh or appearance.
         Delete this block to revert; backup: *.bak_before_ground_fix_20261111 -->
    <model name="launch_ground_pad">
      <static>true</static>
      <pose>7.5 6.0 0.13 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>9.0 6.0 0.27</size></box></geometry>
        </collision>
      </link>
    </model>
"""


def patch(path, transform, label):
    raw = open(path, "rb").read()
    bom = raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("utf-8-sig")
    new = transform(text)
    if new == text:
        print("already patched / nothing to do:", label)
        return
    backup = path + SUFFIX
    if not os.path.exists(backup):
        open(backup, "wb").write(raw)
        print("backup:", backup)
    open(path, "wb").write((b"\xef\xbb\xbf" if bom else b"") + new.encode("utf-8"))
    print("patched:", label)


def world_transform(text):
    if "launch_ground_pad" in text:
        return text
    marker = "  </world>"
    assert marker in text, "missing </world>"
    return text.replace(marker, PAD + "\n" + marker, 1)


for w in WORLDS:
    patch(w, world_transform, os.path.basename(w))


def env_transform(text):
    old = 'export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-5,6,0.5,0,0,0}"'
    new = 'export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-5,6,0.265,0,0,0}"'
    return text.replace(old, new, 1)


patch(ENV, env_transform, "env.sh (spawn z 0.5 -> 0.265)")

print("done")
