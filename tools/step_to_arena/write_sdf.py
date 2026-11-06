"""生成按原包逐面颜色导出的场地 SDF（9 个 visual）。"""
import pathlib
import xml.etree.ElementTree as ET

OUT = pathlib.Path("//wsl$/Ubuntu-26.04/home/gengyinghao/drone_sim/rm27_sim/models/rmuc_2027/model.sdf")

VISUALS = [
    ("black", 0.030, 0.030, 0.030),
    ("darkgrey", 0.150, 0.150, 0.150),
    ("grey", 0.320, 0.320, 0.320),
    ("beige", 0.620, 0.590, 0.470),
    ("white", 0.900, 0.900, 0.900),
    ("red", 0.750, 0.100, 0.100),
    ("blue", 0.100, 0.220, 0.700),
    ("deepblue", 0.020, 0.070, 0.510),
    ("cyan", 0.050, 0.310, 0.350),
]

HEAD = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="rmuc_2027">
    <static>true</static>
    <link name="arena">
      <!-- 实体渲染：官方 STP 经 OCCT 三角化，颜色取自 XCAF label 树里的逐面
           颜色（59,685 个面带色，含纯红/纯蓝），按 9 个主色聚类。
           不再使用贴图或 UV；四区后备版本在 model.sdf.bak_fourzone。 -->
"""

BLOCK = """      <visual name="arena_{name}">
        <geometry>
          <mesh>
            <uri>meshes/color_{name}.stl</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>{ar:.3f} {ag:.3f} {ab:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <specular>0.030 0.030 0.030 1</specular>
        </material>
      </visual>

"""

TAIL = """    </link>
  </model>
</sdf>
"""

parts = [HEAD]
for name, r, g, b in VISUALS:
    parts.append(BLOCK.format(name=name, r=r, g=g, b=b, ar=r * 0.5, ag=g * 0.5, ab=b * 0.5))
parts.append(TAIL)

text = "".join(parts)
OUT.write_bytes(text.encode("utf-8"))
ET.fromstring(text)
print("ok", len(text), "bytes,", text.count("<visual name="), "visuals")
