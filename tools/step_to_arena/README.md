# 从官方 STP 生成场地网格

`rm27_sim/models/rmuc_2027/meshes/color_*.stl` 和同目录的 `model.sdf` 由官方
`RMUC2027_V1.0.0.stp`（653 MB，CREO 导出）生成，不是手工建模，也不使用贴图。

## 依赖

脚本在 Windows / Linux 侧的普通 Python 里跑（WSL 里没有装 OCCT 绑定）：

```bash
pip install cadquery-ocp trimesh numpy
```

## 步骤

1. 解压官方包，得到 `RMUC2027_V1.0.0.stp`，把脚本顶部的 `SRC` / `OUT` 改成实际路径；
2. `python export_colored_mesh.py <stp> <out_dir>`：
   - `STEPCAFControl_Reader` 建 XCAF 装配，653 MB 约 15 分钟；
   - 12 mm 精度三角化；
   - 遍历 `TDF_ChildIterator`，按 label 取每个面的颜色（59,685 个面带色）；
   - 按 9 个主色聚类，每色导出一个 STL；
3. `python write_sdf.py <mesh_dir> <model.sdf>`：生成 9 个 visual 的 SDF；
4. 把 STL 和 SDF 放进 `rm27_sim/models/rmuc_2027/`。

## 关键点

- **颜色不在 `TopoDS_Face` 上**：`GetColor(face, ...)` 对 21 万个面全部返回 False。
  必须遍历 label，用 `GetShape_s(label, shape)` 取 label 里的 shape 再查色。
- **坐标要用整体 shape**：直接三角化 label 的 face 会丢装配 location，z 会从
  0.019 偏到 -0.31。正确做法是 `GetOneShape()` 遍历 face 三角化，颜色用
  `TShape()` 指针去 label 的映射里查。
- **不要逐面保留 319 色**：面级样式 17.9 万条，原样丢给 Gazebo 会有十几万次材质
  切换；聚到 9 个主色后 draw call 固定为 9。
- **单位与偏移**：STP 单位是 mm，原点在场地中心附近；缩放到米后平移
  `(14.149, 6.441, 1.960)`，对齐到 x 0.149..28.149、y 0.064..16.067、z 0.019..3.817。

## 已知限制

- 材质只有 ambient/diffuse，没有 roughness/metalness。这台 Gazebo 上 `<pbr>` 的
  albedo 对 STL 不生效，四个 visual 会一起显示成默认白色。
- 9 个主色是近似：CREO 里的细微分色会并入最近的主色。
- 官方 STP 里没有渲染图，参考效果来自同批次的 CAD 出图，不在包内。
