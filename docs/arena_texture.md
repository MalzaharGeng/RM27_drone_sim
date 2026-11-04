# RMUC 2027 场地贴图

2026-11-03 起做，2026-11-04 定稿。目标是让 Gazebo 里的场地外观贴近真实赛场
（地面标线 + 立体结构着色）。过程记录见 `worklog/2026-11-03.md`。

## 当前方案

整场一个 OBJ 网格，用官方 2027 俯视图做平面投影贴图：

| 项目 | 内容 |
|---|---|
| 网格 | `rm27_sim/models/rmuc_2027/meshes/arena_textured.obj`，35.3 MB，17.5 万顶点 / 37.9 万三角面，含 UV + 法线 |
| 材质 | `arena_textured.mtl`，`map_Kd rmuc2027_floor.png`（裸文件名，与 MTL 同目录） |
| 贴图 | `rmuc2027_floor.png`，2800 × 1500（28 × 15 m @ 100 px/m），图源是 diaofenyuan/RMUC_2027map_unity 的 `map/rm2027-map.png` |
| UV | 平面投影：`u = 1 − (x−0.1491)/28`、`v = (y−0.5655)/15`，超出比赛区钳制 |
| 挂载 | 世界层级模型 `arena_textured`（`rm27_world.sdf` / `rm27_nav_world.sdf`），原来 `model://rmuc_2027` 的 include 已停用 |
| 原始网格 | `rmuc_2027.stl` 保留在同一目录，只作形状基准，不参与渲染 |

## 结论

1. **贴图 URI 在 `model://` 包含的模型里会失效**：gz 会把它和模型目录拼接，
   绝对路径被破坏。plane / box / OBJ+MTL / DAE 四种做法在模型内全部失败，
   挪到世界文件层级后立刻生效。这是整件事最要紧的一条规律。
2. **`<plane>` 几何体的 UV 是退化的**（整面采样同一个像素），平面贴图必须用
   自带 UV 的网格。
3. **OBJ 必须带法线**，否则 ogre 报 `No vertex normals found in TangentSpaceCalc`，
   材质构建失败，整场变纯白。
4. **MTL 的 `map_Kd` 用相对文件名**（贴图和 MTL 同目录）；写绝对路径会找不到贴图，
   同样变白。
5. **`<pbr><metal><albedo_map>` 必须在 `<metal>` 内**，否则世界加载失败（Error 11/9）。
6. **STL 的可见地表不是包围盒最低点**：0.0194 m 是底板下表面；实际地面在
   0.17–0.40 m（双坡，中位 0.29），中央高地到 0.72 m。任何"贴地平面"都要
   ≥ 0.41 m 才露得出来——正因为如此，最终没有走贴地平面方案。
7. ogre2.log 里的 `Fixed Function pipeline is no longer allowed` /
   `Can't assign material scene::Material(...)` 是这套环境的固有噪音，与贴图无关。

## 已知限制

- 底图源约 31 px/m（官方手册图 4-1 原图 981 × 559 经单应展开），**近看地面纹理偏软**，
  这是数据源上限；试过 5600 × 3000 上采样加锐化，观感反而更差，已回退到 2800 × 1500 原图。
- 平面投影会让**竖直面（侧壁、立柱）出现纵向拖影**，占三角面约 55%。试过"竖直面
  平涂去拖影"，整体观感变差，已放弃。
- 想要真正清晰、无拖影，只能换 CAD 资产（`Project/Assets/Field/field_visual_m.fbx`，
  185 MB / 707 万面 / 42 种 CAD 材质），或官方 653 MB 的 `RMUC2027_V1.0.0.stp`
  （BBS 帖 1970956 开源），属独立的大改造。

## 待办

1. 拿到更高分辨率的 2027 官方俯视图后，直接同名替换 `rmuc2027_floor.png`
   （保持 28 × 15 比例）。
2. 若走 CAD 路线：FBX/STEP → 抽稀 → OBJ/GLB + 按件材质，UV 与文件体积都要重做。
3. 物理侧仍是 `arena_ground`（z = 0.019 平面）+ `base_landing_pad`，与视觉无关，未改。

## 相关文件

- `rm27_sim/models/rmuc_2027/meshes/`：`arena_textured.obj` / `.mtl`、
  `rmuc2027_floor.png`、`rmuc_2027.stl`（原始网格）
- `rm27_sim/worlds/rm27_world.sdf`、`rm27_nav_world.sdf`：`arena_textured` 模型定义
- 调试期间的中间产物（DAE 地面、旋转镜像变体、三色隔离测试场景）没有入库，
  结论已收敛到上面的"结论"一节
