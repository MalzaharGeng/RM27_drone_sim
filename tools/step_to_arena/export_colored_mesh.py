"""第五轮：用整体 shape 的 face 三角化（坐标正确），颜色按 TShape 指针从 label 查。

第四轮直接三角化 label 的 face，坐标缺了装配 location，z 偏到 -0.31；
这里改用 whole 的 face（与四区同一条路径，bbox 已验证），
颜色映射用 TopoDS_Shape.TShape() 作键。
"""
import collections
import time
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_ChildIterator
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_DocumentTool

SRC = Path(r"D:\rmuc_tmp\RMUC2027_V1.0.0.stp")
OUT = Path(r"D:\rmuc_tmp\arena_match")
OFFSET = np.array([14.149, 6.441, 1.960])

PALETTE = {
    "black": (0.03, 0.03, 0.03),
    "darkgrey": (0.15, 0.15, 0.15),
    "grey": (0.32, 0.32, 0.32),
    "beige": (0.62, 0.59, 0.47),
    "red": (1.00, 0.00, 0.00),
    "blue": (0.00, 0.00, 1.00),
    "deepblue": (0.02, 0.07, 0.51),
    "white": (1.00, 1.00, 1.00),
    "cyan": (0.05, 0.31, 0.35),
}


def classify(rgb):
    best, dist = "beige", 1e9
    for name, ref in PALETTE.items():
        d = sum((a - b) ** 2 for a, b in zip(rgb, ref))
        if d < dist:
            best, dist = name, d
    return best


def main() -> None:
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(False)
    t0 = time.time()
    reader.ReadFile(str(SRC))
    print(f"ReadFile {time.time() - t0:.1f}s", flush=True)
    t0 = time.time()
    reader.Transfer(doc)
    print(f"Transfer {time.time() - t0:.1f}s", flush=True)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    color_map = {}
    t0 = time.time()
    it = TDF_ChildIterator(doc.Main(), True)
    while it.More():
        label = it.Value()
        shape = TopoDS_Shape()
        if shape_tool.GetShape_s(label, shape) and not shape.IsNull():
            if shape.ShapeType() == TopAbs_FACE:
                col = Quantity_Color()
                got = color_tool.GetColor(shape, XCAFDoc_ColorSurf, col)
                if not got:
                    got = color_tool.GetColor(shape, XCAFDoc_ColorGen, col)
                if got:
                    color_map.setdefault(shape.TShape(), (col.Red(), col.Green(), col.Blue()))
        it.Next()
    print(f"colour map {len(color_map)} faces, {time.time() - t0:.1f}s", flush=True)

    whole = shape_tool.GetOneShape()
    BRepMesh_IncrementalMesh(whole, 12.0, False, 0.4, True)

    buckets = {k: [] for k in PALETTE}
    counts = collections.Counter()
    nface = 0
    t0 = time.time()
    exp = TopExp_Explorer(whole, TopAbs_FACE)
    while exp.More():
        nface += 1
        face = TopoDS.Face(exp.Current())
        rgb = color_map.get(face.TShape())
        name = classify(rgb) if rgb else "beige"
        counts[name] += 1
        tri = BRep_Tool.Triangulation_s(face, face.Location())
        if tri is not None and tri.NbTriangles() > 0:
            trsf = face.Location().Transformation()
            nodes = np.empty((tri.NbNodes(), 3), dtype=np.float64)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                nodes[i - 1] = (p.X(), p.Y(), p.Z())
            nodes = nodes * 0.001 + OFFSET
            idx = np.empty((tri.NbTriangles(), 3), dtype=np.int64)
            for i in range(1, tri.NbTriangles() + 1):
                idx[i - 1] = tri.Triangle(i).Get()
            buckets[name].append(nodes[idx - 1])
        exp.Next()

    print(f"faces {nface}, {time.time() - t0:.1f}s", flush=True)
    print("face counts", dict(counts), flush=True)

    OUT.mkdir(parents=True, exist_ok=True)
    for name, chunks in buckets.items():
        if not chunks:
            continue
        tri = np.concatenate(chunks, axis=0)
        stl = OUT / f"color_{name}.stl"
        trimesh.Trimesh(vertices=tri.reshape(-1, 3), faces=np.arange(len(tri) * 3).reshape(-1, 3), process=False).export(stl)
        print(f"{name}: {len(tri)} tris, {stl.stat().st_size / 1e6:.1f} MB", flush=True)


if __name__ == "__main__":
    main()
