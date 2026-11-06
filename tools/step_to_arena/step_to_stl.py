"""用 OCCT 的 STEPControl_Reader 直接读几何（不走 XCAF），绕开 17.8 万个面样式。

用法: python _convert_ocp.py [tol_linear_mm]
输出: D:\\rmuc_tmp\\arena_ocp.stl（二进制，无颜色）
"""
import sys
import time
from pathlib import Path

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

SRC = Path(r"D:\rmuc_tmp\RMUC2027_V1.0.0.stp")
OUT = Path(r"D:\rmuc_tmp\arena_ocp.stl")
tol = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

reader = STEPControl_Reader()
t0 = time.time()
status = reader.ReadFile(str(SRC))
print(f"ReadFile status={status} {time.time() - t0:.1f}s", flush=True)
if status != IFSelect_RetDone:
    raise SystemExit("STEP read failed")

t0 = time.time()
reader.TransferRoots()
shape = reader.OneShape()
print(f"TransferRoots {time.time() - t0:.1f}s, null={shape.IsNull()}", flush=True)

n = 0
exp = TopExp_Explorer(shape, TopAbs_SOLID)
while exp.More():
    n += 1
    exp.Next()
print(f"solids={n}", flush=True)

t0 = time.time()
BRepMesh_IncrementalMesh(shape, tol, False, 0.4, True)
print(f"mesh {time.time() - t0:.1f}s", flush=True)

t0 = time.time()
writer = StlAPI_Writer()
writer.ASCIIMode = False
writer.Write(shape, str(OUT))
print(f"stl {OUT.stat().st_size / 1e6:.1f} MB, {time.time() - t0:.1f}s", flush=True)
