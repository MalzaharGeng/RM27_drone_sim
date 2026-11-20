"""把仓库内 PX4 源码导出为独立 Git 构建树，固定原始依赖 SHA。"""
import argparse,json,pathlib,shutil,subprocess,os
ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--submodules',action='store_true');args=ap.parse_args()
root=pathlib.Path(__file__).resolve().parents[2];source=root/'PX4-Autopilot';manifest=json.loads((root/'docs/PX4_SOURCE_MANIFEST.json').read_text())
dst=pathlib.Path(args.output).expanduser().resolve()
if dst.exists():raise SystemExit('output must be a new directory: '+str(dst))
dst.mkdir(parents=True)
def git(*a):subprocess.run(['git','-C',str(dst),*a],check=True)
# Copy tracked source only, without a populated dependency/build tree.
listed=subprocess.check_output(['git','-C',str(root),'ls-files','-z','PX4-Autopilot']).decode().split('\0')
if not any(listed):raise SystemExit('PX4 snapshot must be committed before exporting')
subs={x['path'] for x in manifest['submodules']}
for name in listed:
    if not name:continue
    rel=pathlib.Path(name).relative_to('PX4-Autopilot')
    if str(rel) in subs:continue
    p=source/rel;out=dst/rel
    if p.is_symlink():out.parent.mkdir(parents=True,exist_ok=True);out.symlink_to(os.readlink(p))
    elif p.is_file():out.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,out)
git('init','-b','snapshot');git('config','user.name','PX4 snapshot');git('config','user.email','snapshot@localhost')
git('add','--force','.')
for item in manifest['submodules']:git('update-index','--add','--cacheinfo','160000,'+item['sha']+','+item['path'])
git('commit','-m','Import PX4 source snapshot '+manifest['source_commit'])
# Preserve the recorded custom source identity for PX4's build-version generator.
git('tag','-a','v1.17.0','-m','Snapshot baseline; original source '+manifest['source_commit'])
(dst/'SOURCE_REVISION.json').write_text(json.dumps(manifest,indent=2)+'\n')
if args.submodules:git('submodule','update','--init','--recursive')
print('Build source:',dst)
