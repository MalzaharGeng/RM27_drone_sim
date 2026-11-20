#!/usr/bin/env python3
"""Link the actual mixer functional tests against a completed POSIX build.

Uses the system GoogleTest library, avoiding an unrelated fuzztest download.
The tested mixer/parameters/uORB are the production libraries from that build.
"""
import json
import pathlib
import shlex
import subprocess
import sys

build = pathlib.Path(sys.argv[1]).resolve()
commands = json.loads((build / 'compile_commands.json').read_text())
entry = next(e for e in commands if e['file'].endswith('/mixer_module.cpp'))
root = pathlib.Path(entry['file']).parents[3]
argv = shlex.split(entry['command'])
flags = []
i = 1
while i < len(argv):
    arg = argv[i]
    if arg in ('-o', '-c'):
        i += 2
        continue
    if arg.startswith('-DMODULE_NAME='):
        arg = '-DMODULE_NAME="flip_mixer_host"'
    if arg.startswith('-std='):
        arg = '-std=gnu++17'  # system GoogleTest requires C++17; production stays C++14
    flags.append(arg)
    i += 1

out = build / 'flip_mixer_host'
out.mkdir(exist_ok=True)
sources = [root / 'src/lib/mixer_module/mixer_module_tests.cpp',
           root / 'test/flip_recovery/mixer_test_main.cpp',
           root / 'platforms/posix/src/px4/common/test_stubs/stub_daemon.cpp']
objects = []
for src in sources:
    obj = out / (src.stem + '.o')
    cmd = [argv[0], *flags, '-c', str(src), '-o', str(obj)]
    subprocess.run(cmd, cwd=entry['directory'], check=True)
    objects.append(str(obj))

archives = sorted(str(p) for p in build.rglob('*.a') if 'CMakeScratch' not in str(p))
exe = out / 'mixer_tests'
subprocess.run(['g++', *objects, '-Wl,--start-group', *archives, '-Wl,--end-group',
                '-lgtest', '-lpthread', '-ldl', '-lrt', '-lm', '-o', str(exe)], check=True)
subprocess.run([str(exe), '--gtest_filter=MixerModuleTest.recoveryThrottleAndSafety'],
               cwd=out, check=True)
