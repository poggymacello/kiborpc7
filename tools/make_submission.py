#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Build the Web-simulator submission archive exactly like users/packing.sh would:

    cd users && tar -zcvf <name>.tar.gz catkin_ws/src

It starts from the OFFICIAL krpc7th-simulator.tar.gz (so the provided krpc_api_server,
krpc_ib2_msgs and the catkin top-level CMakeLists.txt symlink are byte-identical) and only
replaces krpc_user/scripts/user_template.py with our program. Works on Windows too.

usage:  python tools/make_submission.py [--name GaneshaOrbit_v1] [--mode fast|safe]
"""
import argparse
import io
import os
import re
import tarfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OFFICIAL = os.path.join(ROOT, "vendor", "krpc7th-simulator.tar.gz")
OURS = os.path.join(ROOT, "krpc_user", "scripts", "user_template.py")
PREFIX = "krpc7th-simulator/users/"
TARGET = "catkin_ws/src/krpc_user/scripts/user_template.py"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="GaneshaOrbit_v1")
    ap.add_argument("--mode", choices=["fast", "safe"], default=None,
                    help="override CONFIG['MOTION_MODE'] inside the packaged script")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "submission"))
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    with open(OURS, "rb") as f:
        code = f.read()
    if args.mode:
        code, n = re.subn(rb'"MOTION_MODE": "(fast|safe)"', ('"MOTION_MODE": "%s"' % args.mode).encode(), code)
        assert n == 1, "MOTION_MODE line not found"
    compile(code, "user_template.py", "exec")  # syntax guard
    out = os.path.join(args.outdir, args.name + ".tar.gz")
    replaced = False
    with tarfile.open(OFFICIAL, "r:gz") as src, tarfile.open(out, "w:gz", format=tarfile.GNU_FORMAT) as dst:
        for m in src.getmembers():
            if not m.name.startswith(PREFIX + "catkin_ws/src"):
                continue
            name = m.name[len(PREFIX):]
            info = tarfile.TarInfo(name)
            info.mode, info.uid, info.gid, info.uname, info.gname = m.mode, m.uid, m.gid, m.uname, m.gname
            info.mtime = m.mtime
            if m.issym():
                info.type, info.linkname = tarfile.SYMTYPE, m.linkname
                dst.addfile(info)
            elif m.isdir():
                info.type = tarfile.DIRTYPE
                dst.addfile(info)
            elif m.isfile():
                data = src.extractfile(m).read()
                if name == TARGET:
                    data = code
                    info.mtime = int(time.time())
                    info.mode = 0o755
                    replaced = True
                info.size = len(data)
                dst.addfile(info, io.BytesIO(data))
    assert replaced, "user_template.py not found in the official archive"
    print("written %s (%d bytes)" % (out, os.path.getsize(out)))
    with tarfile.open(out, "r:gz") as t:
        for m in t.getmembers():
            if "krpc_user" in m.name or m.issym():
                print("  %s%s" % (m.name, " -> " + m.linkname if m.issym() else ""))


if __name__ == "__main__":
    main()
