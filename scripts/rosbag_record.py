#!/usr/bin/env python3

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--no-lz4", action="store_true")
    parser.add_argument("topics", nargs="+")
    args = parser.parse_args()

    out_dir = os.path.expanduser(args.dir)
    os.makedirs(out_dir, exist_ok=True)

    cmd = ["rosbag", "record"]
    if not args.no_lz4:
        cmd.append("--lz4")
    cmd += ["-o", os.path.join(out_dir, args.prefix)]
    cmd += list(args.topics)

    os.execvp(cmd[0], cmd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

