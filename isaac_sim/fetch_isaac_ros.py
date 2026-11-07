"""Unpack NVIDIA Isaac ROS 4.0 (Jazzy, arm64) cuVSLAM packages into a private prefix.

No sudo, no change to the system: resolves the dependency closure of the requested
packages against the Isaac ROS apt index, skips anything already installed (dpkg),
downloads the .debs and `dpkg -x`es them under --prefix. Source it with
sim/organizer/isaac_ros_env.sh.

    python3 fetch_isaac_ros.py --prefix ~/organizer-sim/isaac_ros \
        ros-jazzy-isaac-ros-visual-slam ros-jazzy-isaac-ros-image-proc
"""
import argparse
import gzip
import re
import subprocess
import urllib.request
from pathlib import Path

BASE = 'https://isaac.download.nvidia.com/isaac-ros/release-4.0'
INDEX = BASE+'/dists/noble/main/binary-arm64/Packages.gz'


def parse(text):
    pk = {}
    for block in text.split('\n\n'):
        f = {}
        key = None
        for line in block.splitlines():
            if line.startswith(' ') and key:
                f[key] += line
            elif ':' in line:
                key, v = line.split(':', 1)
                f[key] = v.strip()
        if 'Package' in f:
            pk[f['Package']] = f
    return pk


def deps(field):
    out = []
    for alt in (field or '').split(','):
        name = re.split(r'[\s(]', alt.strip().split('|')[0].strip())[0]
        if name:
            out.append(name)
    return out


def installed(name):
    r = subprocess.run(['dpkg-query', '-W', '-f=${Status}', name], capture_output=True, text=True)
    return r.returncode == 0 and 'install ok installed' in r.stdout


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--prefix', type=Path, required=True)
    p.add_argument('packages', nargs='+')
    a = p.parse_args()
    idx = parse(gzip.decompress(urllib.request.urlopen(INDEX, timeout=120).read()).decode())
    todo, seen, need, missing = list(a.packages), set(), [], []
    while todo:
        n = todo.pop()
        if n in seen:
            continue
        seen.add(n)
        if n in idx:
            if not installed(n):
                need.append(n)
            todo += deps(idx[n].get('Depends')) + deps(idx[n].get('Pre-Depends'))
        elif not installed(n):
            missing.append(n)
    print('from Isaac ROS index:', len(need), 'packages')
    print('not installed and not in the Isaac index (must come from Ubuntu/ROS apt):', sorted(missing))
    debs = a.prefix/'debs'
    debs.mkdir(parents=True, exist_ok=True)
    for n in sorted(need):
        f = idx[n]['Filename']
        dst = debs/Path(f).name
        if not dst.exists():
            urllib.request.urlretrieve(BASE+'/'+f, dst)
        subprocess.run(['dpkg', '-x', str(dst), str(a.prefix/'root')], check=True)
        print('unpacked', n, idx[n]['Version'])
    (a.prefix/'missing.txt').write_text('\n'.join(sorted(missing))+'\n')


if __name__ == '__main__':
    main()
