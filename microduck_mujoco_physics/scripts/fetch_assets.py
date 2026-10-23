"""Prepare the official Microduck MJCF and STL assets for a static Vite build.
Requires Internet during this step, not at runtime. No third party Python dependencies.
"""
import re, urllib.request, pathlib, xml.etree.ElementTree as ET
root=pathlib.Path(__file__).resolve().parents[1]/'public'/'robot'
root.mkdir(parents=True,exist_ok=True)
base='https://raw.githubusercontent.com/pollen-robotics/microduck_rl/develop/src/mjlab_microduck/robot/microduck/'
alt='https://cdn.jsdelivr.net/gh/pollen-robotics/microduck_rl@develop/src/mjlab_microduck/robot/microduck/'

def fetch(path):
    target=root/path
    if target.exists() and target.stat().st_size>100:return
    err=None
    for prefix in (base,alt):
        try:
            req=urllib.request.Request(prefix+path,headers={'User-Agent':'MicroduckPhysics/0.1'})
            with urllib.request.urlopen(req,timeout=35) as response:data=response.read()
            if len(data)<80 or data.lstrip()[:1]==b'<':
                if not path.endswith('.xml'):raise ValueError('Unexpected HTML/XML for binary asset')
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(data)
            print('Downloaded',path,len(data))
            return
        except Exception as exc:err=exc
    raise RuntimeError(f'Cannot fetch {path}: {err}')

fetch('robot_walk.xml')
xml=(root/'robot_walk.xml').read_text()
for name in set(re.findall(r'<mesh\b[^>]*\bfile\s*=\s*["\']([^"\']+)',xml)):
    fetch(name if name.startswith('assets/') else 'assets/'+name)
# For optional include files, retrieve recursively.
visited=set()
def includes(document):
    for name in re.findall(r'<include\s+file\s*=\s*["\']([^"\']+)',document):
        if name in visited:continue
        visited.add(name);fetch(name);includes((root/name).read_text())
includes(xml)
print('Assets ready in',root)
