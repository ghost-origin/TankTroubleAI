# -*- coding: utf-8 -*-
import re, json, sys
d = open(r'C:\Users\jasongeorge\deepseek harness workspace\tanktrouble12\src\tanktrouble\data.js', encoding='utf-8').read()
# find object type blocks: names mentioning tank/turret/gun/bullet
names = re.findall(r'"(?:name|fileName)"\s*:\s*"([^"]+)"', d)
interesting = [n for n in names if re.search(r'tank|turret|gun|bullet|tanktrouble', n, re.I)]
print("names with tank/gun:", interesting[:60])
# look for object type definitions with "angle" default: pattern near "tank"
for m in re.finditer(r'\{[^{}]*"name"\s*:\s*"([^"]*[Tt]ank[^"]*)"[^{}]*\}', d):
    seg = m.group(0)
    if '"angle"' in seg:
        print("OBJ", m.group(1)[:40], "->", seg[:400].replace('\n', ' '))
        break
