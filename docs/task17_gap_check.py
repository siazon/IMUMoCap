
import csv, sys
from collections import defaultdict
CSV_PATH = sys.argv[1]
frames = defaultdict(int)
with open(CSV_PATH, encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        pid = int(row['PacketId']); role = row['Role'].strip()
        if role in ('Pelvis','Left','Right'):
            frames[pid] += 1
pids = sorted(p for p,c in frames.items() if c==3)
gaps = [(pids[i+1]-pids[i]) for i in range(len(pids)-1)]
big = [(pids[i], pids[i+1], pids[i+1]-pids[i]) for i in range(len(pids)-1) if pids[i+1]-pids[i] > 50]
print(f"n={len(pids)}  min_gap={min(gaps)}  max_gap={max(gaps)}  mean_gap={sum(gaps)/len(gaps):.2f}")
print(f"num gaps > 50 samples (0.5s @100Hz nominal): {len(big)}")
for b in big[:20]:
    print("  gap:", b, f"-> {b[2]/100:.1f}s")
