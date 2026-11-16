"""3차선 임의 배치(장애물 7대) + 갈림길 표지판 합성 평가: 요청 이동 각도 설정별 성공률 비교 (11/16)

test/three_lane.py 시뮬을 그대로 씀. 3차로 시작 3.5 m, 갈림길 입구(흰 점선 끝) = 시작 + 5.4 m.
표지판은 입구 2.7 m 전부터 보임(11/16 실차 확정 거리 2.6~2.8 m). 조향 지연 0.7(실차 흉내).
판정: 입구에서 목적 차로(중앙에서 0.15 m 안) + 충돌 없음 + 바깥선 안 + 15초 넘는 정지 없음

실행: python3 lc_random_eval.py <ericar_driver 경로> [배치 수=40] [동시 실행 수=8]
한 건: python3 lc_random_eval.py <경로> --one <시드> <설정 이름>
"""
import json
import math
import random
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ZONE_START = 3.5
ENTRANCE = ZONE_START + 5.4
SIGN_FROM = ENTRANCE - 2.7
CONFIGS = {
    'one_lane': dict(request_plan=False),
    'fix36': dict(request_plan=True, request_angles=[36.0]),
    'fix45': dict(request_plan=True, request_angles=[45.0]),
    'fix54': dict(request_plan=True, request_angles=[54.0]),
    'adaptive': dict(request_plan=True, request_angles=[30.0, 36.0, 42.0, 48.0, 54.0]),
    'mid': dict(request_plan=True, request_angles=[40.0, 45.0, 50.0]),
}
import os
if os.environ.get('LC_ONLY'):
    CONFIGS = {k: v for k, v in CONFIGS.items() if k in os.environ['LC_ONLY'].split(',')}


def layout(seed):
    """장애물 7대: 줄 간격 0.9~1.4 m, 줄마다 1~2대, 이웃 줄의 빈 차로는 한 칸 안. 반환 (장애물 [(s, y)], 표지판 방향)"""
    rng = random.Random(seed)
    while True:
        rows, s, left = [], ZONE_START + rng.uniform(0.9, 1.4), 7
        while left > 0 and s <= ENTRANCE - 0.4:
            count = min(left, rng.choice([1, 2, 2]))
            lanes = rng.sample([0, 1, 2], count)
            rows.append((s, lanes))
            left -= count
            s += rng.uniform(0.9, 1.8)
        if left:
            continue
        free = [sorted(set([0, 1, 2]) - set(lanes)) for _, lanes in rows]
        # 통과 가능한 배치만: 이웃 줄 사이에 빈 차로가 이어지지 않으면(차로를 꼭 바꿔야 하면) 한 칸 거리 + 줄 간격 1.5 m 이상
        # (한 칸 이동 실측 1.06~1.14 m + 장애물 길이 0.32 m)
        ok = True
        for i in range(len(free) - 1):
            shift = min(abs(a - b) for a in free[i] for b in free[i + 1])
            gap = rows[i + 1][0] - rows[i][0]
            ok &= shift == 0 or (shift == 1 and gap >= 1.5)
        if not ok:
            continue
        # 마지막 줄에서 목적 차로(끝 차로)가 비어 있어야 함: 입구 바로 앞에 목적 차로가 막힌 배치는 풀 수 없음
        # 마지막 줄 뒤 입구까지: 목적 차로로 옮길 칸 수 × 1.2 m 이상 남아야 함 (그보다 짧으면 어떤 방식으로도 불가)
        room = ENTRANCE - rows[-1][0] - 0.16
        signs = [name for name, lane in (('left', 2), ('right', 0))
                 if min(abs(lane - f) for f in free[-1]) * 1.2 <= room]
        if not signs:
            continue
        obstacles = [(round(s + rng.uniform(-0.05, 0.05), 2), round((lane - 1) * 0.52 + rng.uniform(-0.05, 0.05), 2))
                     for s, lanes in rows for lane in lanes]
        return obstacles, rng.choice(signs)


def one(root, seed, name):
    sys.path.insert(0, str(Path(root) / 'test'))
    sys.path.insert(0, str(root))
    import numpy as np
    import three_lane
    from ericar_driver import params
    obstacles, sign = layout(seed)
    import os
    if os.environ.get('LC_CASE'):      # 직접 지정: {"obstacles": [[s, y], ...], "sign": "right"}
        custom = json.loads(os.environ['LC_CASE'])
        obstacles, sign = [tuple(o) for o in custom['obstacles']], custom['sign']
    three_lane.ZONE_END = ENTRANCE + 0.3
    paint = three_lane.paint_lines

    def paint_lines(ws, wl, finite):
        yellow, white = paint(ws, wl, finite)
        inner = np.abs(np.abs(wl) - 0.26) < three_lane.LINE
        white &= ~(inner & (ws >= ENTRANCE))      # 흰 점선은 갈림길 입구에서 끝남
        return yellow, white
    three_lane.paint_lines = paint_lines
    params.ANSWER_CONFIG['detector']['enabled'] = False
    lane_change = params.ANSWER_CONFIG['lane_change']
    lane_change.update(CONFIGS[name])
    lane_change['request_entrance_distance'] = ENTRANCE - 0.2      # 시뮬은 S2 거리가 s와 같음
    case = dict(signs={sign: (SIGN_FROM, ENTRANCE + 1.0)}, obstacles=obstacles, steer_lag=0.7)
    r = three_lane.run('random', case, duration=120.0)
    want = 2 if sign == 'left' else 0
    if os.environ.get('LC_VERBOSE'):
        print('\n'.join(r['timeline']))
    at = None
    for line in r['timeline']:
        parts = dict(p.split('=') for p in line.split()[2:] if '=' in p)
        s = float(line.split('s=')[1].split()[0])
        if s >= ENTRANCE - 0.05:
            at = (float(parts['y']), float(parts['psi']))
            break
    arrived = at is not None and abs(at[0] - (want - 1) * 0.52) < 0.15
    off = r['max_abs_y'] + three_lane.CAR_W / 2 > 0.78
    ok = arrived and not r['collision'] and not off and r['stuck'] < 15.0
    print(json.dumps(dict(seed=seed, config=name, ok=ok, arrived=arrived, collision=r['collision'], off=off,
                          stuck=round(r['stuck'], 1), reached=at is not None, y=None if at is None else at[0],
                          sign=sign, rows=len(set(round(o[0]) for o in obstacles)))))


def main():
    root = sys.argv[1]
    if '--one' in sys.argv:
        k = sys.argv.index('--one')
        return one(root, int(sys.argv[k + 1]), sys.argv[k + 2])
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    jobs = [(seed, name) for seed in range(count) for name in CONFIGS]

    def launch(job):
        out = subprocess.run([sys.executable, __file__, root, '--one', str(job[0]), job[1]],
                             capture_output=True, text=True)
        lines = [line for line in out.stdout.splitlines() if line.startswith('{')]
        return json.loads(lines[-1]) if lines else dict(seed=job[0], config=job[1], ok=False, error=out.stderr[-300:])
    with ThreadPoolExecutor(workers) as pool:
        results = list(pool.map(launch, jobs))
    Path(__file__).with_name('lc_random_eval_result.json').write_text(json.dumps(results, ensure_ascii=False))
    print('설정      성공  목적차로  충돌  이탈  15초정지  입구미도달  오류')
    for name in CONFIGS:
        rows = [r for r in results if r['config'] == name]
        print('%-9s %3d/%d %6d %6d %5d %7d %9d %6d' % (
            name, sum(r['ok'] for r in rows), len(rows), sum(r.get('arrived', False) for r in rows),
            sum(r.get('collision', False) for r in rows), sum(r.get('off', False) for r in rows),
            sum(r.get('stuck', 0) >= 15 for r in rows), sum(not r.get('reached', False) for r in rows),
            sum('error' in r for r in rows)))
    bad = sorted(set(r['seed'] for r in results if r['config'] == 'adaptive' and not r['ok']))
    print('adaptive 실패 시드', bad)


if __name__ == '__main__':
    main()
