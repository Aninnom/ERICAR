"""리팩토링 회귀 시험: 시나리오 실행 결과가 기준값과 완전히 같은지 비교

실행: python3 -m pytest test/test_regression.py  (또는 python3 test/test_regression.py)
동작을 의도적으로 바꾼 경우에만 python3 test/scenario.py --write 로 기준값 갱신
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import scenario  # noqa: E402


def test_same_as_golden():
    expected = json.loads(scenario.GOLDEN.read_text())
    actual = json.loads(json.dumps(scenario.run(str(HERE.parent / 'config' / 'robot.yaml'))))
    assert len(actual) == len(expected), (len(actual), len(expected))
    for index, (a, e) in enumerate(zip(actual, expected)):
        assert a == e, f'{index}번째 기록 다름\n기준: {str(e)[:300]}\n결과: {str(a)[:300]}'


if __name__ == '__main__':
    test_same_as_golden()
    print('회귀 시험 통과: 기준값과 동일')
