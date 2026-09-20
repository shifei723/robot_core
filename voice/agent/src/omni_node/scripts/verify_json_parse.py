#!/usr/bin/env python3
"""verify_json_parse.py — 用实机观测到的真实模型输出验证 JSON 解析容错"""
import sys

sys.path.insert(0, "/data/sf_code/agent_ws/src/omni_node/scripts")
from tool_agent import extract_json  # noqa: E402

CASES = [
    # 实机日志中真实出现过的输出
    ("真实-残缺缺}", '{"tasks":[{"action":"nod"}]', True),
    ("真实-完整", '{"tasks":[{"action":"navigate","target":"kitchen"}]}', True),
    # 其他可能的截断形态
    ("缺]和}", '{"tasks":[{"action":"nod"}', True),
    ("缺多层", '{"tasks":[{"action":"nod"', True),
    ("多任务残缺", '{"tasks":[{"action":"navigate","target":"kitchen"},{"action":"nod"}]', True),
    ("带前后文残缺", '好的{"tasks":[{"action":"shake"}]', True),
    # 不应误判
    ("普通聊天", "杭州位于中国东部，浙江省省会。", False),
    ("聊天带括号", "这个函数写成 f(x) 就可以了。", False),
    ("空tasks", '{"tasks":[]}', False),
]

ok = 0
for name, text, should_hit in CASES:
    r = extract_json(text)
    hit = bool(r and r.get("tasks"))
    passed = hit == should_hit
    ok += passed
    n = len(r["tasks"]) if hit else 0
    mark = "PASS" if passed else "FAIL"
    exp = "应命中" if should_hit else "不应命中"
    got = f"命中{n}个任务" if hit else "未命中"
    print(f"[{mark}] {name:14s} {exp:6s} 实际={got}")

print(f"\n通过 {ok}/{len(CASES)}")
sys.exit(0 if ok == len(CASES) else 1)
