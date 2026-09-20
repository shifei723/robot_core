#!/usr/bin/env python3
"""publish_text.py — ZMQ 文本发布器，向 tts_stream 发送文本

用法:
  python3 publish_text.py                         # 发送默认课文
  python3 publish_text.py --addr tcp://host:5555  # 自定义地址
  python3 publish_text.py --delay 0.5             # 句间延迟 0.5s
  python3 publish_text.py --mode paragraph         # 整段发送（不分句）
  python3 publish_text.py --interactive            # 交互模式（手动输入）
"""

import argparse
import sys
import time

import zmq

# ─────────────────────────────────────────────────────────────
# 示例课文
# ─────────────────────────────────────────────────────────────
SAMPLE_TEXT = """
春天来了，万物复苏。小草从泥土中探出头来，嫩绿嫩绿的，像给大地铺上了一层柔软的地毯。
桃花、杏花、梨花竞相开放，红的像火，粉的像霞，白的像雪，美丽极了。
小鸟在枝头欢快地歌唱，蝴蝶在花丛中翩翩起舞。
小朋友们脱去了厚重的棉衣，换上了轻便的春装，在草地上奔跑、放风筝。
春天是一个充满希望的季节，我爱春天！
"""

SAMPLE_TEXT_LONG = """
在中国的北方，有一座古老的城市。这座城市有着三千年的历史，城墙依然保存完好。
每天清晨，老人们会在城墙根下打太极，动作缓慢而优雅，像是在和这座古城对话。
城市的中心有一条河，河水清澈见底，两岸种满了垂柳。每到春天，柳絮飘飞，如同雪花一般。
河上有一座石桥，桥上雕刻着精美的龙凤图案，据说这些图案已经存在了八百年。
桥的北边是一座寺庙，香火鼎盛。每逢节日，人们会来这里祈福，钟声悠扬，回荡在整座城市。
城市的南边是一条商业街，虽然不如从前繁华，但依然能看到许多老字号的招牌。
有一家面馆开了三代人，招牌上写着"祖传牛肉面"。每天中午，门口都会排起长队。
这座城市虽然不大，但每一个角落都有故事。走在街上，你总能感受到历史与现代的交融。
"""


def create_publisher(addr: str) -> zmq.Socket:
    """创建 ZMQ PUB socket"""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(addr)
    # 给 subscriber 一点时间连接
    time.sleep(0.3)
    return sock


def send_text(sock: zmq.Socket, text: str, delay: float, mode: str):
    """发送文本到 TTS 节点"""
    if mode == "paragraph":
        # 整段发送，让 TTS 端自己分句
        print(f"[发送] 整段 ({len(text)} 字)")
        sock.send_string(text.strip())
        time.sleep(delay)
    else:
        # 按句发送，模拟 LLM 流式输出
        sentences = split_sentences(text)
        print(f"[发送] 共 {len(sentences)} 句 (句间延迟 {delay}s)")
        for i, sent in enumerate(sentences, 1):
            sent = sent.strip()
            if not sent:
                continue
            print(f"  [{i}/{len(sentences)}] {sent}")
            sock.send_string(sent)
            time.sleep(delay)


def split_sentences(text: str) -> list[str]:
    """简单中文分句"""
    import re
    # 按中英文句号、问号、感叹号、分号、换行分割
    parts = re.split(r'([。！？；\n.!?;])', text)
    sentences = []
    for i in range(0, len(parts) - 1, 2):
        sentences.append(parts[i] + parts[i + 1])
    # 最后一段
    if parts[-1].strip():
        sentences.append(parts[-1])
    return [s for s in sentences if s.strip()]


def interactive_mode(sock: zmq.Socket):
    """交互模式：手动输入文本"""
    print("═══ 交互模式 ═══")
    print("输入文本回车发送，输入 quit 退出")
    print("─────────────────")
    while True:
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break
        if not text:
            continue
        if text.lower() in ("quit", "exit", "q"):
            sock.send_string("__EXIT__")
            print("已发送退出指令")
            break
        sock.send_string(text)
        print(f"  ✓ 已发送 ({len(text)} 字)")


def main():
    parser = argparse.ArgumentParser(description="ZMQ 文本发布器 → TTS 流水线")
    parser.add_argument("--addr", default="tcp://127.0.0.1:5555",
                        help="ZMQ 绑定地址 (默认: tcp://127.0.0.1:5555)")
    parser.add_argument("--delay", type=float, default=0.3,
                        help="句间延迟秒数 (默认: 0.3)")
    parser.add_argument("--mode", choices=["sentence", "paragraph"],
                        default="sentence",
                        help="发送模式: sentence=逐句 / paragraph=整段")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互模式 (手动输入)")
    parser.add_argument("--text", "-t", type=str, default=None,
                        help="自定义文本")
    parser.add_argument("--long", action="store_true",
                        help="使用长课文")
    parser.add_argument("--loop", action="store_true",
                        help="循环发送")
    args = parser.parse_args()

    print(f"[ZMQ] 绑定地址: {args.addr}")
    sock = create_publisher(args.addr)

    try:
        if args.interactive:
            interactive_mode(sock)
        elif args.text:
            send_text(sock, args.text, args.delay, args.mode)
            print("[完成] 文本已发送")
        else:
            text = SAMPLE_TEXT_LONG if args.long else SAMPLE_TEXT
            print(f"[课文] {len(text)} 字")
            print("──────────────────────────────")
            print(text.strip()[:100] + "..." if len(text) > 100 else text.strip())
            print("──────────────────────────────")

            while True:
                send_text(sock, text, args.delay, args.mode)
                print("[完成] 课文发送完毕")
                if not args.loop:
                    break
                print(f"[循环] {args.delay}s 后重新发送...")
                time.sleep(2.0)

            # 发送退出指令
            sock.send_string("__EXIT__")
            print("[退出] 已发送结束信号")

    except KeyboardInterrupt:
        print("\n中断")
        sock.send_string("__EXIT__")

    sock.close()


if __name__ == "__main__":
    main()
