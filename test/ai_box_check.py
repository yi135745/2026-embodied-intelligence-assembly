"""独立盒子联调，无机器人/相机/API调用；默认仅GET健康检查。

python test/ai_box_check.py
python test/ai_box_check.py --action tts
python test/ai_box_check.py --action listen
python test/ai_box_check.py --action dialogue
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from modules.voice import Voice, VoiceServiceError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", choices=("health", "tts", "listen", "dialogue"), default="health")
    parser.add_argument("--url", default=config.AI_BOX_URL, help="仅本次使用的盒子HTTP地址")
    parser.add_argument("--text", default="AI盒子语音播报测试，请确认能听清楚。", help="tts播报内容")
    args = parser.parse_args()
    config.AI_BOX_URL = args.url
    try:
        voice = Voice(backend="ai_box")
        if args.action == "health":
            print(json.dumps(voice.health, ensure_ascii=False, indent=2))
            print("仅资源自检通过；麦克风、识别、扬声器尚需单独测试。")
        elif args.action == "tts":
            voice.speak(args.text)
            print("服务返回播报完成，请人工确认盒子扬声器是否确实出声。")
        elif args.action == "listen":
            print("请直接对盒子麦克风说话，不需要先喊原生唤醒词。", flush=True)
            print("识别结果：" + (voice.listen() or "（未检测到语音）"))
        else:
            print("仅测试语音，绝不执行任务或移动机械臂。", flush=True)
            voice.speak("语音联调，请呼叫" + config.WAKE_WORD)
            if not voice.wake(config.WAKE_WORD):
                print("收到退出指令。")
                return 0
            voice.speak(config.READY_REPLY)
            print("请说任务一、任务二或退出系统。", flush=True)
            command = voice.listen()
            print("识别结果：" + (command or "（未检测到语音）"))
            if command:
                voice.speak("识别到" + command + "。本次仅为语音测试，不执行任务。")
    except VoiceServiceError as exc:
        print("语音联调失败：" + str(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("已退出客户端；盒子端请求可能仍在运行，请等待本轮录音/播报结束。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
