import base64
import json
import sys
from pathlib import Path

import requests

# 直接运行本文件时，将项目根目录加入模块搜索路径，复用 config 的密钥读取逻辑。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import config  # noqa: E402

API_KEY = config.DASHSCOPE_API_KEY  # 从 secrets.json / 环境变量读取，与正式代码一致

# 注意：这里写完整接口
URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"

MODEL = "qwen-vl-max"

IMAGE_PATH = Path(__file__).parent / "test.jpg"


def image_to_base64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def main():

    image_base64 = image_to_base64(IMAGE_PATH)

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }

    data = {
        "model": MODEL,
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "请识别图片中的主要物体，只输出物体名称。"
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64}"
                        }
                    }
                ]
            }
        ]
    }

    response = requests.post(
        URL,
        headers=headers,
        json=data,
        timeout=60
    )

    print("状态码:", response.status_code)

    if response.status_code != 200:
        print(response.text)
        return

    result = response.json()

    print("模型返回:")
    print(
        result["choices"][0]["message"]["content"]
    )


if __name__ == "__main__":
    main()