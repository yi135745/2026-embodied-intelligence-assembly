import requests
import base64
import json


API_KEY = "sk-ws-H.EYHLPXH.5uKG.MEUCIQDE2kTCCAta9ejK0FGoO9fXxECCx-hW5Gt5CszRP_-83AIgTFEgMp69iegDHhLrZ6O7DILC7z64SJBhugbFq57ZCws"  # 千问大模型密钥（用环境变量，不要写进这里）

# 注意：这里写完整接口
URL = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions"

MODEL = "qwen-vl-max"

IMAGE_PATH = r"E:\project_test\project_all\test\test.jpg"


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