"""大模型模块：DashScope Qwen 视觉理解。

对外接口：
    LLM.identify_image(image) -> str  识别图片中任务卡内容，返回可直接播报的文本。
"""

import base64
import io
import json
import mimetypes
from pathlib import Path
from socket import timeout as SOCKET_TIMEOUT
from urllib import error, request

import config


def _guess_image_mime(image_bytes: bytes, image_name: str = "") -> str:
    """先根据文件头判断图片类型；不确定时再根据文件名猜测。"""
    if image_bytes.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if image_bytes.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if image_bytes.startswith(b"BM"):
        return "image/bmp"
    return mimetypes.guess_type(image_name)[0] or "image/jpeg"


def _image_file_to_data_url(image_path: Path, max_edge: int, jpeg_quality: int) -> str:
    """把本地图片转成 base64 data URL；工业相机原图较大，先缩放再上传可明显减少耗时。"""
    if not image_path.exists():
        raise RuntimeError("图片不存在：" + str(image_path))
    image_bytes = image_path.read_bytes()
    original_bytes = image_bytes
    mime_type = _guess_image_mime(image_bytes, image_path.name)
    original_size_text = "未知尺寸"
    final_size_text = "未知尺寸"

    try:
        from PIL import Image

        with Image.open(io.BytesIO(image_bytes)) as image:
            original_size_text = "%dx%d" % (image.width, image.height)
            final_image = image
            longest_edge = max(image.width, image.height)
            if max_edge > 0 and longest_edge > max_edge:
                scale = max_edge / float(longest_edge)
                new_size = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
                final_image = image.resize(new_size, Image.LANCZOS)

            if final_image.mode not in ("RGB", "L"):
                final_image = final_image.convert("RGB")

            buffer = io.BytesIO()
            final_image.save(buffer, format="JPEG", quality=max(1, min(95, jpeg_quality)), optimize=True)
            image_bytes = buffer.getvalue()
            mime_type = "image/jpeg"
            final_size_text = "%dx%d" % (final_image.width, final_image.height)
    except ImportError:
        print("提示：未安装 Pillow，跳过图片压缩。建议执行 pip install pillow 后重试。")
    except Exception as exc:
        print("提示：图片压缩失败，改用原图上传：" + str(exc))

    print(
        "模型输入图片：原图 %s，%.1f KB；上传 %s，%.1f KB"
        % (original_size_text, len(original_bytes) / 1024.0, final_size_text, len(image_bytes) / 1024.0)
    )
    image_base64 = base64.b64encode(image_bytes).decode("utf-8")
    return "data:%s;base64,%s" % (mime_type, image_base64)


def _format_task_card_result(text: str) -> str:
    """统一输出和播报格式，避免模型返回「图片中有...」等不同措辞。"""
    cleaned = text.strip().rstrip("。")
    if cleaned.startswith("任务卡1中有"):
        objects_text = cleaned.removeprefix("任务卡1中有").removesuffix("物体")
        return "任务卡1中有" + objects_text + "。"
    if cleaned.startswith("图片中有"):
        objects_text = cleaned.removeprefix("图片中有").removesuffix("物体")
        return "任务卡1中有" + objects_text + "。"
    return "任务卡1中有" + cleaned.removesuffix("物体") + "。"


class LLM:
    """封装 DashScope Qwen 视觉大模型识别能力。"""

    def __init__(self):
        self.model = config.MODEL
        self.api_url = config.API_URL
        self.api_timeout = config.API_TIMEOUT
        self.image_max_edge = config.IMAGE_MAX_EDGE
        self.jpeg_quality = config.JPEG_QUALITY
        self.prompt = config.PROMPT_TEXT

    def identify_image(self, image) -> str:
        """识别图片中任务卡内容，返回格式化后的播报文本。失败抛 RuntimeError。"""
        image_path = image if isinstance(image, Path) else Path(image)

        api_key = config.DASHSCOPE_API_KEY
        if not api_key:
            raise RuntimeError("请先在 config.py 中配置 DASHSCOPE_API_KEY。")

        data_url = _image_file_to_data_url(image_path, self.image_max_edge, self.jpeg_quality)
        print("大模型接口：" + self.api_url)
        print("大模型名称：" + self.model)

        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.prompt},
                        {"type": "image_url", "image_url": {"url": data_url}},
                    ],
                }
            ],
        }
        body = json.dumps(payload).encode("utf-8")
        api_request = request.Request(
            self.api_url,
            data=body,
            method="POST",
            headers={
                "Authorization": "Bearer " + api_key,
                "Content-Type": "application/json",
            },
        )

        try:
            with request.urlopen(api_request, timeout=self.api_timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
        except error.HTTPError as exc:
            message = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError("模型接口调用失败：HTTP %d %s" % (exc.code, message)) from exc
        except error.URLError as exc:
            if isinstance(exc.reason, (TimeoutError, SOCKET_TIMEOUT)):
                raise RuntimeError(
                    "模型接口调用超时：当前超时设置 %.1f 秒，请检查网络或缩短图片大小。" % self.api_timeout
                ) from exc
            raise RuntimeError("模型接口调用失败：" + repr(exc.reason)) from exc
        except SOCKET_TIMEOUT as exc:
            raise RuntimeError(
                "模型接口调用超时：当前超时设置 %.1f 秒，请检查网络或缩短图片大小。" % self.api_timeout
            ) from exc

        choices = result.get("choices", [])
        if choices:
            text = choices[0].get("message", {}).get("content")
            if isinstance(text, str) and text.strip():
                return _format_task_card_result(text.strip())
        raise RuntimeError("模型没有返回可读取的文本结果。")

    def parse_task2_card(self, image, output_dir=None) -> list:
        """通过视觉 API 识别任务卡2，返回校验后的六步装配指令。"""
        image_path = image if isinstance(image, Path) else Path(image)
        api_key = config.DASHSCOPE_API_KEY
        if not api_key:
            raise RuntimeError("请先在 config.py 中配置 DASHSCOPE_API_KEY。")
        data_url = _image_file_to_data_url(
            image_path, config.TASK2_CARD_IMAGE_MAX_EDGE, config.TASK2_CARD_JPEG_QUALITY
        )
        payload = {"model": self.model, "messages": [{"role": "user", "content": [
            {"type": "text", "text": config.TASK2_CARD_PROMPT},
            {"type": "image_url", "image_url": {"url": data_url}},
        ]}], "temperature": 0}
        api_request = request.Request(self.api_url, data=json.dumps(payload).encode("utf-8"), method="POST", headers={"Authorization": "Bearer " + api_key, "Content-Type": "application/json"})
        try:
            with request.urlopen(api_request, timeout=self.api_timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
        except Exception as exc:
            raise RuntimeError("任务卡2 API 识别失败：" + str(exc)) from exc
        text = result.get("choices", [{}])[0].get("message", {}).get("content", "")
        raw_text = text.strip()
        log_path = None
        if output_dir is not None:
            log_path = Path(output_dir) / ("task2_llm_%s.txt" % __import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S"))
            log_path.write_text(
                "timestamp: %s\nmodel: %s\n\n[RAW_OUTPUT]\n%s\n"
                % (__import__("datetime").datetime.now().isoformat(timespec="seconds"), self.model, raw_text),
                encoding="utf-8",
            )
        text = raw_text.removeprefix("```json").removesuffix("```").strip()
        try:
            steps = json.loads(text)
        except json.JSONDecodeError as exc:
            raise RuntimeError("任务卡2 API 未返回合法 JSON：" + text) from exc
        colors = set(config.TASK2_HSV_RANGES)
        if not isinstance(steps, list) or len(steps) != 6:
            raise RuntimeError("任务卡2必须解析为6步。")
        normalized = []
        for index, item in enumerate(steps, 1):
            block, tray = item.get("block_color"), item.get("tray_color")
            if block not in colors or tray not in colors:
                raise RuntimeError("任务卡2第%d步颜色无效。" % index)
            normalized.append({"step": index, "block_color": block, "tray_color": tray})
        if {x["block_color"] for x in normalized} != colors:
            raise RuntimeError("六步指令没有完整覆盖六种方块颜色。")
        if {x["tray_color"] for x in normalized} != colors:
            raise RuntimeError("六步指令没有完整覆盖六种托盘颜色。")
        if log_path is not None:
            with log_path.open("a", encoding="utf-8") as stream:
                stream.write("\n[VALIDATED_STEPS]\n")
                stream.write(json.dumps(normalized, ensure_ascii=False, indent=2))
                stream.write("\n")
        return normalized
