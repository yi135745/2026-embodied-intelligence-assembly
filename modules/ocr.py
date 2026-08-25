"""任务卡 OCR 门面：OpenCV 预处理后调用视觉 API。"""

from pathlib import Path

import cv2

from modules.task2_vision import preprocess_card


class OCR:
    """保持稳定的任务卡识别接口，后续可在内部替换 OCR 实现。"""

    def __init__(self, llm):
        self.llm = llm

    def recognize_task2(self, image, output_dir) -> list:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        processed = preprocess_card(image)
        processed_path = output_dir / "task2_card_preprocessed.png"
        if not cv2.imwrite(str(processed_path), processed):
            raise RuntimeError("任务卡预处理图片保存失败：" + str(processed_path))
        return self.llm.parse_task2_card(processed_path)
