"""相机能力封装；只负责得到图片，不承担识别或任务流程。"""

from pathlib import Path

import config
from drivers.hik_mvs import capture_hik_mvs_image


class Camera:
    """单帧相机采集功能库。"""

    def __init__(self):
        self.mvs_sdk_dir = Path(config.MVS_SDK_DIR)
        self.device_index = config.MVS_DEVICE_INDEX
        self.camera_ip = config.CAMERA_IP
        self.capture_image = Path(config.MVS_CAPTURE_NAME)
        self.local_test_image = Path(config.LOCAL_TEST_IMAGE_NAME)
        self.debug_image = Path(config.DEBUG_IMAGE) if config.DEBUG_IMAGE else None

    def capture(self, output_name=None, debug_image=None, exposure_time=None, gain=None) -> Path:
        selected_debug = Path(debug_image) if debug_image else self.debug_image
        if selected_debug is not None:
            image_path = selected_debug
            print("调试模式：使用本地图片代替相机拍照：" + str(image_path))
        elif output_name is None and self.local_test_image.exists():
            image_path = self.local_test_image
            print("检测到本地任务卡图片，直接读取硬盘文件：" + str(image_path))
        else:
            image_path = capture_hik_mvs_image(
                mvs_sdk_dir=self.mvs_sdk_dir,
                device_index=self.device_index,
                camera_ip=self.camera_ip,
                output_path=Path(output_name) if output_name else self.capture_image,
                timeout_ms=config.MVS_TIMEOUT_MS,
                exposure_time=config.MVS_EXPOSURE_TIME if exposure_time is None else exposure_time,
                gain=config.MVS_GAIN if gain is None else gain,
            )
        return image_path
