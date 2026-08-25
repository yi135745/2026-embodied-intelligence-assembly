"""视觉模块：海康 MVS 工业相机采集。

对外接口：
    Vision.capture() -> Path  单次采集图像，返回图片文件路径。

采集优先级：debug 图 > 本地测试图（存在时读文件）> 海康 MVS 实机拍照。
"""

import importlib
import os
import sys
from ctypes import POINTER, byref, cast, memset, sizeof
from pathlib import Path
from typing import Any

import config


def _format_mvs_error(ret: int) -> str:
    """MVS SDK 返回整数错误码，统一格式化成 0xXXXXXXXX 便于查手册。"""
    return "0x%08x" % (ret & 0xFFFFFFFF)


def _decode_mvs_text(values) -> str:
    """MVS 设备名/IP 等字段通常是以 0 结尾的 C 字符数组。"""
    chars = []
    for value in values:
        if value == 0:
            break
        chars.append(chr(value))
    return "".join(chars)


def _get_mvs_device_ip(device_info) -> str:
    """GigE 相机 IP 在 SDK 中是 32 位整数，转成点分十进制字符串。"""
    ip_value = device_info.SpecialInfo.stGigEInfo.nCurrentIp
    return ".".join(
        [
            str((ip_value & 0xFF000000) >> 24),
            str((ip_value & 0x00FF0000) >> 16),
            str((ip_value & 0x0000FF00) >> 8),
            str(ip_value & 0x000000FF),
        ]
    )


def _load_mvs_sdk(mvs_sdk_dir: Path) -> Any:
    """加载海康 MVS Python 示例中的 MvImport 封装，并把运行库目录加入 PATH。"""
    mv_import_dir = mvs_sdk_dir / "MvImport"
    if not mv_import_dir.exists():
        raise SystemExit("没有找到海康 MVS Python 封装目录：" + str(mv_import_dir))
    for dll_dir in [
        Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"),
        Path(r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win32_i86"),
    ]:
        if dll_dir.exists():
            os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")
            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(dll_dir))
    sys.path.insert(0, str(mv_import_dir))
    try:
        return importlib.import_module("MvCameraControl_class")
    except (ImportError, OSError) as exc:
        raise SystemExit("加载海康 MVS Python SDK 失败：" + str(exc)) from exc


def _capture_hik_mvs_image(
    mvs_sdk_dir: Path,
    device_index: int,
    camera_ip: str,
    output_path: Path,
    timeout_ms: int,
    exposure_time: float,
    gain: float,
) -> Path:
    """完整拍照流程：枚举设备 -> 选择相机 -> 打开 -> 取一帧 -> 保存 jpg -> 释放资源。"""
    sdk = _load_mvs_sdk(mvs_sdk_dir)
    mv_camera_class = sdk.MvCamera
    device_info_list_class = sdk.MV_CC_DEVICE_INFO_LIST
    device_info_class = sdk.MV_CC_DEVICE_INFO
    frame_out_class = sdk.MV_FRAME_OUT
    save_image_param_class = sdk.MV_SAVE_IMG_TO_FILE_PARAM
    gige_device = sdk.MV_GIGE_DEVICE
    usb_device = sdk.MV_USB_DEVICE
    access_exclusive = sdk.MV_ACCESS_Exclusive
    trigger_mode_off = sdk.MV_TRIGGER_MODE_OFF
    image_jpeg = sdk.MV_Image_Jpeg

    device_list = device_info_list_class()
    ret = mv_camera_class.MV_CC_EnumDevices(gige_device | usb_device, device_list)
    if ret != 0:
        raise SystemExit("枚举海康相机失败：" + _format_mvs_error(ret))
    if device_list.nDeviceNum == 0:
        raise SystemExit("没有找到海康工业相机。请检查网线、交换机、相机 IP、MVS 客户端是否能预览。")

    selected_index = device_index
    print("找到海康相机数量：" + str(device_list.nDeviceNum))
    for index in range(device_list.nDeviceNum):
        device_info = cast(device_list.pDeviceInfo[index], POINTER(device_info_class)).contents
        if device_info.nTLayerType == gige_device:
            model_name = _decode_mvs_text(device_info.SpecialInfo.stGigEInfo.chModelName)
            current_ip = _get_mvs_device_ip(device_info)
            print("[%d] GigE 相机：%s，IP：%s" % (index, model_name, current_ip))
            if camera_ip and current_ip == camera_ip:
                selected_index = index
        elif device_info.nTLayerType == usb_device:
            model_name = _decode_mvs_text(device_info.SpecialInfo.stUsb3VInfo.chModelName)
            print("[%d] USB 相机：%s" % (index, model_name))

    if selected_index < 0 or selected_index >= device_list.nDeviceNum:
        raise SystemExit("相机索引无效：" + str(selected_index))

    cam = mv_camera_class()
    st_device = cast(device_list.pDeviceInfo[selected_index], POINTER(device_info_class)).contents
    ret = cam.MV_CC_CreateHandle(st_device)
    if ret != 0:
        raise SystemExit("创建相机句柄失败：" + _format_mvs_error(ret))

    is_open = False
    is_grabbing = False
    try:
        ret = cam.MV_CC_OpenDevice(access_exclusive, 0)
        if ret != 0:
            raise SystemExit(
                "打开相机失败：" + _format_mvs_error(ret) + "。请确认 MVS 或 Vision Master 没有占用相机。"
            )
        is_open = True

        if st_device.nTLayerType == gige_device:
            packet_size = cam.MV_CC_GetOptimalPacketSize()
            if int(packet_size) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size)
                if ret != 0:
                    print("警告：设置 GigE 最佳包大小失败：" + _format_mvs_error(ret))

        # 每次只需要一张图，关闭硬触发，连续取流后抓一帧。
        ret = cam.MV_CC_SetEnumValue("TriggerMode", trigger_mode_off)
        if ret != 0:
            raise SystemExit("设置相机触发模式失败：" + _format_mvs_error(ret))

        if exposure_time > 0:
            ret = cam.MV_CC_SetFloatValue("ExposureTime", exposure_time)
            if ret != 0:
                print("警告：设置曝光时间失败：" + _format_mvs_error(ret))
        if gain >= 0:
            ret = cam.MV_CC_SetFloatValue("Gain", gain)
            if ret != 0:
                print("警告：设置增益失败：" + _format_mvs_error(ret))

        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise SystemExit("开始取流失败：" + _format_mvs_error(ret))
        is_grabbing = True

        frame = frame_out_class()
        memset(byref(frame), 0, sizeof(frame))
        ret = cam.MV_CC_GetImageBuffer(frame, timeout_ms)
        if ret != 0 or not frame.pBufAddr:
            raise SystemExit("获取相机图片失败：" + _format_mvs_error(ret))

        try:
            output_path = output_path.expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)

            save_param = save_image_param_class()
            memset(byref(save_param), 0, sizeof(save_param))
            save_param.enPixelType = frame.stFrameInfo.enPixelType
            save_param.pData = frame.pBufAddr
            save_param.nDataLen = frame.stFrameInfo.nFrameLen
            save_param.nWidth = frame.stFrameInfo.nWidth
            save_param.nHeight = frame.stFrameInfo.nHeight
            save_param.enImageType = image_jpeg
            save_param.nQuality = 90
            save_param.iMethodValue = 1
            image_path_bytes = str(output_path).encode("mbcs")
            if len(image_path_bytes) >= 256:
                raise SystemExit("MVS 保存图片路径过长，请换一个更短的路径：" + str(output_path))
            save_param.pImagePath = image_path_bytes

            ret = cam.MV_CC_SaveImageToFile(save_param)
            if ret != 0:
                raise SystemExit("保存相机图片失败：" + _format_mvs_error(ret))

            print(
                "相机抓图成功：" + str(output_path)
                + "，宽 %d，高 %d" % (frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight)
            )
            return output_path
        finally:
            cam.MV_CC_FreeImageBuffer(frame)
    finally:
        if is_grabbing:
            cam.MV_CC_StopGrabbing()
        if is_open:
            cam.MV_CC_CloseDevice()
        cam.MV_CC_DestroyHandle()


class Vision:
    """封装海康 MVS 相机采集能力。"""

    def __init__(self):
        self.mvs_sdk_dir = Path(config.MVS_SDK_DIR)
        self.device_index = config.MVS_DEVICE_INDEX
        self.camera_ip = config.CAMERA_IP
        self.capture_image = Path(config.MVS_CAPTURE_NAME)
        self.local_test_image = Path(config.LOCAL_TEST_IMAGE_NAME)
        self.debug_image = Path(config.DEBUG_IMAGE) if config.DEBUG_IMAGE else None

    def capture(self, output_name=None, debug_image=None, exposure_time=None, gain=None) -> Path:
        """单次采集，返回图片文件路径。

        优先级：debug 图 > 本地测试图 > 海康 MVS 实机拍照。
        """
        selected_debug = Path(debug_image) if debug_image else self.debug_image
        if selected_debug is not None:
            image_path = selected_debug
            print("调试模式：使用本地图片代替相机拍照：" + str(image_path))
        elif output_name is None and self.local_test_image.exists():
            image_path = self.local_test_image
            print("检测到本地任务卡图片，直接读取硬盘文件：" + str(image_path))
        else:
            image_path = _capture_hik_mvs_image(
                mvs_sdk_dir=self.mvs_sdk_dir,
                device_index=self.device_index,
                camera_ip=self.camera_ip,
                output_path=Path(output_name) if output_name else self.capture_image,
                timeout_ms=config.MVS_TIMEOUT_MS,
                exposure_time=config.MVS_EXPOSURE_TIME if exposure_time is None else exposure_time,
                gain=config.MVS_GAIN if gain is None else gain,
            )
        return image_path
