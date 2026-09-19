# SpeechReleaseServer HTTP/WS 接口协议

面向上位机 / 业务编排 / `SpeechReleaseClient` 联调。  
适用：源码服务与闭源 `SpeechReleaseServer`（协议相同）。

- 默认 Base URL：`http://<host>:8765`
- 编码：请求/响应均为 **UTF-8 JSON**（`Content-Type: application/json; charset=utf-8`）
- 鉴权：当前版本 **无鉴权**（仅建议内网使用）
- 并发：`/v1/asr` 在服务端有捕获锁，同时仅一路录音；HTTP 服务本身为多线程

---

## 1. 端点一览

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/health` | 就绪与模型自检 |
| POST | `/v1/asr` | 唤醒（可选）+ VAD 录音 + ASR |
| POST | `/v1/tts` | TTS 合成并播放（同步，播完/打断后返回） |
| POST | `/v1/tts/cancel` | 取消当前播报 |
| GET→WS | `/v1/events` | WebSocket 事件流 |

未知路径返回 HTTP `404`，body 示例：

```json
{"ok": false, "error_code": "NOT_FOUND", "message": "not found"}
```

业务失败多数仍返回 **HTTP 200**，以 body 中 `ok` / `error_code` 判定。

---

## 2. 通用约定

### 2.1 成功/失败字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ok` | bool | 是否成功 |
| `error_code` | string | 成功一般为 `OK`；失败为错误码 |
| `message` | string | 人类可读说明 |
| `result_digest` | object | 结构化结果摘要（asr/tts 常用） |
| `retryable` | bool | （部分失败）是否可重试 |

### 2.2 请求体

- POST 可无 body（按空对象 `{}` 处理）
- JSON 非法时按 `{}` 处理（不单独返回 400）
- 客户端常用可选字段 `request_id`（UUID 字符串）便于日志关联；**服务端当前可不回显**

### 2.3 超时建议

| 调用 | 建议客户端超时 |
| --- | --- |
| `/health` | 3–5 s |
| `/v1/asr` | ≥ 60 s（含等待唤醒 + 录音） |
| `/v1/tts` | ≥ 120 s（长句播报） |
| `/v1/tts/cancel` | 5–10 s |

---

## 3. `GET /health`

轻量自检：只检查模型路径是否存在，**不**初始化重型 ASR/TTS 运行时（`capture_ready=lazy`）。

### 请求

无 body。

```bash
curl -s http://127.0.0.1:8765/health | python3 -m json.tool
```

### 响应（就绪）

```json
{
  "ready": true,
  "service": "arm_speech_service",
  "version": "m28-4-real",
  "provider": "real",
  "tts_default_backend": "piper",
  "tts_backends": [
    {"name": "piper", "mode": "offline", "aliases": ["piper", "offline", "local"]},
    {"name": "edge_tts", "mode": "online", "aliases": ["edge_tts", "edge", "online", "cloud"]}
  ],
  "capture_ready": "lazy",
  "models": {
    "asr_model_dir": ".../MODELS/asr",
    "asr_exists": true,
    "keyword_model": ".../wakeup_xiaoyitx.table",
    "keyword_exists": true,
    "wakeup_keyword": "小E同学",
    "wakeup_aliases": ["小E同学", "小艺同学", "..."],
    "vad_model": ".../silero_vad.onnx",
    "vad_exists": true,
    "piper_model": ".../MODELS/tts/zh/....onnx",
    "piper_exists": true
  }
}
```

### 字段说明

| 字段 | 说明 |
| --- | --- |
| `ready` | `asr_exists && keyword_exists && vad_exists`；**不含** piper（piper 仅影响离线 TTS） |
| `provider` | 固定 `real` |
| `tts_default_backend` | 当前默认 TTS（来自配置/环境） |
| `models.wakeup_keyword` | 提示用户说的唤醒词文案 |
| `models.wakeup_aliases` | ASR 剥前缀/判“仅唤醒词”用的别名 |

### 未就绪示例

```json
{
  "ready": false,
  "error_code": "ASR_MODEL_MISSING",
  "message": "missing assets: asr,keyword",
  "...": "其余字段仍可能返回"
}
```

---

## 4. `POST /v1/asr`

阻塞调用：按策略等待唤醒（可选）→ VAD 录音 → ASR → 返回文本。

### 请求体

```json
{
  "request_id": "可选-UUID",
  "mode": "live_capture",
  "wakeup_required": true,
  "prewoken": false,
  "language": "zh-CN",
  "start_timeout_s": 5.0,
  "max_record_seconds": 10.0,
  "vad_threshold": 0.5,
  "wakeup_timeout_s": 30.0
}
```

| 字段 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `wakeup_required` | bool | `true` | `true`：先等唤醒词再录音 |
| `prewoken` | bool | `false` | `true` 或 `wakeup_required=false`：视为已唤醒，直接录音（打断后二次采集） |
| `mode` | string | — | 客户端习惯传 `live_capture`；服务端以 live 采集为准 |
| `language` | string | — | 预留/透传，当前实现以服务端配置为主 |
| `start_timeout_s` | float | 服务端配置 | 开口超时（秒）；`<=0` 或非法则忽略 |
| `max_record_seconds` | float | 服务端配置 | 最长录音（秒） |
| `vad_threshold` | float | 服务端配置 | VAD 阈值，合法范围会被夹到 `[0.05, 0.95]` |
| `wakeup_timeout_s` | float | 服务端配置 | 等待唤醒超时（秒） |

**预唤醒判定**（服务端）：

```text
prewoken = bool(payload.prewoken) OR NOT bool(payload.wakeup_required, default=true)
```

### 成功响应

```json
{
  "ok": true,
  "error_code": "OK",
  "message": "asr ok",
  "result_digest": {
    "tool_name": "asr",
    "summary": "asr ok",
    "instruction": "今天天气怎么样",
    "asr_source": "arm_speech_service",
    "arm_service_provider": "real",
    "input_mode": "live_capture",
    "wakeup_source": "azure_keyword",
    "vad_threshold": 0.5,
    "sample_rate": 16000,
    "record_seconds": 1.25,
    "start_timeout_s": 5.0,
    "max_record_seconds": 10.0,
    "wakeup_timeout_s": 30.0
  }
}
```

| `result_digest` 字段 | 说明 |
| --- | --- |
| `instruction` | ASR 文本（业务主结果） |
| `wakeup_source` | 如 `azure_keyword`；prewoken 场景可能为空或其它标记 |
| `record_seconds` | 有效录音时长 |

### 失败响应

```json
{
  "ok": false,
  "error_code": "NO_SPEECH_DETECTED",
  "message": "...",
  "retryable": true,
  "result_digest": {
    "tool_name": "asr",
    "summary": "...",
    "asr_source": "arm_speech_service",
    "arm_service_provider": "real",
    "input_mode": "live_capture"
  }
}
```

常见 `error_code`：

| 码 | 含义 | 通常 retryable |
| --- | --- | --- |
| `WAKEUP_TIMEOUT` | 等待唤醒超时 | true |
| `NO_SPEECH_DETECTED` | 未检测到有效语音 | true |
| `ASR_TIMEOUT` | ASR/采集超时类 | true |
| `SPEECH_REAL_CAPTURE_FAILED` | 其它采集失败 | false（视实现） |
| `SPEECH_SERVICE_NOT_READY` / `ASR_MODEL_MISSING` | 模型未就绪 | false |

成功时可能推送 WS 事件 `asr.final`；失败推送 `asr.failed`。采集过程中检测到唤醒还会推送 `wakeup.detected`。

---

## 5. `POST /v1/tts`

阻塞调用：合成并播放，播完、跳过或被打断后返回。

### 请求体

```json
{
  "request_id": "可选-UUID",
  "text": "要播报的文本",
  "backend": "piper",
  "voice": "default",
  "interrupt_policy": "cancel_on_wakeup",
  "context": {}
}
```

| 字段 | 类型 | 默认 | 说明 |
| --- | --- | --- | --- |
| `text` | string | `""` | 播报文本（空文本可能导致跳过/失败，取决于后端） |
| `backend` | string | `piper` | 见下方别名表 |
| `voice` | string | `default` | `edge_tts` 常用具体音色，如 `zh-CN-XiaoxiaoNeural`；piper 可用 `default` |
| `interrupt_policy` | string | — | 客户端可传 `cancel_on_wakeup`；打断依赖服务端唤醒监听 + `/v1/tts/cancel` |
| `context` | object | `{}` | 透传上下文（可选） |

#### backend 别名

| 传入值 | 归一化为 |
| --- | --- |
| `piper` / `offline` / `local` / 空 | `piper`（离线） |
| `edge_tts` / `edge` / `online` / `cloud` | `edge_tts`（在线，需外网） |

### 成功响应（示例）

```json
{
  "ok": true,
  "error_code": "OK",
  "message": "tts ok",
  "result_digest": {
    "tool_name": "tts",
    "summary": "tts ok",
    "spoken_text": "要播报的文本",
    "tts_source": "arm_speech_service",
    "arm_service_provider": "real",
    "tts_backend": "piper",
    "tts_voice": "default",
    "tts_format": "wav",
    "tts_cache_path": "",
    "interrupted": false
  }
}
```

| `result_digest` 字段 | 说明 |
| --- | --- |
| `spoken_text` | 实际播报文本 |
| `tts_backend` / `tts_voice` | 实际后端与音色 |
| `interrupted` | 是否被打断取消 |
| `tts_skipped` | 是否跳过播放（运行时不可用等） |
| `cancel_source` | 打断/取消来源（若有） |
| `duration_ms` | 播放时长（部分成功路径） |

### 生命周期事件（WS）

| 事件 | 时机 |
| --- | --- |
| `tts.started` | 开始合成/播放前 |
| `tts.finished` | 正常播完 |
| `tts.cancelled` | 失败或被打断 |
| `tts.skipped` | 跳过播放 |

---

## 6. `POST /v1/tts/cancel`

请求取消当前 TTS 播放。

### 请求体

```json
{
  "request_id": "可选-UUID",
  "cancel_source": "pc_client"
}
```

`cancel_source` 为客户端标注；服务端事件里取消来源可能记为 `pc_manual`。

### 响应

```json
{
  "ok": true,
  "error_code": "OK",
  "message": "tts cancel accepted",
  "result_digest": {
    "tool_name": "tts",
    "summary": "tts cancel accepted",
    "tts_source": "arm_speech_service",
    "arm_service_provider": "real"
  }
}
```

无正在播放时也可能 `ok=true`（message 可能为 `tts cancel requested`）。会推送 WS `tts.cancelled`。

---

## 7. `WS /v1/events`

HTTP `GET /v1/events` 升级为 WebSocket（RFC6455），服务端持续下发 **JSON 文本帧**。

### 握手

客户端需带标准 Upgrade 头（至少 `Sec-WebSocket-Key`）。缺 key 时返回 HTTP `400`。

URL：

- `ws://<host>:8765/v1/events`
- 若 Base 为 https，则用 `wss://...`（当前默认部署为明文 http/ws）

### 订阅语义

- 新连接只接收 **订阅之后** 产生的事件（避免旧 `wakeup.detected` 误取消新 TTS）
- 连接建立时服务端会启动唤醒监听；连接结束时停止监听
- 可多连接；事件按产生顺序推送

### 事件公共字段

| 字段 | 说明 |
| --- | --- |
| `event` | 事件名 |
| `session_id` | 服务进程内会话 ID |
| `timestamp` | UTC ISO 风格时间戳，如 `2026-07-24T04:00:00Z` |

### 事件类型

#### `wakeup.detected`

```json
{
  "event": "wakeup.detected",
  "session_id": "...",
  "wakeup_source": "azure_keyword",
  "timestamp": "..."
}
```

触发：独立唤醒监听，或 `/v1/asr` 采集过程中识别到唤醒词。

上位机典型用法：TTS 播放期间订阅该事件 → 调用 `POST /v1/tts/cancel` 实现打断。

#### `asr.final`

```json
{
  "event": "asr.final",
  "session_id": "...",
  "text": "今天天气怎么样",
  "input_mode": "live_capture",
  "record_seconds": 1.25,
  "sample_rate": 16000,
  "timestamp": "..."
}
```

#### `asr.failed`

```json
{
  "event": "asr.failed",
  "session_id": "...",
  "error_code": "NO_SPEECH_DETECTED",
  "message": "...",
  "retryable": true,
  "timestamp": "..."
}
```

#### `tts.started` / `tts.finished` / `tts.cancelled` / `tts.skipped`

见第 5 节；`tts.cancelled` 可能含 `cancel_source`、`interrupted`；`tts.skipped` 可能含 `skip_code`、`skip_reason`；`tts.finished` 可能含 `duration_ms`。

---

## 8. 推荐集成流程

### 8.1 启动自检

1. 启动 `SpeechReleaseServer`
2. `GET /health` 直至 `ready=true`
3. （可选）`POST /v1/tts` 播一句短文本

### 8.2 唤醒 + 指令

```text
POST /v1/asr  { "wakeup_required": true }
→ result_digest.instruction
```

### 8.3 播报中打断 + 二次指令

```text
1) 连接 WS /v1/events
2) POST /v1/tts  { "text": "...", "backend": "piper" }   # 可放后台线程
3) 收到 wakeup.detected → POST /v1/tts/cancel
4) POST /v1/asr  { "wakeup_required": false, "prewoken": true }
→ 第二句 instruction
```

参考实现：发布包 `SpeechReleaseClient/scripts/verify_wakeup_asr_tts_interrupt.py`。

---

## 9. curl / 脚本示例

### TTS

```bash
curl -s http://127.0.0.1:8765/v1/tts \
  -H 'Content-Type: application/json' \
  -d '{"text":"语音播报测试","backend":"piper","voice":"default"}' \
  | python3 -m json.tool
```

### ASR（需麦克风，先说唤醒词再说指令）

```bash
curl -s http://127.0.0.1:8765/v1/asr \
  -H 'Content-Type: application/json' \
  -d '{"wakeup_required":true,"prewoken":false}' \
  | python3 -m json.tool
```

### 取消 TTS

```bash
curl -s http://127.0.0.1:8765/v1/tts/cancel \
  -H 'Content-Type: application/json' \
  -d '{"cancel_source":"manual"}' \
  | python3 -m json.tool
```

### 使用包内客户端

```bash
cd /path/to/SpeechRelease-*/SpeechReleaseClient
source ./准备验证环境.sh
python3 verify_tts_backends.py --backend piper
```

---

## 10. 版本与边界

| 项 | 说明 |
| --- | --- |
| 协议版本标识 | `/health.version` 当前为 `m28-4-real` |
| 传输音频 | **不**通过 HTTP 上传/下载 PCM；录音与播放均在服务端本机完成 |
| 职责 | 唤醒 / VAD / 录音 / ASR / TTS / 事件 |
| 非职责 | 业务编排、机械臂控制、证据落盘、鉴权 |

配置项见 `speech_server.json` 与《生产部署手册》；用户口令见《用户交互手册》。
