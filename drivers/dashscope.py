"""DashScope HTTP JSON 适配。"""

import json
from socket import timeout as SOCKET_TIMEOUT
from urllib import error, request


def post_json(url, payload, api_key, timeout, operation="模型接口"):
    body = json.dumps(payload).encode("utf-8")
    api_request = request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Authorization": "Bearer " + api_key,
            "Content-Type": "application/json",
        },
    )
    try:
        with request.urlopen(api_request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except error.HTTPError as exc:
        message = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError("%s调用失败：HTTP %d %s" % (operation, exc.code, message)) from exc
    except error.URLError as exc:
        if isinstance(exc.reason, (TimeoutError, SOCKET_TIMEOUT)):
            raise RuntimeError(
                "%s调用超时：当前超时设置 %.1f 秒。" % (operation, timeout)
            ) from exc
        raise RuntimeError("%s调用失败：%r" % (operation, exc.reason)) from exc
    except SOCKET_TIMEOUT as exc:
        raise RuntimeError(
            "%s调用超时：当前超时设置 %.1f 秒。" % (operation, timeout)
        ) from exc

