"""AI 盒子 HTTP 协议适配；请求失败绝不自动重试。"""

import json
import uuid
from http.client import HTTPException
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener


class AiBoxRequestError(RuntimeError):
    pass


class AiBoxClient:
    def __init__(self, base_url):
        self.base_url = str(base_url).rstrip("/")
        # 工位内网直连，不经过系统或环境代理。
        self.opener = build_opener(ProxyHandler({}))

    def request(self, path, payload=None, timeout=5.0):
        data = None if payload is None else json.dumps(
            {**payload, "request_id": str(uuid.uuid4())}, ensure_ascii=False
        ).encode("utf-8")
        http_request = Request(
            self.base_url + path,
            data=data,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="GET" if payload is None else "POST",
        )
        try:
            with self.opener.open(http_request, timeout=timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
        except (URLError, OSError, ValueError, HTTPException) as exc:
            raise AiBoxRequestError(
                "AI盒子%s请求失败（未自动重试）：%s" % (path, exc)
            ) from exc
        if not isinstance(result, dict):
            raise AiBoxRequestError("AI盒子%s响应不是JSON对象" % path)
        return result

