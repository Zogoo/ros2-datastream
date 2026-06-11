"""LLM access for data scientists: an OpenAI-compatible HTTP client plus a
deterministic mock so the stack (and CI) runs fully offline.

Selection:
  LLM_BASE_URL + LLM_API_KEY set -> OpenAICompatibleClient (any /chat/completions
                                    endpoint: OpenAI, Ollama, vLLM, LM Studio...)
  otherwise                      -> MockLLMClient

Both expose: complete(system, user) -> str (assistant text).
JSON helper: complete_json() parses the reply and falls back gracefully.
"""
from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Protocol


class LLMClient(Protocol):
    name: str

    def complete(self, system: str, user: str) -> str: ...


class OpenAICompatibleClient:
    def __init__(self, base_url: str, api_key: str, model: str, timeout_s: float = 30.0) -> None:
        self.name = f"openai-compatible:{model}"
        self._url = base_url.rstrip("/") + "/chat/completions"
        self._api_key = api_key
        self._model = model
        self._timeout = timeout_s

    def complete(self, system: str, user: str) -> str:
        payload = json.dumps({
            "model": self._model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "temperature": 0.1,
        }).encode()
        req = urllib.request.Request(
            self._url,
            data=payload,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._api_key}",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as res:
                body = json.loads(res.read())
            return body["choices"][0]["message"]["content"]
        except (urllib.error.URLError, KeyError, json.JSONDecodeError, TimeoutError) as exc:
            raise LLMError(f"LLM request failed: {exc}") from exc


class MockLLMClient:
    """Deterministic stand-in: applies the same priority rules a prompted LLM
    is asked to follow, so notebooks and tests behave identically offline."""

    name = "mock"

    def complete(self, system: str, user: str) -> str:
        try:
            context = json.loads(user)
        except json.JSONDecodeError:
            return json.dumps({"action": "continue_search", "reason": "unparseable context"})

        detections = context.get("detections", [])
        towels = [d for d in detections if d.get("class") == "towel"]
        if context.get("safety_stop"):
            return json.dumps({"action": "emergency_stop", "reason": "safety latch active"})
        if towels:
            nearest = min(
                towels, key=lambda d: (d.get("estimated_position") or {"x": 99}).get("x", 99),
            )
            return json.dumps({
                "action": "pick_object",
                "target_id": nearest["id"],
                "target_position": nearest.get("estimated_position"),
                "reason": "towel visible; nearest first",
            })
        return json.dumps({"action": "continue_search", "reason": "no towels in view"})


class LLMError(RuntimeError):
    pass


def create_llm_client() -> Any:
    base_url = os.environ.get("LLM_BASE_URL", "")
    api_key = os.environ.get("LLM_API_KEY", "")
    model = os.environ.get("LLM_MODEL", "gpt-4o-mini")
    if base_url and api_key:
        return OpenAICompatibleClient(base_url, api_key, model)
    return MockLLMClient()


def complete_json(client: Any, system: str, user: str) -> dict[str, Any]:
    try:
        raw = client.complete(system, user)
    except LLMError:
        return {"action": "continue_search", "reason": "llm unavailable"}
    try:
        start = raw.find("{")
        end = raw.rfind("}")
        return json.loads(raw[start: end + 1])
    except (json.JSONDecodeError, ValueError):
        return {"action": "continue_search", "reason": "llm reply not json", "raw": raw[:200]}
