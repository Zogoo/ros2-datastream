"""LLM client tests: mock determinism + OpenAI-compatible payload shape."""
import io
import json

from onsen_ai_worker import llm_client
from onsen_ai_worker.llm_client import (
    MockLLMClient,
    OpenAICompatibleClient,
    complete_json,
    create_llm_client,
)

CONTEXT = json.dumps({
    "safety_stop": False,
    "detections": [
        {"id": "det_001", "class": "towel", "estimated_position": {"x": 1.1, "y": 0.0}},
        {"id": "det_002", "class": "towel", "estimated_position": {"x": 0.6, "y": 0.4}},
    ],
    "towels_remaining": 2,
    "holding": False,
})


class TestMock:
    def test_deterministic(self):
        client = MockLLMClient()
        assert client.complete("sys", CONTEXT) == client.complete("sys", CONTEXT)

    def test_picks_nearest_towel(self):
        verdict = complete_json(MockLLMClient(), "sys", CONTEXT)
        assert verdict["action"] == "pick_object"
        assert verdict["target_id"] == "det_002"

    def test_safety_stop_wins(self):
        ctx = json.loads(CONTEXT)
        ctx["safety_stop"] = True
        verdict = complete_json(MockLLMClient(), "sys", json.dumps(ctx))
        assert verdict["action"] == "emergency_stop"

    def test_unparseable_context_falls_back(self):
        verdict = complete_json(MockLLMClient(), "sys", "not json")
        assert verdict["action"] == "continue_search"


class TestClientSelection:
    def test_mock_when_env_unset(self, monkeypatch):
        monkeypatch.delenv("LLM_BASE_URL", raising=False)
        monkeypatch.delenv("LLM_API_KEY", raising=False)
        assert create_llm_client().name == "mock"

    def test_openai_compatible_when_configured(self, monkeypatch):
        monkeypatch.setenv("LLM_BASE_URL", "http://localhost:11434/v1")
        monkeypatch.setenv("LLM_API_KEY", "key")
        monkeypatch.setenv("LLM_MODEL", "qwen2")
        assert create_llm_client().name == "openai-compatible:qwen2"


class TestOpenAIPayload:
    def test_request_shape_and_auth(self, monkeypatch):
        captured = {}

        def fake_urlopen(req, timeout):
            captured["url"] = req.full_url
            captured["auth"] = req.get_header("Authorization")
            captured["payload"] = json.loads(req.data)
            body = json.dumps({
                "choices": [{"message": {"content": '{"action": "pick_object"}'}}],
            }).encode()
            return _FakeResponse(body)

        monkeypatch.setattr(llm_client.urllib.request, "urlopen", fake_urlopen)
        client = OpenAICompatibleClient("http://llm.local/v1/", "secret", "test-model")
        reply = client.complete("system prompt", "user prompt")

        assert reply == '{"action": "pick_object"}'
        assert captured["url"] == "http://llm.local/v1/chat/completions"
        assert captured["auth"] == "Bearer secret"
        assert captured["payload"]["model"] == "test-model"
        assert [m["role"] for m in captured["payload"]["messages"]] == ["system", "user"]

    def test_llm_failure_degrades_to_search(self, monkeypatch):
        def fail(req, timeout):
            raise llm_client.urllib.error.URLError("down")

        monkeypatch.setattr(llm_client.urllib.request, "urlopen", fail)
        client = OpenAICompatibleClient("http://llm.local/v1", "k", "m")
        verdict = complete_json(client, "sys", CONTEXT)
        assert verdict["action"] == "continue_search"


class _FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False
