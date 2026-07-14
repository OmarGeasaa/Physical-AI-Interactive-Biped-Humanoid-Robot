import requests

class OllamaClient:
    def __init__(self, host="http://localhost:11434", model="qwen3.5:4b",
                 timeout_sec=60, max_tokens=256, temperature=0.5):
        self.host = host.rstrip("/")
        self.model = model
        self.timeout = timeout_sec
        self.max_tokens = max_tokens
        self.temperature = temperature

    def generate(self, prompt):
        url = f"{self.host}/api/generate"
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "think": False,
            "options": {
                "num_predict": self.max_tokens,
                "temperature": self.temperature
            }
        }
        try:
            r = requests.post(url, json=payload, timeout=self.timeout)
            r.raise_for_status()
            data = r.json()
            return data.get("response", "").strip() or "I am ready, but I did not get a response from the language model."
        except requests.RequestException:
            return "I cannot reach the language model right now."
        except ValueError:
            return "I received an invalid response from the language model."