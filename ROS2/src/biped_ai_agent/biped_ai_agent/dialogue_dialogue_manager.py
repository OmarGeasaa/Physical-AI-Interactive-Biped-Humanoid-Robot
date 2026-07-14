from collections import deque

class DialogueManager:
    def __init__(self, llm_client, system_prompt, max_turns=6):
        self.llm = llm_client
        self.system_prompt = system_prompt
        self.history = deque(maxlen=max_turns)

    def handle(self, user_text):
        self.history.append({"role": "user", "content": user_text})
        prompt = self._build_prompt()
        response = self.llm.generate(prompt)
        self.history.append({"role": "assistant", "content": response})
        return response

    def _build_prompt(self):
        lines = [f"System: {self.system_prompt}"]
        for h in self.history:
            lines.append(f"{h['role'].capitalize()}: {h['content']}")
        lines.append("Assistant:")
        return "\n".join(lines)