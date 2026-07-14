import json
from vosk import Model, KaldiRecognizer

class VoskStreamingSTT:
    def __init__(self, model_path, sample_rate=16000, confidence_min=0.6, partial_results=True):
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        self.recognizer.SetWords(True)
        self.confidence_min = confidence_min
        self.partial_results = partial_results

    def feed(self, audio_chunk):
        if self.recognizer.AcceptWaveform(audio_chunk):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").strip()
            conf = self._avg_confidence(result)
            if text and conf >= self.confidence_min:
                return {"type": "final", "text": text, "confidence": conf}
            return {"type": None, "text": "", "confidence": conf}
        else:
            if not self.partial_results:
                return {"type": None, "text": "", "confidence": 0.0}
            partial = json.loads(self.recognizer.PartialResult())
            return {"type": "partial", "text": partial.get("partial", "").strip(), "confidence": 0.0}

    def _avg_confidence(self, result):
        words = result.get("result", [])
        if not words:
            return 1.0 if result.get("text", "").strip() else 0.0
        return sum(w.get("conf", 0.0) for w in words) / len(words)