import queue
import re
import threading
import sounddevice as sd

class AudioCaptureService:
    def __init__(self, sample_rate=16000, chunk_ms=20, device_index=None, wake_phrase="what",):
        self.sample_rate = sample_rate
        self.chunk_ms = chunk_ms
        self.device_index = device_index
        self.chunk_size = int(self.sample_rate * (self.chunk_ms / 1000.0))
        self.q = queue.Queue()
        self.stream = None
        self.wake_phrase = wake_phrase or ""
        self._wake_regex = self._build_wake_regex(self.wake_phrase)
        self._awake = threading.Event()
        if not self._wake_regex:
            self._awake.set()

    def _build_wake_regex(self, wake_phrase):
        if not wake_phrase:
            return None
        if isinstance(wake_phrase, (list, tuple, set)):
            phrases = [p for p in wake_phrase if p]
            if not phrases:
                return None
            pattern = "(?:" + "|".join(re.escape(p) for p in phrases) + ")"
            return re.compile(pattern, re.IGNORECASE)
        if isinstance(wake_phrase, str) and "|" in wake_phrase:
            parts = [p.strip() for p in wake_phrase.split("|") if p.strip()]
            if not parts:
                return None
            pattern = "(?:" + "|".join(re.escape(p) for p in parts) + ")"
            return re.compile(pattern, re.IGNORECASE)
        return re.compile(re.escape(str(wake_phrase)), re.IGNORECASE)

    def _callback(self, indata, frames, time, status):
        if status:
            pass
        self.q.put(bytes(indata))

    def start(self):
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.chunk_size,
            device=self.device_index,
            dtype="int16",
            channels=1,
            callback=self._callback,
        )
        self.stream.start()

    def read(self):
        return self.q.get()

    def update_wake_state(self, text):
        if self._wake_regex and text and self._wake_regex.search(text):
            self._awake.set()
            return True
        return False

    def is_awake(self):
        return self._awake.is_set()

    def reset_wake(self):
        if self._wake_regex:
            self._awake.clear()

    def stop(self):
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None