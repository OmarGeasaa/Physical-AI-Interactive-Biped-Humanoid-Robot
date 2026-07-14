import threading
import queue
import pyttsx3

class TTSService:
    def __init__(self, voice_name="robot", rate=170, volume=1.0):
        self.q = queue.Queue()
        self.voice_name = voice_name
        self.rate = rate
        self.volume = volume
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _set_voice(self, engine, voice_name):
        if not voice_name:
            return False
        for v in engine.getProperty("voices"):
            if voice_name.lower() in v.name.lower():
                engine.setProperty("voice", v.id)
                return True
        return False

    def speak(self, text):
        self.q.put(text)

    def _run(self):
        engine = pyttsx3.init()
        found = self._set_voice(engine, self.voice_name)
        if self.voice_name and not found:
            print(f"[TTS] Voice '{self.voice_name}' not found; using default.")
        engine.setProperty("rate", self.rate)
        engine.setProperty("volume", self.volume)
        while True:
            text = self.q.get()
            engine.say(text)
            engine.runAndWait()