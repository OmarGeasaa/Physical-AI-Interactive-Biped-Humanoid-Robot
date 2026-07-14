import yaml
import threading
import time
import sys
import os
from audio_audio_capture import AudioCaptureService
from stt_vosk_streaming_stt import VoskStreamingSTT
from dialogue_ollama_client import OllamaClient
from dialogue_dialogue_manager import DialogueManager
from tts_tts_service import TTSService
from vision_face_recognition import FaceRecognitionService

def load_config():
    with open("config/config.yaml", "r") as f:
        return yaml.safe_load(f)

class RobotAssistant:
    def __init__(self, camera_id=None, use_face_recognition=True, faces_dir=None):
        print("🤖 Initializing Robot Assistant...")
        cfg = load_config()
        
        if faces_dir:
            cfg["face_recognition"]["known_faces_dir"] = faces_dir
            print(f"📁 Using custom faces directory: {faces_dir}")
        
        # Audio + STT
        self.audio = AudioCaptureService(
            sample_rate=cfg["audio"]["sample_rate"],
            chunk_ms=cfg["audio"]["chunk_ms"],
            device_index=cfg["audio"]["device_index"],
            wake_phrase=["what", "hey", "robot"],
        )
        self.stt = VoskStreamingSTT(
            model_path=cfg["stt"]["model_path"],
            sample_rate=cfg["audio"]["sample_rate"],
            confidence_min=cfg["stt"]["confidence_min"],
            partial_results=cfg["stt"]["partial_results"],
        )
        
        # LLM (Ollama)
        llm = OllamaClient(
            host=cfg["llm"]["host"],
            model=cfg["llm"]["model"],
            timeout_sec=cfg["llm"]["timeout_sec"],
            max_tokens=cfg["llm"]["max_tokens"],
            temperature=cfg["llm"]["temperature"],
        )
        system_prompt = (
            "You are a humanoid robot assistant. "
            "Be concise, helpful, and conversational."
        )
        self.dialogue = DialogueManager(llm, system_prompt)
        
        # TTS (Text-to-Speech)
        self.tts = TTSService(
            voice_name=cfg["tts"]["voice_name"],
            rate=cfg["tts"]["rate"],
            volume=cfg["tts"]["volume"],
        )
        
        # Face Recognition (optional)
        self.use_face_recognition = use_face_recognition and cfg.get("face_recognition", {}).get("enabled", True)
        self.face_recognition = None
        
        if self.use_face_recognition:
            if camera_id is None:
                camera_id = cfg.get("face_recognition", {}).get("camera_id", 0)
            
            known_faces_dir = cfg.get("face_recognition", {}).get("known_faces_dir", "known_faces")
            
            if not os.path.exists(known_faces_dir):
                print(f"⚠️  Faces directory '{known_faces_dir}' not found!")
                self.use_face_recognition = False
            else:
                self.face_recognition = FaceRecognitionService(
                    known_faces_dir=known_faces_dir,
                    model_path=cfg.get("face_recognition", {}).get("model_path", "face_model.pkl"),
                    confidence_threshold=cfg.get("face_recognition", {}).get("confidence_threshold", 0.6),
                    greeting_cooldown=cfg.get("face_recognition", {}).get("greeting_cooldown", 10),
                    camera_id=camera_id,
                    camera_width=cfg.get("face_recognition", {}).get("camera_width", 640),
                    camera_height=cfg.get("face_recognition", {}).get("camera_height", 480),
                    greeting_mode=cfg.get("face_recognition", {}).get("greeting_mode", "wake_only"),
                    face_seen_threshold=cfg.get("face_recognition", {}).get("face_seen_threshold", 3)
                )
                
                self.face_recognition.on_face_recognized = self.handle_face_recognized
                self.face_recognition.on_unknown_face = self.handle_unknown_face
                
                if self.face_recognition.start_camera():
                    print(f"📷 Face recognition service started")
                    print(f"👤 Known people: {', '.join(self.face_recognition.known_face_names)}")
                else:
                    print("⚠️  Face recognition service failed to start")
                    self.use_face_recognition = False
        
        self.is_speaking = False
        self.face_greeting_queue = []
        self.face_lock = threading.Lock()
        self.wake_triggered = False
        
        print("✅ Robot Assistant initialized successfully!")

    def handle_face_recognized(self, name, confidence):
        if self.is_speaking:
            with self.face_lock:
                self.face_greeting_queue.append(name)
            return
        self.greet_person(name, confidence)

    def handle_unknown_face(self):
        if self.face_recognition and self.face_recognition.is_awake:
            print("[Face] 👤 Unknown person detected")

    def identify_speaker_from_voice(self, user_text):
        if not self.face_recognition:
            return None
        for name in self.face_recognition.known_face_names:
            if name.lower() in user_text.lower():
                return name
        if self.face_recognition.last_known_face:
            return self.face_recognition.last_known_face
        return None

    def greet_person(self, name, confidence):
        if confidence < 0.5:
            return
        self.is_speaking = True
        try:
            greeting = f"Hello {name}! Nice to see you. How can I help you today?"
            print(f"👋 [Face] {greeting}")
            self.tts.speak(greeting)
            time.sleep(3)
        except Exception as e:
            print(f"[Face] Error greeting: {e}")
        finally:
            self.is_speaking = False
            with self.face_lock:
                if self.face_greeting_queue:
                    next_name = self.face_greeting_queue.pop(0)
                    threading.Thread(target=self.greet_person, args=(next_name, 0.7)).start()

    def handle_wake_phrase(self, user_text):
        if self.audio.update_wake_state(user_text):
            print("🔊 [Wake] Robot activated!")
            speaker_name = None
            if self.use_face_recognition and self.face_recognition:
                speaker_name = self.identify_speaker_from_voice(user_text)
                self.face_recognition.set_wake_state(True, speaker_name)
            self.wake_triggered = True
            if speaker_name:
                print(f"👤 [Wake] Identified speaker: {speaker_name}")
                threading.Timer(0.5, self.greet_person, args=(speaker_name, 0.8)).start()
            else:
                print("🔍 [Wake] Speaker not identified yet - checking camera...")
                if self.use_face_recognition and self.face_recognition:
                    time.sleep(1)
                    if self.face_recognition.last_known_face:
                        speaker_name = self.face_recognition.last_known_face
                        self.face_recognition.current_speaker = speaker_name
                        print(f"👤 [Wake] Identified from camera: {speaker_name}")
                        threading.Timer(0.5, self.greet_person, args=(speaker_name, 0.7)).start()
            return True
        return False

    def run(self):
        print("\n" + "="*50)
        print("🤖 Robot Assistant Running")
        print("="*50)
        print("Voice: Say 'what' to wake me up")
        if self.use_face_recognition and self.face_recognition:
            print(f"Face: I know {len(self.face_recognition.known_face_names)} people")
            print("       I'll greet recognized faces when you wake me")
        print("Press Ctrl+C to stop")
        print("="*50 + "\n")
        
        self.audio.start()
        
        try:
            while True:
                chunk = self.audio.read()
                result = self.stt.feed(chunk)
                
                if result["type"] == "partial" and result["text"]:
                    print(f"📝 (partial) {result['text']}")
                    
                if result["type"] == "final" and result["text"]:
                    user_text = result["text"]
                    
                    if self.handle_wake_phrase(user_text):
                        continue
                    
                    if not self.audio.is_awake():
                        print("💤 [Waiting] Say 'what' to wake up the robot")
                        continue
                    
                    print(f"💬 [User] {user_text}")
                    
                    self.is_speaking = True
                    try:
                        response = self.dialogue.handle(user_text)
                        print(f"🤖 [Robot] {response}")
                        self.tts.speak(response)
                        time.sleep(2)
                    except Exception as e:
                        print(f"❌ Error processing command: {e}")
                    finally:
                        self.is_speaking = False
                    
                    self.audio.reset_wake()
                    if self.use_face_recognition and self.face_recognition:
                        self.face_recognition.set_wake_state(False)
                    self.wake_triggered = False
                    
        except KeyboardInterrupt:
            print("\n🛑 Stopping...")
        finally:
            self.audio.stop()
            if self.use_face_recognition and self.face_recognition:
                self.face_recognition.stop_camera()
            print("👋 Goodbye!")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Robot Assistant with Face Recognition')
    parser.add_argument('--camera', type=int, default=None, help='Camera ID to use')
    parser.add_argument('--no-face', action='store_true', help='Disable face recognition')
    parser.add_argument('--faces-dir', type=str, default=None, 
                       help='Path to folder with person subfolders (default: known_faces)')
    args = parser.parse_args()
    
    assistant = RobotAssistant(
        camera_id=args.camera, 
        use_face_recognition=not args.no_face,
        faces_dir=args.faces_dir
    )
    assistant.run()

if __name__ == "__main__":
    main()