#!/home/jetson/ai_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import threading
import time
import yaml
import os
import sys

# =====================================================================
# 1. THE PATH BYPASS
# =====================================================================
current_file_path = os.path.realpath(__file__)
physical_src_dir = os.path.dirname(current_file_path)

if physical_src_dir not in sys.path:
    sys.path.insert(0, physical_src_dir)

# =====================================================================
# 2. SIBLING IMPORTS
# =====================================================================
from audio_audio_capture import AudioCaptureService
from stt_vosk_streaming_stt import VoskStreamingSTT
from dialogue_ollama_client import OllamaClient
from dialogue_dialogue_manager import DialogueManager
from tts_tts_service import TTSService
from vision_face_recognition import FaceRecognitionService

# =====================================================================
# 3. THE ROS 2 NODE
# =====================================================================
class BipedAIAgent(Node):
    def __init__(self):
        super().__init__('biped_ai_agent')
        
        self.get_logger().info("Initializing ROS 2 AI Agent inside Virtual Environment...")
        
        # --- THE ASSET BYPASS ---
        ASSET_DIR = os.path.expanduser('~/ros2_ws/src/biped_ai_agent/biped_ai_agent')
        config_path = os.path.join(ASSET_DIR, 'config', 'config.yaml')
        
        try:
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config.yaml: {e}")
            sys.exit(1)

        # Initialize Audio
        self.audio = AudioCaptureService(
            sample_rate=cfg["audio"]["sample_rate"],
            chunk_ms=cfg["audio"]["chunk_ms"],
            device_index=cfg["audio"]["device_index"],
            wake_phrase=["what", "hey", "robot"]
        )
        
        # Initialize STT (Vosk)
        stt_model = os.path.join(ASSET_DIR, cfg["stt"]["model_path"])
        self.stt = VoskStreamingSTT(
            model_path=stt_model,
            sample_rate=cfg["audio"]["sample_rate"]
        )
        
        # Initialize LLM
        llm = OllamaClient(
            host=cfg["llm"]["host"], 
            model=cfg["llm"]["model"]
        )
        self.dialogue = DialogueManager(llm, "You are a bipedal robot assistant.")
        
        # Initialize TTS
        self.tts = TTSService(voice_name=cfg["tts"]["voice_name"])
        
        # Initialize Face Recognition
        faces_dir = os.path.join(ASSET_DIR, cfg["face_recognition"]["known_faces_dir"])
        model_rel_path = cfg.get("face_recognition", {}).get("model_path", "models/face_model.pkl")
        face_model_pkl = os.path.join(ASSET_DIR, model_rel_path)
        
        self.face_rec = FaceRecognitionService(
            known_faces_dir=faces_dir,
            model_path=face_model_pkl
        )
        self.face_rec.on_face_recognized = self.handle_face_recognized
        
        # ROS 2 State Variables
        self.bridge = CvBridge()
        self.is_double_stance = False 
        self.is_speaking = False

        # ROS 2 Subscribers
        self.create_subscription(Int32, '/gait_phase', self.gait_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Start the Audio Listening Thread
        self.audio.start()
        self.audio_thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.audio_thread.start()

        self.get_logger().info("AI Agent Online. Waiting for Double Stance (Phase 0)...")

    def gait_callback(self, msg):
        if msg.data == 0 and not self.is_double_stance:
            self.is_double_stance = True
            self.get_logger().info("[STATE] Robot Grounded. Activating Microphone and Vision.")
            # self.audio.queue.queue.clear()  <-- Commented out to prevent AttributeError

        elif msg.data != 0 and self.is_double_stance:
            self.is_double_stance = False
            self.get_logger().info("[STATE] Robot Stepping. Pausing AI to ignore motor noise.")

    def camera_callback(self, msg):
        if self.is_double_stance and not self.is_speaking:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                processed_frame, detected_faces = self.face_rec.process_frame(cv_image)
            except Exception as e:
                self.get_logger().error(f"Camera Bridge Error: {e}")

    def handle_face_recognized(self, name, confidence):
        if self.is_speaking:
            return
        self.is_speaking = True
        greeting = f"Hello {name}! Nice to see you."
        self.get_logger().info(f"[Face Recognized] {greeting}")
        self.tts.speak(greeting)
        time.sleep(2)
        self.is_speaking = False

    def audio_loop(self):
        while rclpy.ok():
            if self.is_double_stance and not self.is_speaking:
                chunk = self.audio.read()
                if chunk:
                    result = self.stt.feed(chunk)
                    
                    if result and result.get("type") == "final" and result.get("text"):
                        user_text = result["text"]
                        
                        # Prevent empty strings from triggering the logic
                        if user_text.strip():
                            self.get_logger().info(f"[DEBUG] Vosk Heard: '{user_text}'")
                            
                            if self.audio.update_wake_state(user_text):
                                self.get_logger().info("[DEBUG] WAKE WORD DETECTED! Waking up vision...")
                                self.face_rec.set_wake_state(True)
                                continue
                                
                            if self.audio.is_awake():
                                self.is_speaking = True
                                self.get_logger().info(f"[DEBUG] Sending to LLM: '{user_text}'")
                                
                                try:
                                    self.get_logger().info("[DEBUG] Waiting for Ollama response...")
                                    response = self.dialogue.handle(user_text)
                                    self.get_logger().info(f"[DEBUG] Ollama Replied: '{response}'")
                                    
                                    self.get_logger().info("[DEBUG] Sending to TTS Speakers...")
                                    self.tts.speak(response)
                                except Exception as e:
                                    self.get_logger().error(f"LLM/TTS Error: {e}")
                                    
                                self.is_speaking = False
                                self.audio.reset_wake()
                                self.face_rec.set_wake_state(False)
                                self.get_logger().info("[DEBUG] Cycle complete. Listening for wake word...")
            else:
                time.sleep(0.05) 

def main(args=None):
    rclpy.init(args=args)
    node = BipedAIAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.audio.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
