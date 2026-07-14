import cv2
import face_recognition
import numpy as np
import os
import pickle
import threading
import time
from collections import defaultdict
import logging
import platform

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FaceRecognitionService:
    def __init__(self, known_faces_dir="known_faces", model_path="face_model.pkl", 
                 confidence_threshold=0.6, greeting_cooldown=10,
                 camera_id=0, camera_width=640, camera_height=480,
                 greeting_mode="wake_only", face_seen_threshold=3):
        self.known_faces_dir = known_faces_dir
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.greeting_cooldown = greeting_cooldown
        self.camera_id = camera_id
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.greeting_mode = greeting_mode
        self.face_seen_threshold = face_seen_threshold
        
        self.known_face_encodings = []
        self.known_face_names = []
        self.last_greeting_time = defaultdict(float)
        self.face_seen_time = {}
        self.face_last_detected = {}
        
        self.lock = threading.Lock()
        self.is_running = False
        self.camera_thread = None
        self.video_capture = None
        
        self.current_speaker = None
        self.is_awake = False
        self.last_known_face = None
        
        if os.path.exists(model_path):
            self.load_model()
        else:
            logger.info("No existing model found. Training from your folders...")
            self.train_model()
            
        self.on_face_recognized = None
        self.on_unknown_face = None
        
        logger.info(f"Face recognition service initialized with {len(self.known_face_names)} faces")

    def train_model(self):
        logger.info("Training face recognition model from your folders...")
        
        if not os.path.exists(self.known_faces_dir):
            logger.error(f"Directory {self.known_faces_dir} does not exist!")
            return
            
        self.known_face_encodings = []
        self.known_face_names = []
        
        person_folders = [f for f in os.listdir(self.known_faces_dir) 
                         if os.path.isdir(os.path.join(self.known_faces_dir, f))]
        
        if not person_folders:
            logger.warning(f"No person folders found in {self.known_faces_dir}")
            return
        
        logger.info(f"Found {len(person_folders)} person folders")
        
        for person_name in person_folders:
            person_dir = os.path.join(self.known_faces_dir, person_name)
            
            if not os.path.isdir(person_dir):
                continue
                
            logger.info(f"Processing photos for: {person_name}")
            face_count = 0
            
            image_files = [f for f in os.listdir(person_dir) 
                          if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.gif'))]
            
            if not image_files:
                logger.warning(f"No image files found for {person_name}")
                continue
            
            for filename in image_files:
                image_path = os.path.join(person_dir, filename)
                
                try:
                    image = face_recognition.load_image_file(image_path)
                    face_encodings = face_recognition.face_encodings(image)
                    
                    if face_encodings:
                        self.known_face_encodings.append(face_encodings[0])
                        self.known_face_names.append(person_name)
                        face_count += 1
                        logger.debug(f"  Added face from: {filename}")
                    else:
                        logger.warning(f"  No face found in: {filename}")
                except Exception as e:
                    logger.error(f"  Error processing {filename}: {e}")
            
            logger.info(f"  Added {face_count} faces for {person_name}")
        
        self.save_model()
        logger.info(f"Model trained with {len(self.known_face_names)} faces from {len(person_folders)} people")

    def save_model(self):
        model_data = {
            'encodings': self.known_face_encodings,
            'names': self.known_face_names
        }
        try:
            with open(self.model_path, 'wb') as f:
                pickle.dump(model_data, f)
            logger.info(f"Model saved to {self.model_path}")
        except Exception as e:
            logger.error(f"Failed to save model: {e}")

    def load_model(self):
        try:
            with open(self.model_path, 'rb') as f:
                model_data = pickle.load(f)
            self.known_face_encodings = model_data['encodings']
            self.known_face_names = model_data['names']
            logger.info(f"Model loaded with {len(self.known_face_names)} faces")
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            logger.info("Training new model from your folders...")
            self.train_model()

    def set_wake_state(self, is_awake, speaker_name=None):
        self.is_awake = is_awake
        if is_awake and speaker_name:
            self.current_speaker = speaker_name
            logger.info(f"System awakened by: {speaker_name}")
        elif not is_awake:
            self.current_speaker = None
            logger.info("System went to sleep")

    def recognize_face(self, face_encoding):
        if not self.known_face_encodings:
            return None, 0.0
            
        matches = face_recognition.compare_faces(
            self.known_face_encodings, 
            face_encoding,
            tolerance=self.confidence_threshold
        )
        
        if True in matches:
            face_distances = face_recognition.face_distance(
                self.known_face_encodings, 
                face_encoding
            )
            best_match_index = np.argmin(face_distances)
            
            if matches[best_match_index]:
                confidence = 1 - face_distances[best_match_index]
                return self.known_face_names[best_match_index], confidence
                
        return None, 0.0

    def process_frame(self, frame):
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        
        detected_faces = []
        current_time = time.time()
        
        for face_encoding in face_encodings:
            name, confidence = self.recognize_face(face_encoding)
            
            if name:
                self.face_last_detected[name] = current_time
                
                should_greet = False
                greeting_reason = ""
                
                if self.greeting_mode == "always":
                    should_greet = True
                    greeting_reason = "always mode"
                    
                elif self.greeting_mode == "wake_only":
                    if self.is_awake and self.current_speaker == name:
                        should_greet = True
                        greeting_reason = "wake word"
                        
                elif self.greeting_mode == "hybrid":
                    if self.is_awake and self.current_speaker == name:
                        should_greet = True
                        greeting_reason = "wake word"
                    else:
                        if name not in self.face_seen_time:
                            self.face_seen_time[name] = current_time
                        
                        time_seen = current_time - self.face_seen_time[name]
                        
                        if time_seen >= self.face_seen_threshold:
                            should_greet = True
                            greeting_reason = f"seen for {time_seen:.1f}s"
                            self.face_seen_time[name] = current_time
                
                if should_greet:
                    if current_time - self.last_greeting_time[name] >= self.greeting_cooldown:
                        self.last_greeting_time[name] = current_time
                        logger.info(f"Greeting {name} ({greeting_reason}) - confidence: {confidence:.2f}")
                        
                        if self.on_face_recognized:
                            self.on_face_recognized(name, confidence)
                            
                        self.last_known_face = name
            else:
                if self.on_unknown_face and self.is_awake:
                    self.on_unknown_face()
                    
                if name in self.face_seen_time:
                    del self.face_seen_time[name]
            
            detected_faces.append({
                'name': name or "Unknown",
                'confidence': confidence,
                'box': None
            })
        
        for (top, right, bottom, left), face_info in zip(face_locations, detected_faces):
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            face_info['box'] = (top, right, bottom, left)
            
            if face_info['name'] != "Unknown":
                if self.is_awake and self.current_speaker == face_info['name']:
                    color = (0, 255, 0)
                else:
                    color = (255, 255, 0)
            else:
                color = (0, 0, 255)
            
            cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
            
            if self.is_awake and self.current_speaker == face_info['name']:
                status = "ACTIVE"
            elif face_info['name'] != "Unknown":
                status = "DETECTED"
            else:
                status = "UNKNOWN"
                
            label = f"{face_info['name']} ({status})"
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, label, (left + 6, bottom - 6), font, 0.8, (255, 255, 255), 1)
        
        return frame, detected_faces

    def start_camera(self, camera_id=None):
        if self.is_running:
            logger.warning("Camera already running")
            return False
            
        cam_id = camera_id if camera_id is not None else self.camera_id
        
        if isinstance(cam_id, str) and cam_id.startswith(('http', 'rtsp')):
            logger.info(f"Connecting to IP camera: {cam_id}")
            self.video_capture = cv2.VideoCapture(cam_id)
        else:
            backends = [cv2.CAP_ANY]
            
            if platform.system() == "Windows":
                backends.extend([cv2.CAP_DSHOW, cv2.CAP_MSMF])
            elif platform.system() == "Linux":
                backends.extend([cv2.CAP_V4L2, cv2.CAP_V4L])
            
            self.video_capture = None
            
            for backend in backends:
                try:
                    logger.info(f"Trying camera {cam_id} with backend {backend}")
                    cap = cv2.VideoCapture(cam_id, backend)
                    
                    if cap.isOpened():
                        self.video_capture = cap
                        break
                except Exception as e:
                    logger.debug(f"Backend {backend} failed: {e}")
            
            if not self.video_capture or not self.video_capture.isOpened():
                self.video_capture = cv2.VideoCapture(cam_id)
        
        if not self.video_capture.isOpened():
            logger.error(f"Could not open camera {cam_id}")
            self.is_running = False
            return False
            
        if self.camera_width and self.camera_height:
            self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        
        actual_width = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.video_capture.get(cv2.CAP_PROP_FPS)
        
        logger.info(f"Camera opened: {actual_width}x{actual_height} @ {fps:.1f}fps")
        
        self.is_running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        logger.info(f"Camera started on device {cam_id}")
        return True

    def _camera_loop(self):
        frame_skip = 0
        while self.is_running and self.video_capture:
            ret, frame = self.video_capture.read()
            if not ret:
                logger.error("Failed to capture frame")
                break
                
            frame_skip += 1
            if frame_skip % 2 == 0:
                continue
                
            processed_frame, detected_faces = self.process_frame(frame)
            
            if self.is_running:
                cv2.imshow('Face Recognition', processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop_camera()
                    break

    def stop_camera(self):
        self.is_running = False
        if self.camera_thread:
            self.camera_thread.join(timeout=2)
        if self.video_capture:
            self.video_capture.release()
        cv2.destroyAllWindows()
        logger.info("Camera stopped")