import socket
import threading
import struct
import cv2
import numpy as np
import time
import select
import ctypes
import re
import os

from chaser_logic import chaser_box_metrics
from decision_logic import evaluate_decision
from lane_geometry import occupied_lanes, fit_road_model, model_from_bounds

try:
    import pytesseract
    TESSERACT_AVAILABLE = True
    if os.path.exists(r'C:\Program Files\Tesseract-OCR\tesseract.exe'):
        pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
except ImportError:
    TESSERACT_AVAILABLE = False
    print("[WARNING] pytesseract module not found. Golden Lane disabled.")

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
CAMERA_HOST = '127.0.0.1'
FRONT_CAMERA_PORT = 8080
BACK_CAMERA_PORT = 8082
CONTROL_HOST = '127.0.0.1'
CONTROL_PORT = 8081

# Split Locks for Max Performance
frame_lock = threading.Lock()
state_lock = threading.Lock()

shared_data = {
    'latest_front_frame': None,
    'latest_back_frame': None,
    'steering_input' : 0.0,
    'acceleration_input' : 1.0,
    'tap_state': 'IDLE',      
    'debug_info': "WAITING",  
    'debug_tokens': [],        
    'net_lane_position': 0,
    'lane_history': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
    'road_bounds': None,   
    'low_light': False,
    'chaser_behind': False,
    'chaser_boxes': [],
    'chaser_evade_end_time': 0.0,
    'chaser_proximity': 0.0,
    'chaser_side': 0,
    'seek_red_end_time': 0.0,
    'police_seen_end_time': 0.0,
    'golden_lane_target': 0,
    'golden_lane_end_time': 0.0,
    'debug_ocr_text': "",
    'debug_golden_mask': None
}
is_running = True
baseline_brightness = None

tap_state = 'IDLE'            
tap_timer = 0
active_steering_value = 0.0
smoothed_steering = 0.0      
TAP_HOLD_FRAMES = 12
COOLDOWN_FRAMES = 15

# --- Chaser evasion tunables (EV3/EV4) ---
MIN_CHASER_AREA = 5           # rear-mask noise floor (320x240 space); low so a
                              # far/small chaser is seen early -- largest-contour
                              # selection already rejects stray noise. Raise if
                              # false weaves appear with no chaser present.
CHASER_EVADE_LATCH_SEC = 0.6  # keep evading through 1-frame detection dropouts
CHASER_CLOSE_PROXIMITY = 0.6  # >= this => escalate (never coast)
EMERGENCY_TAP_HOLD_FRAMES = 7 # shorter hold so lane changes chain into a weave
EMERGENCY_COOLDOWN_FRAMES = 2 # near-zero cooldown during chaser evasion
GOLDEN_COMMIT_SEC = 2.5       # commit to golden lane for the final part of the
                              # 5.5s OCR latch; wide enough to cover the real EV5
                              # expiry despite OCR read lag (tune vs. EV5 hit rate)
MAX_RED_TOKEN_AREA = 130      # reds bigger than this are treated as the police
                              # CAR (never driven into), not a collectible token.
                              # Lower if police still gets hit; raise if real red
                              # tokens stop being collected (EV2). Read live sizes
                              # off the "RED:<area>" overlay labels to tune.
evade_dir = 1                 # current sweep direction (-1 left / +1 right)

# Morphology Kernel
morph_kernel = np.ones((3, 3), np.uint8)

# ---------------------------------------------------------
# Real-Time Scheduling Framework 
# ---------------------------------------------------------
class TaskPriority:
    HIGH = 1
    MEDIUM = 2
    LOW = 3

class RTTask(threading.Thread):
    """
    Real-Time Task implementing:
    - Concurrency (inherits threading.Thread)
    - Task Period (enforced in run loop)
    - Task Priority (logical priority assigned)
    """
    def __init__(self, name, period, priority, execute_func):
        super().__init__()
        self.name = name
        self.period = period
        self.priority = priority
        self.execute_func = execute_func
        self.daemon = True

    def run(self):
        print(f"[{self.name}] Started | Period: {self.period}s | Priority: {self.priority}")
        try:
            handle = ctypes.windll.kernel32.GetCurrentThread()
            if self.priority == TaskPriority.HIGH: ctypes.windll.kernel32.SetThreadPriority(handle, 2)
            elif self.priority == TaskPriority.MEDIUM: ctypes.windll.kernel32.SetThreadPriority(handle, 0)
            elif self.priority == TaskPriority.LOW: ctypes.windll.kernel32.SetThreadPriority(handle, -2)
        except Exception: pass

        while is_running:
            start_time = time.time()
            self.execute_func()
            exec_time = time.time() - start_time
            sleep_time = self.period - exec_time
            if sleep_time > 0:
                if sleep_time > 0.002: time.sleep(sleep_time - 0.002)
                while (time.time() - start_time) < self.period: pass

# ---------------------------------------------------------
# Network Connection Setup
# ---------------------------------------------------------
front_camera_sock = None
back_camera_sock = None
control_conn = None

def setup_cameras():
    global front_camera_sock, back_camera_sock
    print("Connecting to Cameras...")
    front_connected = False
    back_connected = False
    while is_running and not (front_connected and back_connected):
        if not front_connected:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((CAMERA_HOST, FRONT_CAMERA_PORT))
                front_camera_sock = s
                print("Connected to Front Camera successfully.")
                front_connected = True
            except Exception: pass
        if not back_connected:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(1.0)
                s.connect((CAMERA_HOST, BACK_CAMERA_PORT))
                back_camera_sock = s
                print("Connected to Back Camera successfully.")
                back_connected = True
            except Exception: pass
        if not (front_connected and back_connected): time.sleep(1)

def setup_control_server():
    global control_conn
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((CONTROL_HOST, CONTROL_PORT))
    server_sock.listen()
    server_sock.settimeout(1.0)
    print(f"Control server listening on {CONTROL_HOST}:{CONTROL_PORT}")
    while is_running:
        try:
            conn, addr = server_sock.accept()
            print(f"Control client connected from {addr}")
            control_conn = conn
            break
        except socket.timeout: continue

# ---------------------------------------------------------
# Task Implementations (This is where you write your tasks)
# ---------------------------------------------------------

def read_single_camera(sock, data_key):
    #This function reads the latest frame from the camera socket and stores it in the shared data
    if sock is None: return
    try:
        sock.settimeout(None)
        length_bytes = sock.recv(4)
        if not length_bytes: return
        image_length = int.from_bytes(length_bytes, 'little')
        received_bytes = b''
        while len(received_bytes) < image_length and is_running:
            packet = sock.recv(image_length - len(received_bytes))
            if not packet: break
            received_bytes += packet
        if len(received_bytes) == image_length:
            np_arr = np.frombuffer(received_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with frame_lock: shared_data[data_key] = frame

        # Clear backlog to ensure real-time performance
        while is_running:
            readable, _, _ = select.select([sock], [], [], 0.0)
            if not readable: break
            sock.settimeout(1.0)
            length_bytes = sock.recv(4)
            if not length_bytes: return
            image_length = int.from_bytes(length_bytes, 'little')
            received_bytes = b''
            while len(received_bytes) < image_length and is_running:
                packet = sock.recv(image_length - len(received_bytes))
                if not packet: break
                received_bytes += packet
            if len(received_bytes) == image_length:
                np_arr = np.frombuffer(received_bytes, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    with frame_lock: shared_data[data_key] = frame
    except Exception: pass

def read_front_camera_task(): read_single_camera(front_camera_sock, 'latest_front_frame')
def read_back_camera_task(): read_single_camera(back_camera_sock, 'latest_back_frame')

# ------------------------------------
# OCR Golden Lane Scanner Task
# ------------------------------------
def ocr_task():
    if not TESSERACT_AVAILABLE: return
    with frame_lock: frame = shared_data.get('latest_front_frame')
    if frame is None: return

    with state_lock:
        if time.time() < shared_data.get('golden_lane_end_time', 0.0):
            return

    try:
        h, w = frame.shape[:2]
        crop_img = frame[0:int(h * 0.15), int(w * 0.1):int(w * 0.9)]
        
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, 100, 100]), np.array([40, 255, 255]))
        
        with state_lock: shared_data['debug_golden_mask'] = mask.copy()
        
        if cv2.countNonZero(mask) > 15: 
            mask_large = cv2.resize(mask, (0, 0), fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
            mask_inv = cv2.bitwise_not(mask_large)
            mask_padded = cv2.copyMakeBorder(mask_inv, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
            
            text = pytesseract.image_to_string(mask_padded, config='--psm 7').upper()
            
            with state_lock: shared_data['debug_ocr_text'] = text.strip()
            
            match = re.search(r'LANE\s*([1-5])', text)
            if match:
                lane_num = int(match.group(1)) 
                with state_lock:
                    shared_data['golden_lane_target'] = lane_num
                    shared_data['golden_lane_end_time'] = time.time() + 5.5 
        else:
            with state_lock: shared_data['debug_ocr_text'] = ""
    except Exception: pass

# ---------------------------------------------------------
# Simplified Perception & Decision
# ---------------------------------------------------------
ROI_START_Y = 100

def get_occupied_lanes(x, y, w, h, road_model=None):
    # Classify against the fitted road model (floating horizon) when available so
    # decisions match the real road through slopes; falls back to a fixed model
    # when no curbs are seen. Pure math lives in lane_geometry (unit-tested).
    return occupied_lanes(x + w/2.0, y + h/2.0, w, road_model, ROI_START_Y)

def detect_back_environment(back_frame):
    if back_frame is None: return [], 0.0, 0
    small_frame = cv2.resize(back_frame, (320, 240))
    roi_back = small_frame[130:240, 40:280]
    roi_hsv = cv2.cvtColor(roi_back, cv2.COLOR_BGR2HSV)
    mask_car = cv2.inRange(roi_hsv, np.array([85, 150, 80]), np.array([130, 255, 255]))
    mask_car = cv2.morphologyEx(mask_car, cv2.MORPH_OPEN, morph_kernel)
    contours, _ = cv2.findContours(mask_car, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Pick the LARGEST valid contour (the closest/real chaser), not the first.
    best_rect = None
    best_area = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CHASER_AREA:
            continue
        if area > best_area:
            best_area = area
            best_rect = cv2.boundingRect(c)

    if best_rect is None:
        return [], 0.0, 0

    x, y, w, h = best_rect
    final_box, proximity, side = chaser_box_metrics(x, y, w, h)
    return [final_box], proximity, side

def is_valid_3d_token(x, y, w, h, area):
    if area < 5: return False
    aspect_ratio = float(w) / h if h > 0 else 0
    if not (0.1 <= aspect_ratio <= 2.5): return False
    min_required_height = max(5, y * 0.15) 
    if h < min_required_height: return False
    return True

def detect_environment(front_frame):
    global baseline_brightness

    small_frame = cv2.resize(front_frame, (320, 240))
    roi_front = small_frame[ROI_START_Y:190, 0:320]
    
    blurred_roi = cv2.GaussianBlur(roi_front, (5, 5), 0)
    roi_hsv = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)
    
    gray_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
    current_periph_brightness = (np.mean(gray_frame[120:180, 0:40]) + np.mean(gray_frame[120:180, 280:320])) / 2.0
    
    if baseline_brightness is None:
        if current_periph_brightness > 40: baseline_brightness = current_periph_brightness
            
    low_light_mode = False
    if baseline_brightness is not None:
        if current_periph_brightness < (baseline_brightness * 0.3): low_light_mode = True
        elif current_periph_brightness > (baseline_brightness * 0.5):
            baseline_brightness = (baseline_brightness * 0.99) + (current_periph_brightness * 0.01)

    mask_green = cv2.inRange(roi_hsv, np.array([35, 80, 80]), np.array([85, 255, 255]))
    mask_red1 = cv2.inRange(roi_hsv, np.array([0, 100, 80]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(roi_hsv, np.array([170, 100, 80]), np.array([180, 255, 255]))
    mask_red = mask_red1 | mask_red2
    mask_blue = cv2.inRange(roi_hsv, np.array([90, 100, 100]), np.array([135, 255, 255]))
    mask_white = cv2.inRange(roi_hsv, np.array([0, 0, 180]), np.array([180, 45, 255]))
    
    mask_curb = cv2.bitwise_or(mask_red, mask_white)
    mask_curb = cv2.morphologyEx(mask_curb, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8), iterations=2)
    
    mask_curb_clean = np.zeros_like(mask_curb)
    contours_curb, _ = cv2.findContours(mask_curb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours_curb:
        if cv2.contourArea(c) > 150: 
            cv2.drawContours(mask_curb_clean, [c], -1, 255, -1)
            
    h_roi, w_roi = roi_hsv.shape[:2]
    best_road_bounds = None
    max_measured_gap = 0
    road_samples = []  # (y_fullframe, center_x, road_width) per measured curb row

    # Scan many rows across the ROI (not just the bottom few) so the road model
    # can FIT where the curbs converge -- a horizon that floats with the camera
    # pitch on slopes, instead of a fixed horizon that drifts out of position.
    for scan_y in range(h_roi - 5, 5, -5):
        row = mask_curb_clean[scan_y, :]
        curb_idx = np.where(row > 0)[0]
        curb_idx = np.concatenate(([0], curb_idx, [w_roi - 1]))

        diffs = np.diff(curb_idx)
        if len(diffs) == 0:
            continue
        max_gap_idx = np.argmax(diffs)
        max_gap = int(diffs[max_gap_idx])
        if max_gap < 80:
            continue
        x_left = int(curb_idx[max_gap_idx])
        x_right = int(curb_idx[max_gap_idx + 1])
        road_samples.append((scan_y + ROI_START_Y, (x_left + x_right) / 2.0, float(max_gap)))
        if max_gap > max_measured_gap:
            max_measured_gap = max_gap
            best_road_bounds = (x_left, x_right, max_gap / 5.0, scan_y)

    # Prefer the multi-row fit (slope-aware); else the single widest row; else fixed.
    road_model = fit_road_model(road_samples) or model_from_bounds(best_road_bounds, ROI_START_Y)

    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, morph_kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, morph_kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, morph_kernel)

    contours_g, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_objects, debug_tokens = [], []
    raw_red_boxes, red_areas = [], {} 
    
    for c in contours_red:
        area = cv2.contourArea(c)
        if area > 5: 
            x, y, w, h = cv2.boundingRect(c)
            raw_red_boxes.append((x, y, w, h))
            red_areas[(x, y, w, h)] = area

    police_car_zones = []

    for c in contours_blue:
        area = cv2.contourArea(c)
        if area > 10: 
            bx, by, bw, bh = cv2.boundingRect(c)
            if by <= 15: continue
            b_cx, b_cy = bx + bw/2.0, by + bh/2.0
            
            is_police = False
            rx, ry, rw, rh = 0, 0, 0, 0
            
            for tx, ty, tw, th in raw_red_boxes:
                if abs(b_cy - (ty + th/2.0)) < 15 and (0.3 < (bw / max(1.0, float(tw))) < 3.0) and abs(b_cx - (tx + tw/2.0)) < (bw/2.0 + tw/2.0 + max(bw, tw)*1.5):
                    is_police, rx, ry, rw, rh = True, tx, ty, tw, th
                    break 
            
            if is_police:
                light_x, light_y = min(bx, rx), min(by, ry)
                light_w, light_h = max(bx+bw, rx+rw) - light_x, max(by+bh, ry+rh) - light_y
                if light_w < 120 and (light_w / max(1.0, float(light_h))) > 1.2:
                    pad_x = int(light_w * 0.5)
                    car_x, car_w = max(0, light_x - pad_x), int(light_w * 2.0)
                    car_y, car_h = max(0, light_y - int(light_h * 0.2)), int(light_h * 4.0)
                    police_car_zones.append((car_x, car_y, car_w, car_h))
                    
                    lanes = get_occupied_lanes(car_x, car_y, car_w, car_h, road_model)
                    if lanes:
                        detected_objects.append({'type': 'DANGER', 'subtype': 'POLICE', 'lanes': lanes, 'dist': (car_y + car_h/2 + ROI_START_Y) - 80})
                        debug_tokens.append(('POLICE', car_x, car_y, car_w, car_h))

    for r_box in raw_red_boxes:
        x, y, w, h = r_box
        cx, cy = x + w/2, y + h/2
        if any(px <= cx <= px+pw and py <= cy <= py+ph for (px, py, pw, ph) in police_car_zones): continue
        area = red_areas[(x, y, w, h)]
        if is_valid_3d_token(x, y, w, h, area):
            lanes = get_occupied_lanes(x, y, w, h, road_model)
            if lanes:
                detected_objects.append({'type': 'DANGER', 'subtype': 'RED', 'lanes': lanes, 'dist': (y + h/2 + ROI_START_Y) - 80, 'area': area})
                debug_tokens.append((f'RED:{int(area)}', x, y, w, h))

    # Yellow tokens are NEUTRAL (net = green - red), so they are not avoided.
    # Only green is harvested here; red is handled above as danger.
    for c in contours_g:
        area, x, y, w, h = cv2.contourArea(c), *cv2.boundingRect(c)
        cx, cy = x + w/2, y + h/2
        if any(px <= cx <= px+pw and py <= cy <= py+ph for (px, py, pw, ph) in police_car_zones): continue
        if is_valid_3d_token(x, y, w, h, area):
            lanes = get_occupied_lanes(x, y, w, h, road_model)
            if lanes:
                detected_objects.append({'type': 'GREEN', 'subtype': 'GREEN', 'lanes': lanes, 'dist': (y + h/2 + ROI_START_Y) - 80})
                debug_tokens.append(('GREEN', x, y, w, h))
    return detected_objects, debug_tokens, low_light_mode, best_road_bounds

# ---------------------------------------------------------
# Decision Making
# ---------------------------------------------------------
# The priority cascade now lives in decision_logic.evaluate_decision (pure /
# dependency-free so it can be unit-tested). This file owns perception, the
# evade_dir state, and the shared_data plumbing around it.

def processing_task():
    #This is where you write your image processing code to decide how to control the car
    #You can use libraries like OpenCV to process the image
    #There is no limtation to the complexity of the processing task, you can use any libraries you want
    #Remember to use the shared_data to get the latest frame
    global tap_state, evade_dir
    with frame_lock:
        front_frame = shared_data['latest_front_frame']
        back_frame = shared_data.get('latest_back_frame')
    
    with state_lock:
        current_lane = shared_data.get('net_lane_position', 0)
        last_processed_id = shared_data.get('last_processed_id', None)
        
    if front_frame is not None and id(front_frame) != last_processed_id:
        with state_lock:
            shared_data['last_processed_id'] = id(front_frame)
            
        chaser_boxes, chaser_proximity, chaser_side = detect_back_environment(back_frame)
        chaser_detected = len(chaser_boxes) > 0
        now = time.time()
        with state_lock:
            was_latched = now < shared_data.get('chaser_evade_end_time', 0.0)
            if chaser_detected:
                shared_data['chaser_evade_end_time'] = now + CHASER_EVADE_LATCH_SEC
                shared_data['chaser_proximity'] = chaser_proximity
                shared_data['chaser_side'] = chaser_side
                if not was_latched:
                    # Fresh chaser: start the sweep AWAY from where it is.
                    evade_dir = -1 if chaser_side > 0 else 1
            chaser_behind = now < shared_data.get('chaser_evade_end_time', 0.0)
        detected_objects, debug_tokens, low_light_mode, road_bounds = detect_environment(front_frame)
        
        with state_lock:
            shared_data['road_bounds'] = road_bounds

        police_detected = any('POLICE' in t[0] for t in debug_tokens)
        
        with state_lock:
            now2 = time.time()
            # EV2: the police car stays on-screen ~10s and you have the FULL 10s to
            # collect a red. Arm a one-shot 10s window on a fresh police sighting
            # (debounced 1s through detection dropouts, like the chaser latch) rather
            # than a short window re-extended each frame -- so the seek survives the
            # police car leaving the front ROI before a red is grabbed, and does not
            # re-arm after one is collected (which would over-collect score-negative reds).
            police_seen_end = shared_data.get('police_seen_end_time', 0.0)
            police_present_before = now2 < police_seen_end
            if police_detected:
                shared_data['police_seen_end_time'] = now2 + 1.0
                if not police_present_before:
                    shared_data['seek_red_end_time'] = now2 + 10.0

            seek_red_end = shared_data.get('seek_red_end_time', 0.0)
            seek_red_mode = now2 < seek_red_end
            if seek_red_mode:
                for obj in detected_objects:
                    # Only a small red TOKEN counts as collected -- a large red
                    # (police car body) ahead must not end the seek.
                    if obj.get('subtype') == 'RED' and 0 in obj['lanes'] and obj.get('area', 0) <= MAX_RED_TOKEN_AREA:
                        if obj['dist'] > -5:
                            shared_data['seek_red_end_time'] = 0.0
                            seek_red_mode = False
                            break
            
            golden_lane_end = shared_data.get('golden_lane_end_time', 0.0)
            golden_lane_target = shared_data.get('golden_lane_target', 0)
            golden_time_left = golden_lane_end - time.time()
            proximity_for_decision = shared_data.get('chaser_proximity', 0.0)

        target_steer, target_accel, debug_text, evade_dir = evaluate_decision(
            detected_objects, current_lane, low_light_mode, chaser_behind, seek_red_mode,
            golden_time_left, golden_lane_target, evade_dir, proximity_for_decision,
            chaser_close_proximity=CHASER_CLOSE_PROXIMITY,
            max_red_token_area=MAX_RED_TOKEN_AREA,
            golden_commit_sec=GOLDEN_COMMIT_SEC,
        )

        with state_lock:
            shared_data['steering_input'] = target_steer
            shared_data['acceleration_input'] = target_accel
            shared_data['debug_tokens'] = debug_tokens
            shared_data['debug_info'] = debug_text
            shared_data['low_light'] = low_light_mode
            shared_data['chaser_behind'] = chaser_behind
            shared_data['chaser_boxes'] = chaser_boxes

def send_controls_task():
    #This is where you send the control commands to the car using the control_conn
    global control_conn, tap_state, tap_timer, active_steering_value, smoothed_steering
    if control_conn is None: 
        return

    #these are the variables used to control the car
    #steering_input: -1.0 to 1.0 (left to right)
    #acceleration_input: -1.0 to 1.0 (reverse to forward)
    #this example always accelerate forward
    with state_lock:
        auto_steer = shared_data['steering_input']
        accel_input = shared_data['acceleration_input']
        is_emergency = shared_data.get('chaser_behind', False)

    if tap_state == 'IDLE':
        if auto_steer != 0.0:
            active_steering_value = auto_steer
            tap_state = 'TAPPING'
            tap_timer = EMERGENCY_TAP_HOLD_FRAMES if is_emergency else TAP_HOLD_FRAMES

            with state_lock:
                if auto_steer < -0.1: shared_data['net_lane_position'] = max(-2, shared_data.get('net_lane_position', 0) - 1)
                elif auto_steer > 0.1: shared_data['net_lane_position'] = min(2, shared_data.get('net_lane_position', 0) + 1)
        else: active_steering_value = 0.0
    elif tap_state == 'TAPPING':
        if tap_timer > 0: tap_timer -= 1
        else:
            active_steering_value = 0.0
            tap_state = 'COOLDOWN'
            tap_timer = EMERGENCY_COOLDOWN_FRAMES if is_emergency else COOLDOWN_FRAMES
    elif tap_state == 'COOLDOWN':
        active_steering_value = 0.0
        if tap_timer > 0: tap_timer -= 1
        else: tap_state = 'IDLE'

    ALPHA = 1.0 if is_emergency else 0.4
    smoothed_steering = (ALPHA * active_steering_value) + ((1.0 - ALPHA) * smoothed_steering)

    try: control_conn.sendall(struct.pack('ff', smoothed_steering, accel_input))
    except Exception: control_conn = None

# ---------------------------------------------------------
# Main (Scheduler Initialization)
# ---------------------------------------------------------
if __name__ == '__main__':
    print("Initializing Phase 1 RTSE Drive...")
    try: ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception: pass
    
    # Initialize network connections
    threading.Thread(target=setup_control_server, daemon=True).start()
    threading.Thread(target=setup_cameras, daemon=True).start()
    
    print("\n--- Starting Real-Time Tasks (awaiting connections dynamically) ---\n")
    
    # This is where you define tasks with explicit Scheduling parameters (Concurrency, Priority, Period)
    # Period refers to the period of execution of the task in seconds
    # Priority refers to the priority of the task, higher priority means higher priority
    # Concurrency refers to the number of instances of the task that can run at the same time
    t_front_camera = RTTask("ReadFrontCamera", period=0.01, priority=TaskPriority.LOW, execute_func=read_front_camera_task)
    t_back_camera = RTTask("ReadBackCamera", period=0.01, priority=TaskPriority.LOW, execute_func=read_back_camera_task)
    t_processing = RTTask("Processing", period=0.005, priority=TaskPriority.MEDIUM, execute_func=processing_task)
    t_controls = RTTask("SendControls", period=0.005, priority=TaskPriority.HIGH, execute_func=send_controls_task)
    
    # Start tasks to run concurrently
    t_front_camera.start()
    t_back_camera.start()
    t_processing.start()
    t_controls.start()
    
    if TESSERACT_AVAILABLE:
        t_ocr = RTTask("OCR_Scanner", period=0.3, priority=TaskPriority.LOW, execute_func=ocr_task)
        t_ocr.start() 
    
    display_paused = False
    last_display_frame = None

    print("\n=============================================")
    print(" PRESS 'p' TO PAUSE VIDEO FEED")
    print(" PRESS 'q' TO QUIT")
    print("=============================================\n")

    try:
        # You need this to keep the main thread alive, otherwise the program will exit immediately
        while is_running:
            with frame_lock:
                front_frame = shared_data['latest_front_frame']
                back_frame = shared_data.get('latest_back_frame', None)
            
            with state_lock:
                debug_info = shared_data['debug_info']
                debug_tokens = shared_data['debug_tokens'].copy()
                steer_input = shared_data['steering_input']
                accel_input = shared_data.get('acceleration_input', 1.0)
                low_light = shared_data.get('low_light', False)
                chaser_behind = shared_data.get('chaser_behind', False)
                chaser_boxes = shared_data.get('chaser_boxes', [])
                chaser_proximity = shared_data.get('chaser_proximity', 0.0)
                chaser_evade_end = shared_data.get('chaser_evade_end_time', 0.0)
                seek_red_end = shared_data.get('seek_red_end_time', 0.0)
                
                current_rel_lane = shared_data.get('net_lane_position', 0)
                road_bounds = shared_data.get('road_bounds', None)
                
                golden_lane_target = shared_data.get('golden_lane_target', 0)
                golden_lane_end = shared_data.get('golden_lane_end_time', 0.0)
                ocr_text = shared_data.get('debug_ocr_text', "")
                golden_mask = shared_data.get('debug_golden_mask')

            key = cv2.waitKey(1) & 0xFF
            if key in [ord('p'), ord(' ')]: display_paused = not display_paused
            elif key == ord('q'): is_running = False

            if front_frame is not None and not display_paused:
                display_front = cv2.resize(front_frame, (640, 480))
                
                # Combine ACCEL and AUTO texts side-by-side
                top_status_text = f"ACCEL: {accel_input:.2f} | {debug_info}"
                cv2.putText(display_front, top_status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                current_abs_lane = current_rel_lane + 3
                cv2.putText(display_front, f"CURRENT LANE: {current_abs_lane}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if road_bounds is not None:
                    x_left, x_right, lane_w, scan_y = road_bounds
                    
                    draw_y = (scan_y + ROI_START_Y) * 2
                    draw_xl, draw_xr = int(x_left * 2), int(x_right * 2)
                    draw_lw = int(lane_w * 2)
                    
                    cv2.circle(display_front, (draw_xl, draw_y), 5, (0, 0, 255), -1)
                    cv2.circle(display_front, (draw_xr, draw_y), 5, (0, 0, 255), -1)
                    cv2.line(display_front, (draw_xl, draw_y), (draw_xr, draw_y), (0, 255, 255), 3)
                    cv2.putText(display_front, "TRACK BOUNDARY (R/W)", (max(10, draw_xl), draw_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    for i in range(5):
                        seg_x = int(draw_xl + (i + 0.5) * draw_lw)
                        cv2.putText(display_front, f"L{i+1}", (seg_x - 15, draw_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        if i > 0:
                            div_x = int(draw_xl + i * draw_lw)
                            cv2.line(display_front, (div_x, draw_y - 10), (div_x, draw_y + 10), (255, 0, 255), 2)
                            
                    cv2.line(display_front, (320, draw_y - 40), (320, draw_y + 40), (0, 255, 0), 3)
                    cv2.putText(display_front, "CAR", (300, draw_y - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                time_left = seek_red_end - time.time()
                if time_left > 0: 
                    cv2.putText(display_front, f"SEEK RED MODE: {time_left:.1f}s", (120, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                
                golden_time_left = golden_lane_end - time.time()
                if golden_time_left > 0 and golden_lane_target > 0:
                    cv2.putText(display_front, f"GOLDEN DETECTED: L{golden_lane_target} ({golden_time_left:.1f}s)", (100, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 215, 255), 3)
                
                if low_light: 
                    cv2.putText(display_front, "LOW LIGHT DETECTED", (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)

                for token_data in debug_tokens:
                    if len(token_data) >= 5:
                        ttype, x, y, w, h = token_data[:5]
                        color = (255, 0, 0) if 'POLICE' in ttype else ((0, 0, 255) if ('DANGER' in ttype or 'RED' in ttype) else (0, 255, 0))
                        disp_x, disp_y = x * 2, (y + ROI_START_Y) * 2
                        disp_w, disp_h = w * 2, h * 2
                        cv2.rectangle(display_front, (disp_x, disp_y), (disp_x + disp_w, disp_y + disp_h), color, 2)
                        cv2.putText(display_front, ttype, (disp_x, disp_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                action_text = "STRAIGHT"
                if steer_input < -0.1: action_text = "<< STEER LEFT <<"
                elif steer_input > 0.1: action_text = ">> STEER RIGHT >>"
                else:
                    if golden_time_left > 0 and golden_lane_target > 0:
                        action_text = f"HOLDING AT LANE {golden_lane_target}"
                
                cv2.putText(display_front, f"ACTION: {action_text}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                if golden_mask is not None:
                    gm_bgr = cv2.cvtColor(cv2.resize(golden_mask, (320, 60)), cv2.COLOR_GRAY2BGR)
                    display_front[420:480, 0:320] = gm_bgr
                    cv2.putText(display_front, f"OCR EXTR: {ocr_text}", (5, 415), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                last_display_frame = display_front
                
            if back_frame is not None and not display_paused:
                display_back = cv2.resize(back_frame, (320, 240))
                display_back = cv2.flip(display_back, 1) 
                
                latched = time.time() < chaser_evade_end
                if chaser_behind or latched:
                    sweep = "RIGHT" if evade_dir > 0 else "LEFT"
                    cv2.putText(display_back, f"CHASER | prox {chaser_proximity:.2f} | SWEEP {sweep}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                    for (x, y, w, h) in chaser_boxes:
                        cv2.rectangle(display_back, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(display_back, "CHASER", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    
                cv2.imshow("Back Camera", display_back)

            if display_paused and last_display_frame is not None:
                pause_frame = last_display_frame.copy()
                cv2.putText(pause_frame, "PAUSED (Press 'p' to resume)", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                cv2.imshow("Front Camera", pause_frame)
            elif last_display_frame is not None:
                cv2.imshow("Front Camera", last_display_frame)
                
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt detected. Stopping system...")
        is_running = False

    # This is to make sure that the tasks are terminated cleanly
    t_front_camera.join()
    t_back_camera.join()
    t_processing.join()
    t_controls.join()
    if TESSERACT_AVAILABLE: t_ocr.join()
    
    # This is to close all the connections
    if front_camera_sock: front_camera_sock.close()
    if back_camera_sock: back_camera_sock.close()
    if control_conn: control_conn.close()
        
    try: ctypes.windll.winmm.timeEndPeriod(1)
    except Exception: pass
    cv2.destroyAllWindows()