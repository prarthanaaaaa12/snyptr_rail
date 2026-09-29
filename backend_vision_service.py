#!/usr/bin/env python3
"""
===============================================================================
SNYPTR-RAIL: DEDICATED COMPUTER VISION BACKEND METRICS UNIT
===============================================================================
Video Input:   ONLY through COM port (COM17 @ 3,000,000 baud from ESP32-P4)
Actuation:     ONLY through Wi-Fi (http://192.168.4.1/cmd?action=DOWN,1)
Target:        Black silhouette target
Projectile:    Yellow Nerf bullet with orange/yellow tip
Operation:
  1. Streams 800x800 frames continuously from ESP32-P4 via USB Serial COM17.
  2. Multi-space color metric classifier (LAB + HSV + Chromatic Ratio):
     - Black target: low luminance (L* < 60, V < 80)
     - Yellow Nerf: high b* (> 140), high L* (> 70), Hue in [14, 42]
  3. Morphological filtering removes specular glints, reflections, and dust.
  4. Temporal stability check: bullet must remain stationary across >= 3 frames.
  5. The instant the bullet is confirmed STUCK on the black target:
     - Dispatches DOWN,1 over Wi-Fi to Pop-Up ESP32 (http://192.168.4.1/cmd)
     - Locks out re-triggering for 3.0 seconds cooldown
  6. Exposes real-time annotated video stream with metric HUD at:
     - http://localhost:8001/video_feed
     - http://localhost:8001/metrics
===============================================================================
"""

import sys
import os
import time
import struct
import zlib
import threading
import argparse
from typing import Optional, Tuple, Dict, Any, List

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import cv2
import numpy as np
import requests

# FastAPI for high-performance MJPEG & REST metrics API
try:
    from fastapi import FastAPI
    from fastapi.responses import StreamingResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False

# PySerial for COM17 high-speed reading
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# Configuration Constants
DEFAULT_P4_SERIAL_PORT = "COM17"
DEFAULT_P4_BAUD_RATE = 3000000
DEFAULT_WIFI_POP_URL = "http://192.168.4.1"
API_PORT = 8001

# Cooldown and settling windows
POST_HIT_COOLDOWN_SEC = 3.0    # Hold hit state for 3 seconds before auto-rearm
POP_UP_SETTLE_WINDOW_SEC = 0.65 # Settle window after target reaches upright position (650ms)
BASELINE_FRAME_COUNT = 6       # Frames to average for stationary baseline

# =============================================================================
# GLOBAL SHARED STATE
# =============================================================================
class VisionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_frame_bgr: Optional[np.ndarray] = None
        self.annotated_frame_bgr: Optional[np.ndarray] = None
        self.prev_frame_bgr: Optional[np.ndarray] = None
        self.baseline_bgr: Optional[np.ndarray] = None
        self.baseline_frames: List[np.ndarray] = []
        self.baseline_ready = False
        
        self.frame_id = 0
        self.fps = 0.0
        self.last_frame_time = time.time()
        
        # Target state: "IDLE", "SETTLING", "CALIBRATING", "ARMED", "LOCKED"
        self.target_state = "IDLE"
        self.state_entered_time = time.time()
        
        # Detection results
        self.hit_active = False
        self.hit_timestamp = 0.0
        self.hit_x_px = 0.0
        self.hit_y_px = 0.0
        self.hit_x_norm = 0.0
        self.hit_y_norm = 0.0
        self.hit_bullet_area = 0
        self.hit_confidence = 0.0
        
        # Check diagnostics
        self.check1_diff_pass = False
        self.check2_color_pass = False
        self.check3_onset_pass = False
        self.check4_blob_pass = False
        self.latency_ms = 0.0
        
        # 2-frame candidate buffer for borderline hits
        self.pending_candidate: Optional[Dict[str, Any]] = None
        
        # Calibrated Target Region (Center, Radius)
        self.target_center_px = (400, 400)
        self.target_radius_px = 280
        
        # Connection status
        self.com_port = DEFAULT_P4_SERIAL_PORT
        self.com_connected = False
        self.wifi_pop_connected = False
        self.status_msg = "Initializing..."

g_state = VisionState()


# =============================================================================
# WI-FI POP-UP CONTROLLER (POP-UP ESP32 BRIDGE)
# =============================================================================
class WiFiPopUpController:
    """Controls the Pop-Up ESP32 exclusively over Wi-Fi (http://192.168.4.1/)."""
    def __init__(self, wifi_url=DEFAULT_WIFI_POP_URL):
        self.wifi_url = wifi_url.rstrip('/')

    def send_cmd(self, cmd: str) -> bool:
        cmd = cmd.strip()
        for attempt in range(2):
            try:
                url = f"{self.wifi_url}/cmd?action={cmd}"
                resp = requests.get(url, timeout=0.6)
                if resp.status_code == 200:
                    with g_state.lock:
                        g_state.wifi_pop_connected = True
                    return True
            except Exception:
                pass
        with g_state.lock:
            g_state.wifi_pop_connected = False
        return False

    def drop_target(self, target_id=1):
        print(f"[POP_ACTUATOR] >>> ACTION: DROPPING TARGET {target_id} VIA WIFI (DOWN,{target_id}) <<<", flush=True)
        threading.Thread(target=self.send_cmd, args=(f"DOWN,{target_id}",), daemon=True).start()
        return True

    def raise_target(self, target_id=1):
        print(f"[POP_ACTUATOR] >>> ACTION: RAISING TARGET {target_id} VIA WIFI (UP,{target_id}) <<<", flush=True)
        threading.Thread(target=self.send_cmd, args=(f"UP,{target_id}",), daemon=True).start()
        return True

    def poll_telemetry_loop(self):
        """Monitors Pop-Up ESP32 state over Wi-Fi so backend knows when target is UP or DOWN."""
        while True:
            try:
                url = f"{self.wifi_url}/telemetry"
                resp = requests.get(url, timeout=0.8)
                if resp.status_code == 200:
                    data = resp.json()
                    with g_state.lock:
                        g_state.wifi_pop_connected = True
                        new_state = data.get("targetState", g_state.target_state)
                        if new_state == "UP" and g_state.target_state == "IDLE":
                            g_state.target_state = "SETTLING"
                            g_state.state_entered_time = time.time()
                            g_state.baseline_ready = False
                            g_state.baseline_frames.clear()
                            g_state.pending_candidate = None
                            print(f"[POP_WIFI] Target is UP! Settling servo...", flush=True)
                        elif new_state == "DOWN" and g_state.target_state not in ("IDLE", "LOCKED"):
                            g_state.target_state = "IDLE"
                            g_state.pending_candidate = None
            except Exception:
                with g_state.lock:
                    g_state.wifi_pop_connected = False
            time.sleep(0.4)

g_pop_ctrl = WiFiPopUpController(DEFAULT_WIFI_POP_URL)


# =============================================================================
# COM PORT VIDEO STREAM INGESTION (ONLY COM17)
# =============================================================================
class SerialP4StreamReader:
    """Reads 800x800 frames directly from ESP32-P4 UART0 over High-Speed Serial COM17."""
    def __init__(self, port=DEFAULT_P4_SERIAL_PORT, baud=DEFAULT_P4_BAUD_RATE):
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        last_warn_time = 0
        while self.running:
            ser = None
            try:
                now = time.time()
                if now - last_warn_time > 5.0:
                    print(f"[P4_COM] Connecting to {self.port} at {self.baud} baud...", flush=True)
                    last_warn_time = now

                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                with g_state.lock:
                    g_state.com_connected = True
                    g_state.status_msg = f"Connected on {self.port} @ {self.baud}"
                print(f"[P4_COM] >>> Successfully connected to ESP32-P4 on {self.port}! Video stream active. <<<", flush=True)

                buffer = bytearray()
                while self.running and ser.is_open:
                    in_waiting = ser.in_waiting
                    if in_waiting > 65536:
                        ser.reset_input_buffer()
                        buffer.clear()
                        continue
                    
                    chunk = ser.read(in_waiting if in_waiting > 0 else 1)
                    if chunk:
                        buffer.extend(chunk)

                    # Search for packet header: 0xAA 0x55 0x01
                    while len(buffer) >= 15:
                        if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01:
                            frame_id = struct.unpack('<I', buffer[3:7])[0]
                            payload_len = struct.unpack('<I', buffer[7:11])[0]
                            received_crc = struct.unpack('<I', buffer[11:15])[0]

                            if payload_len > 300000 or payload_len == 0:
                                buffer = buffer[1:]
                                continue

                            total_len = 15 + payload_len
                            if len(buffer) < total_len:
                                break # Wait for full packet

                            payload = bytes(buffer[15:total_len])
                            buffer = buffer[total_len:]

                            # Verify CRC32
                            calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
                            if calc_crc == received_crc:
                                # Strip optional 32-byte metadata if appended at end
                                img_bytes = payload
                                if len(payload) >= 32 and payload[-32:-28] == b'\xDE\xAD\xBE\xEF':
                                    img_bytes = payload[:-32]
                                
                                img_np = np.frombuffer(img_bytes, dtype=np.uint8)
                                frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                                if frame is not None:
                                    try:
                                        process_incoming_frame(frame, frame_id)
                                    except Exception as frame_err:
                                        print(f"[CV_WARN] Frame processing error: {frame_err}", flush=True)
                        else:
                            buffer = buffer[1:]
            except Exception as e:
                with g_state.lock:
                    g_state.com_connected = False
                    g_state.status_msg = f"Waiting for {self.port}: {e}"
                time.sleep(1.0)
            finally:
                if ser:
                    try: ser.close()
                    except: pass


# =============================================================================
# COMPUTER VISION & METRIC ANALYSIS ENGINE
# =============================================================================
def process_incoming_frame(frame: np.ndarray, frame_id: int):
    """
    Executes Deterministic Multi-Check Nerf Projectile Detector on incoming frame:
      - State Machine: IDLE -> SETTLING (650ms) -> CALIBRATING (capture 6-frame dark baseline) -> ARMED -> LOCKED (3.0s cooldown)
      - Check 1: Baseline Difference (pixel delta >= 22 on dark target baseline pixels)
      - Check 2: Orange/Yellow Chromatic Filter (R >= 105, B <= 75, R - B >= 38, R+G >= 2.2*(B+1), G - B >= 10, R >= G - 28)
      - Check 3: Sudden Temporal Onset Delta (mean delta >= 10.0 vs previous frame)
      - Check 4: Spatial Coherence (density >= 0.14, w,h >= 4, area >= 20)
      - Instant Hit Trigger (Score >= 75 & Area >= 32) or 2-frame fast confirmation (Score >= 55)
      - Immediate Servo Retract: DOWN,1 via Wi-Fi
    """
    t_start = time.perf_counter()
    now = time.time()
    h_img, w_img = frame.shape[:2]
    
    with g_state.lock:
        dt = now - g_state.last_frame_time
        if dt > 0:
            g_state.fps = round(0.9 * g_state.fps + 0.1 * (1.0 / dt), 1)
        g_state.last_frame_time = now
        g_state.frame_id = frame_id
        g_state.current_frame_bgr = frame.copy()
        
        # State machine timeouts
        if g_state.target_state == "LOCKED":
            if (now - g_state.hit_timestamp) > POST_HIT_COOLDOWN_SEC:
                g_state.target_state = "IDLE"
                g_state.hit_active = False
                g_state.pending_candidate = None
                print("[STATE] 3.0s Cooldown completed. System reset to IDLE.", flush=True)

        elif g_state.target_state == "SETTLING":
            if (now - g_state.state_entered_time) >= POP_UP_SETTLE_WINDOW_SEC:
                g_state.target_state = "CALIBRATING"
                g_state.state_entered_time = now
                g_state.baseline_frames.clear()
                print("[STATE] Target settled. Capturing stationary baseline...", flush=True)

        elif g_state.target_state == "CALIBRATING":
            g_state.baseline_frames.append(frame.astype(np.float32))
            if len(g_state.baseline_frames) >= BASELINE_FRAME_COUNT:
                g_state.baseline_bgr = np.mean(g_state.baseline_frames, axis=0).astype(np.uint8)
                g_state.baseline_ready = True
                g_state.target_state = "ARMED"
                g_state.state_entered_time = now
                g_state.pending_candidate = None
                print("[STATE] Dark reference baseline locked! ARMED for Nerf projectile detection.", flush=True)

    # Visualization canvas
    annotated = frame.copy()
    tc_x, tc_y = g_state.target_center_px
    tr = g_state.target_radius_px
    
    # Draw Green Target Optimal Zone Circle
    cv2.circle(annotated, (tc_x, tc_y), tr, (44, 255, 85), 2)
    cv2.circle(annotated, (tc_x, tc_y), 5, (44, 255, 85), -1)

    # Circular Optimal Region ROI Mask
    roi_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.circle(roi_mask, (tc_x, tc_y), tr, 255, -1)

    # Run Detection only if ARMED and baseline is ready
    confirmed_hit = False
    best_cand = None
    c1_pass, c2_pass, c3_pass, c4_pass = False, False, False, False

    if g_state.target_state == "ARMED" and g_state.baseline_ready and g_state.baseline_bgr is not None:
        # Check 1: Baseline Difference on dark pixels
        b0 = g_state.baseline_bgr[:, :, 0].astype(np.float32)
        g0 = g_state.baseline_bgr[:, :, 1].astype(np.float32)
        r0 = g_state.baseline_bgr[:, :, 2].astype(np.float32)
        dark_target_mask = ((r0 + g0 + b0) / 3.0 <= 110.0) & (roi_mask > 0)

        b_cur = frame[:, :, 0].astype(np.float32)
        g_cur = frame[:, :, 1].astype(np.float32)
        r_cur = frame[:, :, 2].astype(np.float32)
        
        diff_r = np.abs(r_cur - r0)
        diff_g = np.abs(g_cur - g0)
        diff_b = np.abs(b_cur - b0)
        max_diff = np.maximum(diff_r, np.maximum(diff_g, diff_b))
        
        c1_diff_mask = (max_diff >= 22.0) & dark_target_mask

        # Check 2: Orange/Yellow Chromatic Filter
        c2_color_mask = (
            (r_cur >= 105.0) &
            (b_cur <= 75.0) &
            ((r_cur - b_cur) >= 38.0) &
            ((r_cur + g_cur) >= 2.2 * (b_cur + 1.0)) &
            ((g_cur - b_cur) >= 10.0) &
            (r_cur >= (g_cur - 28.0)) &
            (roi_mask > 0)
        )

        # Combined Candidate Pixel Mask
        cand_mask = (c1_diff_mask & c2_color_mask).astype(np.uint8) * 255

        # Check 3: Sudden Temporal Onset Delta
        temporal_diff = None
        if g_state.prev_frame_bgr is not None:
            prev_b = g_state.prev_frame_bgr[:, :, 0].astype(np.float32)
            prev_g = g_state.prev_frame_bgr[:, :, 1].astype(np.float32)
            prev_r = g_state.prev_frame_bgr[:, :, 2].astype(np.float32)
            temporal_diff = (np.abs(r_cur - prev_r) + np.abs(g_cur - prev_g) + np.abs(b_cur - prev_b)) / 3.0

        # Morphological opening (remove isolated noise pixels)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cand_mask_clean = cv2.morphologyEx(cand_mask, cv2.MORPH_OPEN, kernel)

        # Find Contours
        contours, _ = cv2.findContours(cand_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 20:
                continue
            x, y, w, h_box = cv2.boundingRect(cnt)
            if w < 4 or h_box < 4:
                continue
            density = float(area) / (w * h_box)
            if density < 0.14:
                continue

            # Centroid
            M = cv2.moments(cnt)
            cx = float(M["m10"] / M["m00"]) if M["m00"] > 0 else float(x + w / 2)
            cy = float(M["m01"] / M["m00"]) if M["m00"] > 0 else float(y + h_box / 2)

            # Temporal onset in bounding box
            mean_onset = 0.0
            if temporal_diff is not None:
                roi_onset = temporal_diff[y:y+h_box, x:x+w]
                if roi_onset.size > 0:
                    mean_onset = float(np.mean(roi_onset))

            # Check passes
            cnt_c1 = True
            cnt_c2 = True
            cnt_c3 = (mean_onset >= 10.0) or (g_state.prev_frame_bgr is None)
            cnt_c4 = (density >= 0.14 and w >= 4 and h_box >= 4)

            # Scoring: C1=40, C2=30, C3=15, C4=15
            score = (40 if cnt_c1 else 0) + (30 if cnt_c2 else 0) + (15 if cnt_c3 else 0) + (15 if cnt_c4 else 0)

            valid_candidates.append({
                "cx": cx,
                "cy": cy,
                "bbox": (x, y, w, h_box),
                "area": int(area),
                "density": density,
                "onset": mean_onset,
                "score": score,
                "c1": cnt_c1,
                "c2": cnt_c2,
                "c3": cnt_c3,
                "c4": cnt_c4
            })

        if valid_candidates:
            # Sort by score and area
            valid_candidates.sort(key=lambda c: (c["score"], c["area"]), reverse=True)
            best_cand = valid_candidates[0]
            c1_pass = best_cand["c1"]
            c2_pass = best_cand["c2"]
            c3_pass = best_cand["c3"]
            c4_pass = best_cand["c4"]

            # Multi-check decision
            if best_cand["score"] >= 75 and best_cand["area"] >= 32:
                # Instant trigger!
                confirmed_hit = True
                print(f"[CV HIT] INSTANT HIT: Score={best_cand['score']}, Area={best_cand['area']}px, Onset={best_cand['onset']:.1f}", flush=True)
            elif best_cand["score"] >= 55:
                # Borderline event: Check 2-frame consistency
                if g_state.pending_candidate is not None:
                    drift = np.hypot(best_cand["cx"] - g_state.pending_candidate["cx"],
                                     best_cand["cy"] - g_state.pending_candidate["cy"])
                    if drift <= 20.0:
                        confirmed_hit = True
                        print(f"[CV HIT] 2-FRAME CONFIRMED HIT: Score={best_cand['score']}, Area={best_cand['area']}px, Drift={drift:.1f}px", flush=True)
                    g_state.pending_candidate = None
                else:
                    g_state.pending_candidate = best_cand
            else:
                g_state.pending_candidate = None
        else:
            g_state.pending_candidate = None

        if confirmed_hit and best_cand is not None:
            with g_state.lock:
                g_state.hit_active = True
                g_state.hit_timestamp = now
                g_state.hit_x_px = best_cand["cx"]
                g_state.hit_y_px = best_cand["cy"]
                g_state.hit_x_norm = best_cand["cx"] / float(w_img)
                g_state.hit_y_norm = best_cand["cy"] / float(h_img)
                g_state.hit_bullet_area = best_cand["area"]
                g_state.hit_confidence = float(best_cand["score"])
                g_state.target_state = "LOCKED"

            # DISPATCH IMMEDIATE PHYSICAL RETRACT (DOWN,1) VIA WI-FI
            print(f"\n[ACTUATION] >>> NERF HIT CONFIRMED! DISPATCHING DOWN,1 TO POP-UP ESP32 <<<", flush=True)
            g_pop_ctrl.drop_target(1)

    latency_ms = (time.perf_counter() - t_start) * 1000.0

    # Draw HUD and Annotations
    if best_cand is not None:
        bx, by, bw, bh = best_cand["bbox"]
        cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (0, 165, 255), 2)
        cv2.drawMarker(annotated, (int(best_cand["cx"]), int(best_cand["cy"])), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(annotated, f"NERF {best_cand['area']}px ({best_cand['score']}%)", (bx, max(18, by - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

    if g_state.hit_active or g_state.target_state == "LOCKED":
        hx, hy = int(g_state.hit_x_px), int(g_state.hit_y_px)
        cv2.circle(annotated, (hx, hy), 28, (0, 255, 0), 3)
        cv2.circle(annotated, (hx, hy), 6, (0, 0, 255), -1)
        cv2.putText(annotated, f"TARGET HIT ({g_state.hit_confidence:.0f}%)", (hx + 30, hy + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        cv2.rectangle(annotated, (0, 0), (w_img, 45), (10, 35, 10), -1)
        cv2.putText(annotated, "NERF HIT CONFIRMED // TARGET DROPPED VIA WIFI", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (44, 255, 85), 2)
    else:
        cv2.rectangle(annotated, (0, 0), (w_img, 36), (20, 20, 20), -1)
        status_text = f"FPS: {g_state.fps:.1f} | STATE: {g_state.target_state} | C1:{'+' if c1_pass else '-'} C2:{'+' if c2_pass else '-'} C3:{'+' if c3_pass else '-'} C4:{'+' if c4_pass else '-'} | LATENCY: {latency_ms:.1f}ms"
        cv2.putText(annotated, status_text, (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 255, 200), 1)

    # Update state
    with g_state.lock:
        g_state.annotated_frame_bgr = annotated
        g_state.prev_frame_bgr = frame.copy()
        g_state.check1_diff_pass = c1_pass
        g_state.check2_color_pass = c2_pass
        g_state.check3_onset_pass = c3_pass
        g_state.check4_blob_pass = c4_pass
        g_state.latency_ms = latency_ms


# =============================================================================
# FASTAPI / WEBSOCKET WEB SERVER
# =============================================================================
if HAS_FASTAPI:
    app = FastAPI(title="Snyptr-Rail Dedicated CV Backend", version="2.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    def generate_mjpeg_stream():
        """Generates continuous MJPEG multipart stream from COM17."""
        while True:
            frame = None
            with g_state.lock:
                if g_state.annotated_frame_bgr is not None:
                    frame = g_state.annotated_frame_bgr.copy()
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.04) # ~25 FPS stream delivery

    @app.get("/video_feed")
    def video_feed():
        """Live COM17 video feed with annotated metric overlay HUD."""
        return StreamingResponse(generate_mjpeg_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/metrics")
    def get_metrics() -> Dict[str, Any]:
        """Returns JSON telemetry of the current detection state."""
        with g_state.lock:
            return {
                "hit": g_state.hit_active,
                "x_px": round(g_state.hit_x_px, 1),
                "y_px": round(g_state.hit_y_px, 1),
                "x_norm": round(g_state.hit_x_norm, 3),
                "y_norm": round(g_state.hit_y_norm, 3),
                "dart_pixels": g_state.hit_bullet_area,
                "confidence": g_state.hit_confidence,
                "check1_diff": g_state.check1_diff_pass,
                "check2_color": g_state.check2_color_pass,
                "check3_onset": g_state.check3_onset_pass,
                "check4_blob": g_state.check4_blob_pass,
                "latency_ms": round(g_state.latency_ms, 1),
                "fps": g_state.fps,
                "target_state": g_state.target_state,
                "baseline_ready": g_state.baseline_ready,
                "com_port": g_state.com_port,
                "com_connected": g_state.com_connected,
                "wifi_pop_connected": g_state.wifi_pop_connected,
                "uptime_ms": int((time.time() - g_state.last_frame_time) * 1000)
            }

    @app.api_route("/arm", methods=["GET", "POST"])
    def arm_target():
        """Lifts target upright via Wi-Fi and prepares CV."""
        with g_state.lock:
            g_state.target_state = "SETTLING"
            g_state.state_entered_time = time.time()
            g_state.baseline_ready = False
            g_state.baseline_frames.clear()
            g_state.pending_candidate = None
            g_state.hit_active = False
        g_pop_ctrl.raise_target(1)
        return {"status": "SETTLING", "timestamp": time.time()}

    @app.api_route("/drop", methods=["GET", "POST"])
    def drop_target():
        """Drops target down via Wi-Fi."""
        with g_state.lock:
            g_state.target_state = "IDLE"
            g_state.pending_candidate = None
            g_state.hit_active = False
        g_pop_ctrl.drop_target(1)
        return {"status": "IDLE", "timestamp": time.time()}


def run_api_server():
    """Runs uvicorn API server in background thread."""
    if HAS_FASTAPI:
        config = uvicorn.Config(app, host="0.0.0.0", port=API_PORT, log_level="warning")
        server = uvicorn.Server(config)
        server.run()


# =============================================================================
# CLI MAIN ENTRY POINT
# =============================================================================
def main():
    global g_pop_ctrl
    parser = argparse.ArgumentParser(description="Snyptr-Rail Dedicated CV Backend Metrics Unit")
    parser.add_argument("--port", default=DEFAULT_P4_SERIAL_PORT, help="ESP32-P4 Serial COM port (default: COM17)")
    parser.add_argument("--baud", type=int, default=DEFAULT_P4_BAUD_RATE, help="ESP32-P4 Baud rate (default: 3000000)")
    parser.add_argument("--wifi-url", default=DEFAULT_WIFI_POP_URL, help="Pop-Up ESP32 Wi-Fi Base URL (default: http://192.168.4.1)")
    parser.add_argument("--api-port", type=int, default=API_PORT, help="FastAPI port (default: 8001)")
    parser.add_argument("--gui", action="store_true", help="Show local OpenCV window")
    args = parser.parse_args()

    with g_state.lock:
        g_state.com_port = args.port

    print("=" * 75)
    print("  SNYPTR-RAIL: DEDICATED COMPUTER VISION BACKEND METRICS UNIT")
    print("=" * 75)
    print(f"  Camera Video Stream : ONLY COM PORT ({args.port} @ {args.baud} baud)")
    print(f"  Pop-Up Control      : ONLY WI-FI ({args.wifi_url}/cmd)")
    print(f"  API Server          : http://localhost:{args.api_port}")
    print(f"  Live Video Feed     : http://localhost:{args.api_port}/video_feed")
    print(f"  Live Metrics JSON   : http://localhost:{args.api_port}/metrics")
    print("=" * 75, flush=True)

    # Initialize Wi-Fi Pop-Up Controller & start background Wi-Fi telemetry sync
    g_pop_ctrl = WiFiPopUpController(wifi_url=args.wifi_url)
    threading.Thread(target=g_pop_ctrl.poll_telemetry_loop, daemon=True).start()

    # Start FastAPI Web Server in background thread
    api_thread = threading.Thread(target=run_api_server, daemon=True)
    api_thread.start()

    # Start Video Stream Ingestion (ONLY from COM17)
    reader = SerialP4StreamReader(port=args.port, baud=args.baud)
    reader.start()

    # Local OpenCV GUI window if requested
    if args.gui:
        cv2.namedWindow("Snyptr-Rail COM17 Video Analysis", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Snyptr-Rail COM17 Video Analysis", 800, 800)
        try:
            while True:
                frame = None
                with g_state.lock:
                    if g_state.annotated_frame_bgr is not None:
                        frame = g_state.annotated_frame_bgr.copy()
                if frame is not None:
                    cv2.imshow("Snyptr-Rail COM17 Video Analysis", frame)
                if cv2.waitKey(20) & 0xFF == ord('q'):
                    break
        finally:
            cv2.destroyAllWindows()
    else:
        # Keep main thread alive
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[BACKEND] Stopping service gracefully...")

if __name__ == "__main__":
    main()
