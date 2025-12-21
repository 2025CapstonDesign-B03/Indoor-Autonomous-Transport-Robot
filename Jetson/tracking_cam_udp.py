# ------------------------------
# 실시간 추적 함수 (core + far)
# ------------------------------
import cv2
import numpy as np
import os
import torch
import math
import time
import socket
import pyrealsense2 as rs
from ultralytics import YOLO
from reid_model import load_osnet_model, preprocess

def run_tracking(core_feature_path='target_features_core.npy',
                 far_feature_path='target_features_far.npy'):
    
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    FAR_WEIGHT = 0.8
    SIMILARITY_THRESHOLD = 0.45
    CONFIRM_FRAMES = 3
    MAX_COUNTER = 5

    osnet = load_osnet_model(device)

    if not os.path.exists(core_feature_path) or not os.path.exists(far_feature_path):
        raise FileNotFoundError("[ERROR] 특징 벡터 파일이 없습니다. .npy 파일을 먼저 생성해야 합니다.")

    core_features = np.load(core_feature_path)
    far_features = np.load(far_feature_path)

    model = YOLO("yolo11s.engine")

    # RealSense 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    # 필터
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter()

    # 필터 파라미터 조정
    spatial.set_option(rs.option.filter_magnitude, 3)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)

    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)

    time.sleep(2)

    PI_IP = "192.168.10.2"   # Raspberry Pi 5의 IP로 변경
    PORT = 5000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (PI_IP, PORT)

    print("[INFO] OSNet + YOLO + RealSense 기반 추적 시작")

    target_confirmed_counter = 0
    target_tracking = False

    while True:
        # 프레임 수신
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 필터 적용
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = hole.process(depth_frame)
        depth_frame = depth_frame.as_depth_frame()

        frame = np.asanyarray(color_frame.get_data())
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        results = model(frame, imgsz=640)
        result = results[0]

        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)

            best_box = None
            best_score = 0

            for i, box in enumerate(boxes):
                if classes[i] != 0:
                    continue  # person only

                x1, y1, x2, y2 = map(int, box[:4])
                roi = frame[y1:y2, x1:x2]

                if roi.shape[0] < 50 or roi.shape[1] < 30:
                    continue

                with torch.no_grad():
                    input_tensor = preprocess(roi).to(device)
                    feature = osnet(input_tensor).cpu().numpy().flatten()
                feature = feature / np.linalg.norm(feature)

                sim_core = max(np.dot(feature, tf) for tf in core_features)
                sim_far = max(np.dot(feature, tf) for tf in far_features)
                score = max(sim_core, FAR_WEIGHT * sim_far)

                if score > best_score:
                    best_score = score
                    best_box = (x1, y1, x2, y2)

            if best_score > SIMILARITY_THRESHOLD:
                target_confirmed_counter = min(MAX_COUNTER, target_confirmed_counter + 1)
            else:
                target_confirmed_counter = max(0, target_confirmed_counter - 1)

            target_tracking = target_confirmed_counter >= CONFIRM_FRAMES
            
            depth = None
            azimuth = None

            if target_tracking and best_box is not None:
                x1, y1, x2, y2 = best_box
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                depth = depth_frame.get_distance(cx, cy)

                if depth > 0:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                    azimuth = math.degrees(math.atan2(X, Z))
                    
                    message = f"{depth:.2f},{azimuth:.2f}"
                    sock.sendto(message.encode(), server_address)

                    print("[SEND]", message)
                
    pipeline.stop()
