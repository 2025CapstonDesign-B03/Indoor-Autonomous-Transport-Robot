# ------------------------------
# 폴더 내 특징 벡터 생성 (YOLO crop)
# ------------------------------
import os
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from reid_model import preprocess

def create_osnet_feature_database(model, device, image_dir, save_file):
    
    yolo = YOLO("yolo11s.engine")
    features = []

    for fname in os.listdir(image_dir):
        path = os.path.join(image_dir, fname)
        img = cv2.imread(path)
        if img is None:
            continue

        results = yolo(img, imgsz=640)
        if results and results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)

            for i, box in enumerate(boxes):
                if classes[i] != 0:  # only person class
                    continue

                x1, y1, x2, y2 = map(int, box[:4])
                roi = img[y1:y2, x1:x2]

                if roi.shape[0] < 50 or roi.shape[1] < 30:
                    print(f"[SKIP] ROI too small (h={roi.shape[0]}, w={roi.shape[1]}) → {fname}")
                    continue

                with torch.no_grad():
                    input_tensor = preprocess(roi).to(device)
                    feat = model(input_tensor).cpu().numpy().flatten()
                    feat = feat / np.linalg.norm(feat)
                    features.append(feat)

    np.save(save_file, features)
    print(f"[INFO] '{save_file}' 저장 완료. 특징 벡터 수: {len(features)}")
