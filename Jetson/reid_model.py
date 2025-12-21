# ------------------------------
# OSNet 로딩 및 전처리
# ------------------------------
import cv2
import torch
import numpy as np
from torchvision import transforms
from PIL import Image
from torchreid.models import build_model
from torchreid.utils import load_pretrained_weights

def load_osnet_model(device='cuda'):
    model = build_model(
        name='osnet_x1_0',
        num_classes=1041,
        loss='softmax',
        pretrained=False
    )
    load_pretrained_weights(model, '/home/moca7/projects/osnet_tracking/osnet_x1_0_msmt17.pth')
    model.to(device)
    model.eval()
    return model

transform = transforms.Compose([
    transforms.Resize((256, 128)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225])
])

def preprocess(img_bgr):
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    return transform(pil_img).unsqueeze(0)  # (1, 3, 256, 128)
