# ------------------------------
# 실행 트리거
# ------------------------------
import os
import torch
from reid_model import load_osnet_model
from feature_builder import create_osnet_feature_database
from tracking_cam_udp import run_tracking

# 설정
core_feat_path = 'target_features_core.npy'
far_feat_path = 'target_features_far.npy'

if __name__ == "__main__":
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    osnet = load_osnet_model(device)

    # 특징 벡터 없으면 생성
    if not os.path.exists(core_feat_path):
        create_osnet_feature_database(
            model=osnet,
            device=device,
            image_dir="target_person_core",
            save_file=core_feat_path
        )

    if not os.path.exists(far_feat_path):
        create_osnet_feature_database(
            model=osnet,
            device=device,
            image_dir="target_person_far",
            save_file=far_feat_path
        )

    # 실시간 추적 시작
    run_tracking(
        core_feature_path=core_feat_path,
        far_feature_path=far_feat_path,
    )
