---
pretty_name: FFW-SH5 MuJoCo Can Color Sort
license: other
size_categories:
- n<1K
tags:
- robotics
- imitation-learning
- teleoperation
- mujoco
- act
- hdf5
- image
- timeseries
---

# FFW-SH5 MuJoCo Can Color Sort

FFW-SH5 MuJoCo 환경에서 오른팔 teleoperation으로 수집한 색상 분류 시연
데이터셋입니다. 캔을 집어 같은 분류군의 상자에 넣는 작업을 수행합니다.

이 카드는 코드 릴리즈 `v3.1.0`과 함께 검증되었습니다.

- 빨강·주황 캔 → 빨강 상자
- 초록·파랑 캔 → 파랑 상자
- 캔 위치, 캔 색상, 좌우 상자 색상 배치는 episode reset마다 무작위화

## Dataset summary

| 항목 | 값 |
|---|---:|
| Episodes | 150 |
| Successful episodes | 150 |
| Frames | 58,676 |
| Control frequency | 25 Hz |
| Green / Red / Orange / Blue | 50 / 47 / 24 / 29 |
| Simulator | MuJoCo |
| Robot | ROBOTIS AIWORKER FFW-SH5 |

`dataset_summary.json`과 `dataset_manifest.csv`에는 실제 업로드 파일에서 생성한
통계, episode별 frame 수, 색상, 목표 상자, 파일 크기와 SHA-256이 들어 있습니다.

## HDF5 schema

```text
episode_xxxxxx.hdf5
├── observations
│   ├── images
│   │   ├── cam_high                uint8 [T,H,W,3]
│   │   ├── cam_left_wrist          uint8 [T,H,W,3]
│   │   └── cam_right_wrist         uint8 [T,H,W,3]
│   ├── qpos                        float32 [T,16]
│   ├── qvel                        float32 [T,16]
│   └── ee_pose
│       ├── left                    float32 [T,7]
│       └── right                   float32 [T,7]
├── action                          float32 [T,16]
└── attrs
    ├── success, object_variant, target_label
    ├── control_hz, seed, schema_version
    └── ee_pose_frame=world, ee_pose_quaternion_order=wxyz
```

`qpos`와 `action`은 왼팔 7축, 왼손 grasp, 오른팔 7축, 오른손 grasp 순서의
16차원 벡터입니다. EE pose는 `[x, y, z, qw, qx, qy, qz]` 순서입니다.

## Loading example

```python
import h5py

with h5py.File("data/episode_000000.hdf5", "r") as episode:
    qpos = episode["observations/qpos"][:]
    right_ee_pose = episode["observations/ee_pose/right"][:]
    cam_high = episode["observations/images/cam_high"][:]
    action = episode["action"][:]
```

전체 데이터는 고정 revision으로 내려받을 수 있습니다.

```bash
hf download {{DATASET_REPO_ID}} --repo-type dataset \
  --revision v3.1.0 --local-dir datasets/can_color_sort_hf
```

MuJoCo에서는 RGB, joint state, EE pose와 action을 같은 25 Hz control tick에 저장합니다.
실제 로봇 데이터는 camera와 joint controller timestamp를 공통 clock에 맞춘 뒤 학습
시점으로 resampling해야 합니다.

원본 학습·검증 코드는 {{CODE_REPO_URL}}에서 확인할 수 있습니다.

## Intended use

- ACT 계열 imitation-learning 정책 학습
- Joint-space와 Task-space 표현 비교
- Temporal ensemble 및 PTE 추론 비교
- MuJoCo 기반 로봇 학습 파이프라인 연구·교육

## Limitations

- 시뮬레이션 데이터이며 실제 센서 노이즈와 동역학 차이를 포함하지 않습니다.
- 단일 로봇, 오른팔, 하나의 색상 분류 작업에서 수집했습니다.
- D97과 D150 비교에서는 데이터 수와 색상 다양성이 동시에 달라집니다.
- 실제 로봇에 적용하려면 좌표계, 카메라, 제어 주기와 안전 계층을 재검증해야 합니다.

## License

현재 데이터와 로봇/환경 asset의 재배포 라이선스를 최종 확인 중이므로
`license: other`로 표시했습니다. 공개 전 사용한 mesh와 texture의 배포 조건을
확인하십시오.
