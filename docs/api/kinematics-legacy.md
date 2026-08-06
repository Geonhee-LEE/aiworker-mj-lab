# `kinematics.legacy`

이전 `KinematicsSolver`와 `InverseKinematics` 이름을 위한 최소 FK adapter다.
`forward()`와 `forward_kinematics()`만 제공하며 반복형 pose solve API는 제거되었다.

새 실시간 코드는 [`KinematicTree.forward_site()`](kinematics-tree.md)를 직접 사용한다.
이 모듈에는 새 기능을 추가하지 않는다.
