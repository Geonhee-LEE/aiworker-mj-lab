# aiworker-mj-lab

FFW-SH5 양팔 모바일 로봇을 MuJoCo 물리에서 조작하는 텔레오퍼레이션 프로젝트다.
손 목표를 지정하면 전신 IK가 팔·리프트·스워브 베이스 명령을 계산하고, 실제 actuator와
접촉 물리가 로봇을 움직인다. ROS는 사용하지 않는다.

[빠른 시작](getting-started.md){ .md-button .md-button--primary }
[화면과 조작](run.md){ .md-button }
[모방학습](guide/il/index.md){ .md-button }
[시스템 구조](overview.md){ .md-button }

<figure class="hero-figure" markdown>
  ![양팔, 리프트, 스워브 베이스와 캔이 포함된 MuJoCo 전체 장면](assets/hero.jpg)
  <figcaption>`full_scene.xml`의 로봇과 작업 공간.</figcaption>
</figure>

## 어디서 시작할까

<div class="grid cards" markdown>

-   :material-rocket-launch: **로봇을 실행하고 싶다**

    설치, headless 검사와 첫 조작을 순서대로 진행한다.

    [빠른 시작](getting-started.md)

-   :material-gamepad-variant: **조작법을 찾고 싶다**

    키보드, Task Space 입력, IK/FK와 Whole-body 모드를 확인한다.

    [화면과 조작](run.md) · [모드 선택](control-modes.md)

-   :material-source-branch: **구현을 이해하고 싶다**

    애플리케이션부터 기구학과 제어까지 코드 흐름을 따라간다.

    [시스템 이해와 개발](guide/index.md)

-   :material-code-braces: **함수를 찾고 싶다**

    공개 함수의 입력, 반환값과 데이터 변경 여부를 패키지별로 찾는다.

    [API 레퍼런스](api/index.md)

-   :material-robot-industrial: **ACT 정책을 학습하고 싶다**

    색상 분류 데이터를 수집하고 Joint/Task 정책과 PTE를 같은 조건에서 비교한다.

    [연구 개요](research-report.md) · [학습과 평가](modular-act-training.md) ·
    [공개 모델과 데이터](huggingface.md)

</div>

문제가 발생했다면 [증상별 문제 해결](troubleshooting.md)에서 바로 찾을 수 있다.

## 동작 흐름

```mermaid
flowchart LR
    INPUT["키보드 · UI · 3D Gizmo"] --> TARGET["손·가상 물체 목표"]
    TARGET --> IK["전신 또는 팔 전용 IK"]
    IK --> CTRL["팔 · 리프트 · 스워브 · 손 명령"]
    CTRL --> MJ["MuJoCo 물리"]
    MJ --> STATE["실제 pose · 속도 · 접촉"]
    STATE --> IK
```

UI는 로봇 관절을 순간 이동시키지 않는다. 목표값을 바꾸면 solver와 controller가
명령을 계산하고, `mujoco.mj_step()`이 다음 실제 상태를 만든다.

## 주요 기능

<div class="grid cards" markdown>

-   **전신과 팔 전용 IK**

    base x/y/yaw, lift와 양팔 14축을 함께 풀거나 base·lift를 고정하고 팔만 푼다.

-   **양손 목표 조작**

    독립 MoveL, world XYZ/RPY 입력, 3D Gizmo와 virtual object 기반 양손 이동을 제공한다.

-   **충돌 대응과 실제 파지**

    가까운 충돌에는 velocity CBF로 반응하고, 물체는 weld 없이 손가락 접촉력과 마찰로 잡는다.

-   **스워브 모바일 베이스**

    차체 속도를 실제 steer/drive actuator 명령으로 바꾸고 wheel-ground contact로 이동한다.

-   **ACT 색상 분류**

    4색 캔을 좌우 상자에 분류하며 Joint/Task policy와 PTE 미래 offset을 비교한다.

</div>

!!! info "구현 범위"

    Collision avoidance는 현재 거리와 접근 속도에 반응하는 제어 계층이며 경로 계획기는
    아니다. Whole-body OFF는 자동 IK의 base·lift 참여만 끄며 키보드 주행은 유지한다.

## 데모

=== "전신 제어"

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 8px;">
      <iframe
        src="https://www.youtube.com/embed/AXAByoi5CxU"
        title="aiworker-mj-lab 전신 제어 데모"
        style="position: absolute; inset: 0; width: 100%; height: 100%; border: 0;"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen>
      </iframe>
    </div>

    [YouTube에서 보기](https://www.youtube.com/watch?v=AXAByoi5CxU) ·
    [전신 IK 구현](guide/whole_body_ik.md)

=== "팔 전용 제어"

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 8px;">
      <iframe
        src="https://www.youtube.com/embed/2LV_RsAGdz8?list=PLWyQPsEn5Atg&index=2"
        title="aiworker-mj-lab 팔 전용 제어 데모"
        style="position: absolute; inset: 0; width: 100%; height: 100%; border: 0;"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowfullscreen>
      </iframe>
    </div>

    [YouTube에서 보기](https://www.youtube.com/watch?v=2LV_RsAGdz8&list=PLWyQPsEn5Atg&index=2) ·
    [Whole-body OFF 동작](guide/whole_body_ik.md#whole-body-modes)
