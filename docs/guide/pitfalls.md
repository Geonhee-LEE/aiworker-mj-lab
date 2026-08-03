# 체크리스트

MuJoCo 코드 수정 전후에 확인할 항목과 이 프로젝트에서 실제로 발생했던 실패
사례를 한 페이지에 모았다.

| 항목 | 확인 |
|---|---|
| `mj_name2id` 결과 | `-1` 여부 확인 후 사용 |
| actuator lookup | `aid is None` 여부 확인 후 `data.ctrl[aid]` 사용 |
| numpy index | `arr[None] = value`가 전체 배열 broadcast가 되지 않는지 확인 |
| qpos 직접 쓰기 | reset/초기 배치 외 live robot qpos 수정 금지 |
| IK 상태 분리 | live `data.qpos`를 넘겨도 solver가 복사한 배열과 자체 트리만 수정하는지 확인 |
| site/body 기준 | IK target은 body origin이 아니라 `site` 기준인지 확인 |
| quaternion frame | local/world frame 변환 순서 확인 |
| contact force | 위치 조건이 아니라 `mj_contactForce()` 기반 판정인지 확인 |
| actuator range | `ctrlrange`, `forcerange`, joint `range` 확인 |
| wheel command | 조향 정렬 전 wheel velocity가 0으로 gated 되는지 확인 |
| marker state | UI target, mocap marker, IK world target 동기화 확인 |
| whole-body toggle | ON/OFF 전후 hand/virtual world pose와 cached base command 확인 |
| arm-only gate | base/lift weight가 아니라 lower/upper velocity bound를 0으로 고정 |
| base-only gate | `participation_scale: 0.0`은 base 3축 bound만 0, lift·팔 참여는 유지 |
| base 참여율 | 목표와 속도 상한을 같은 `participation_scale`로 축소하는지 확인 |
| manual handover | key release 동안 zero 유지, 정지 뒤 target/reference rebase 확인 |
| collision 범위 | finger-object와 wheel-floor 의도 접촉을 CBF pair에서 제외 |
| 문서 | 함수 역할이나 target 의미가 바뀌면 `docs/guide/` 업데이트 |

## 실제 실패 사례와 교훈

### 미러 관절은 숫자가 아니라 물리적 방향을 확인한다

왼손 엄지 curl range는 오른손과 부호뿐 아니라 “어느 끝이 편 상태인가”도 반대였다.
항상 `lo`를 편 상태로 가정하자 `thumb=0`에서도 엄지가 손바닥을 약 40 N으로 눌렀다.
좌우 관절을 추가할 때는 range 숫자의 대칭성뿐 아니라 open/close 자세를 실제 FK와
contact로 검증한다.

### `data.ctrl[None]`은 오류 없이 전체 배열을 바꿀 수 있다

actuator lookup 실패값 `None`을 NumPy index로 쓰면 `np.newaxis`로 해석되어 전체
control 배열에 값이 broadcast될 수 있다. 모델마다 actuator 구성이 다르므로
`find_actuator_for_joint()`의 반환값을 항상 `is None`으로 확인한 뒤 기록한다.

### contact `priority`는 관련 파라미터 묶음 전체에 영향을 준다

캔에 높은 priority를 주자 finger의 `solref`와 `solimp`까지 캔 값으로 대체되어
관통이 크게 늘었다. priority가 높은 geom에는 friction·condim뿐 아니라 필요한
접촉 solver 파라미터를 모두 명시하고 실제 접촉력과 침투 깊이로 검증한다.

### 움직일 자유도가 없는 방향의 접촉력은 직접 측정한다

수직 자유도가 없는 base가 바닥과 고정 깊이로 겹치면서 기본 contact 반발력이 로봇
무게의 약 28배가 되었고 조향을 막았다. 전용 `<pair>`의 접촉 강성을 조정해 해결했다.
고정 body의 의도된 초기 침투는 기본값을 믿지 말고 힘과 응답 시간을 측정한다.

### 자동 생성 형상과 설정은 결과로 확인한다

원통 primitive의 자동 UV는 세부 라벨을 올바르게 펴지 못해 명시적 UV가 있는 OBJ가
필요했다. 마찬가지로 auto-inertia, auto-limits 같은 엔진 자동 기능은 렌더링과 모델
배열을 검사하기 전까지 보장으로 취급하지 않는다.

### `range="0 0"`만으로 관절이 잠기지는 않는다

약지·새끼 관절에 `range="0 0"`을 적었지만 `limited="true"`가 없어 장시간 중력에
밀렸다. 잠금 불변식은 XML 문자열이 아니라 컴파일된 `model.jnt_limited`와 장시간
물리 테스트로 확인한다.
