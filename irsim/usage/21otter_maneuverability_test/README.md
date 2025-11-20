# Otter USV Maneuverability Test

PVS와 ir-sim을 이용한 Otter USV 기동 성능 실측 테스트

## 📋 측정 항목

1. **Turning Circle Test** (선회권 테스트)
   - Tactical Diameter: 180도 선회 시 좌우 거리
   - Advance: 90도 선회 시 전진 거리
   - Transfer: 90도 선회 시 횡이동 거리
   - Steady Turning Radius: 정상 선회 반경
   - Maximum Yaw Rate: 실제 최대 선회 각속도

2. **Stopping Distance Test** (정지 거리 테스트)
   - 최대 속도에서 역추진으로 정지
   - Stopping Distance: 정지 거리
   - Stopping Time: 정지 시간
   - Average Deceleration: 평균 감속도

3. **Acceleration Test** (가속 성능 테스트)
   - 0 → 최대속도 가속
   - Acceleration Time: 목표 속도 95% 도달 시간
   - Acceleration Distance: 가속 거리
   - Average Acceleration: 평균 가속도

## 🚀 실행 방법

```bash
# 1. Conda 환경 활성화
conda activate DRL-otter-nav

# 2. 테스트 디렉토리로 이동
cd /home/hyo/ir-sim/irsim/usage/21otter_maneuverability_test

# 3. 테스트 실행
python3 otter_maneuverability_test.py
```

## 📊 출력 결과

- **콘솔**: 실시간 진행상황 및 측정 결과 요약
- **그래프**: `otter_maneuverability_results.png` (6개 subplot)
  1. Turning Circle Trajectory (궤적)
  2. Yaw Rate during Turn (선회 중 각속도)
  3. Speed during Turn (선회 중 속도)
  4. Stopping Distance Trajectory (정지 궤적)
  5. Speed during Stopping (정지 중 속도 감소)
  6. Acceleration Test (가속 곡선)

## 📁 파일 설명

- `otter_maneuver_world.yaml`: 빈 세계에 Otter USV 배치
- `otter_maneuverability_test.py`: 기동 성능 측정 메인 스크립트
- `README.md`: 이 파일

## 🔧 파라미터 조정

```python
# otter_maneuverability_test.py 마지막 부분 수정

tester = OtterManeuverabilityTest()

# 선회 테스트: 속도 변경 가능
tester.turning_circle_test(velocity=2.5, duration=60.0)

# 정지 테스트: 초기 속도 변경 가능
tester.stopping_distance_test(initial_velocity=2.5, duration=30.0)

# 가속 테스트: 목표 속도 변경 가능
tester.acceleration_test(target_velocity=2.5, duration=20.0)
```

## 📚 참고

- PVS (Python Vehicle Simulator): `/home/hyo/PythonVehicleSimulator`
- ir-sim: `/home/hyo/ir-sim`
- Fossen (2021), Handbook of Marine Craft Hydrodynamics and Motion Control
