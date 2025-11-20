#!/usr/bin/env python3
"""
Otter USV Maneuverability Test Suite
측정 항목:
1. Turning Circle Test (Tactical Diameter, Advance, Transfer)
2. Stopping Distance Test
3. Acceleration Test
4. Maximum Turning Rate Test
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import sys
import os

# PVS 경로 추가
sys.path.append('/home/hyo/PythonVehicleSimulator/src')

# ir-sim 경로 추가
sys.path.append('/home/hyo/ir-sim')
import irsim


class OtterManeuverabilityTest:
    def __init__(self, world_file='otter_maneuver_world.yaml'):
        """초기화"""
        self.world_file = world_file
        self.results = {}
        
    def turning_circle_test(self, velocity=2.0, duration=60.0):
        """
        Turning Circle Test - 360도 선회 테스트
        
        측정값:
        - Tactical Diameter: 180도 선회 후 좌우 거리
        - Advance: 90도 선회 시 전진 거리
        - Transfer: 90도 선회 시 횡이동 거리
        - Steady Turning Radius: 정상 선회 반경
        
        Args:
            velocity: 선회 속도 (m/s)
            duration: 최대 테스트 시간 (s)
        """
        print("\n" + "="*60)
        print(f"🔄 TURNING CIRCLE TEST @ {velocity} m/s")
        print("="*60)
        
        env = irsim.make(self.world_file, display=False, disable_all_plot=True)
        robot = env.robot  # Use .robot property (singular) to get the first robot
        
        # 초기 가속 단계
        print("  Phase 1: 가속 중...")
        for _ in range(50):  # 5초간 가속
            action = np.array([[velocity], [0.0]])
            env.step(action_id=0, action=action)
            
        # 데이터 기록 시작
        trajectory = []
        headings = []
        velocities = []
        yaw_rates = []
        start_x, start_y = robot.state[0, 0], robot.state[1, 0]
        start_heading = robot.state[2, 0]
        
        print("  Phase 2: 선회 시작...")
        
        # 최대 차동 추력으로 선회 (좌회전)
        max_yaw_rate = 0.5  # rad/s (약 28.6 deg/s) - 테스트로 결정
        
        step_count = 0
        max_steps = int(duration / env._world.step_time)
        heading_change = 0
        
        while step_count < max_steps:
            # 선회 명령 (일정 속도, 최대 각속도)
            action = np.array([[velocity], [max_yaw_rate]])
            env.step(action_id=0, action=action)
            
            # 데이터 기록
            x, y = robot.state[0, 0], robot.state[1, 0]
            heading = np.degrees(robot.state[2, 0])  # Convert to degrees
            vx, vy = robot.state[3, 0], robot.state[4, 0]
            r = robot.state[5, 0]  # yaw rate
            
            trajectory.append([x, y])
            headings.append(heading)
            velocities.append(np.sqrt(vx**2 + vy**2))
            yaw_rates.append(r)
            
            # 360도 선회 완료 체크
            start_heading_deg = np.degrees(start_heading)
            heading_change = abs(heading - start_heading_deg)
            if heading_change > 360:
                print(f"  ✅ 360도 선회 완료! (steps: {step_count})")
                break
                
            step_count += 1
            
            # 진행상황 출력
            if step_count % 50 == 0:
                print(f"     진행: {heading_change:.1f}°, 평균 각속도: {np.mean(yaw_rates[-50:]):.3f} rad/s")
        
        env.end()
        
        # 결과 분석
        trajectory = np.array(trajectory)
        headings = np.array(headings)
        start_heading_deg = np.degrees(start_heading)
        
        # 90도, 180도 시점 찾기
        idx_90 = np.argmin(np.abs(headings - (start_heading_deg + 90)))
        idx_180 = np.argmin(np.abs(headings - (start_heading_deg + 180)))
        
        # Advance & Transfer (90도 기준)
        advance = trajectory[idx_90, 0] - start_x  # North 방향 전진거리
        transfer = abs(trajectory[idx_90, 1] - start_y)  # East 방향 횡이동
        
        # Tactical Diameter (180도 기준)
        tactical_diameter = 2 * abs(trajectory[idx_180, 1] - start_y)
        
        # Steady Turning Radius (후반부 평균)
        late_traj = trajectory[int(len(trajectory)*0.5):]
        center_x = np.mean(late_traj[:, 0])
        center_y = np.mean(late_traj[:, 1])
        
        radii = np.sqrt((late_traj[:, 0] - center_x)**2 + 
                       (late_traj[:, 1] - center_y)**2)
        steady_radius = np.mean(radii)
        
        # 실제 평균 각속도
        avg_yaw_rate = np.mean(np.abs(yaw_rates[50:]))  # rad/s
        
        results = {
            'velocity': velocity,
            'tactical_diameter': tactical_diameter,
            'advance': advance,
            'transfer': transfer,
            'steady_radius': steady_radius,
            'avg_yaw_rate_rad': avg_yaw_rate,
            'avg_yaw_rate_deg': np.degrees(avg_yaw_rate),
            'trajectory': trajectory,
            'headings': headings,
            'velocities': velocities,
            'yaw_rates': yaw_rates,
            'center': [center_x, center_y]
        }
        
        # 결과 출력
        print("\n" + "-"*60)
        print("📊 TURNING CIRCLE TEST 결과:")
        print("-"*60)
        print(f"  속도: {velocity:.2f} m/s")
        print(f"  Tactical Diameter: {tactical_diameter:.2f} m")
        print(f"  Advance (90°): {advance:.2f} m")
        print(f"  Transfer (90°): {transfer:.2f} m")
        print(f"  Steady Turning Radius: {steady_radius:.2f} m")
        print(f"  평균 각속도: {avg_yaw_rate:.3f} rad/s ({np.degrees(avg_yaw_rate):.1f}°/s)")
        print(f"  L/R ratio: {2.0/steady_radius:.2f}")
        print("-"*60 + "\n")
        
        self.results['turning_circle'] = results
        return results
    
    
    def stopping_distance_test(self, initial_velocity=3.0, duration=30.0):
        """
        Stopping Distance Test - 정지 거리 측정
        
        Args:
            initial_velocity: 초기 속도 (m/s)
            duration: 최대 테스트 시간 (s)
        """
        print("\n" + "="*60)
        print(f"🛑 STOPPING DISTANCE TEST @ {initial_velocity} m/s")
        print("="*60)
        
        env = irsim.make(self.world_file, display=False, disable_all_plot=True)
        robot = env.robot  # Use .robot property (singular) to get the first robot
        
        # Phase 1: 가속
        print("  Phase 1: 최대속도까지 가속 중...")
        for _ in range(100):
            action = np.array([[initial_velocity], [0.0]])
            env.step(action_id=0, action=action)
        
        # 정지 시작 위치 기록
        start_pos = np.array([robot.state[0, 0], robot.state[1, 0]])
        actual_velocity = np.sqrt(robot.state[3, 0]**2 + robot.state[4, 0]**2)
        
        print(f"  Phase 2: 역추진 정지 시작... (실제 속도: {actual_velocity:.2f} m/s)")
        
        # Phase 2: 역추진 정지
        trajectory = []
        velocities = []
        
        step_count = 0
        max_steps = int(duration / env._world.step_time)
        
        while step_count < max_steps:
            # 역추진 명령
            action = np.array([[-initial_velocity], [0.0]])
            env.step(action_id=0, action=action)
            
            x, y = robot.state[0, 0], robot.state[1, 0]
            vx, vy = robot.state[3, 0], robot.state[4, 0]
            speed = np.sqrt(vx**2 + vy**2)
            
            trajectory.append([x, y])
            velocities.append(speed)
            
            # 정지 판정 (0.05 m/s 이하)
            if speed < 0.05:
                print(f"  ✅ 정지 완료! (steps: {step_count}, time: {step_count*0.1:.1f}s)")
                break
            
            step_count += 1
            
            if step_count % 20 == 0:
                print(f"     현재 속도: {speed:.3f} m/s")
        
        env.end()
        
        # 결과 분석
        trajectory = np.array(trajectory)
        final_pos = trajectory[-1]
        
        stopping_distance = np.linalg.norm(final_pos - start_pos)
        stopping_time = step_count * 0.1
        
        results = {
            'initial_velocity': actual_velocity,
            'stopping_distance': stopping_distance,
            'stopping_time': stopping_time,
            'trajectory': trajectory,
            'velocities': velocities
        }
        
        print("\n" + "-"*60)
        print("📊 STOPPING DISTANCE TEST 결과:")
        print("-"*60)
        print(f"  초기 속도: {actual_velocity:.2f} m/s")
        print(f"  정지 거리: {stopping_distance:.2f} m")
        print(f"  정지 시간: {stopping_time:.2f} s")
        print(f"  평균 감속도: {actual_velocity/stopping_time:.3f} m/s²")
        print("-"*60 + "\n")
        
        self.results['stopping'] = results
        return results
    
    
    def acceleration_test(self, target_velocity=3.0, duration=20.0):
        """
        Acceleration Test - 가속 성능 측정
        
        Args:
            target_velocity: 목표 속도 (m/s)
            duration: 최대 테스트 시간 (s)
        """
        print("\n" + "="*60)
        print(f"🚀 ACCELERATION TEST to {target_velocity} m/s")
        print("="*60)
        
        env = irsim.make(self.world_file, display=False, disable_all_plot=True)
        robot = env.robot  # Use .robot property (singular) to get the first robot
        
        print("  Phase 1: 정지 상태에서 가속 시작...")
        
        trajectory = []
        velocities = []
        times = []
        
        step_count = 0
        max_steps = int(duration / env._world.step_time)
        
        while step_count < max_steps:
            action = np.array([[target_velocity], [0.0]])
            env.step(action_id=0, action=action)
            
            vx, vy = robot.state[3, 0], robot.state[4, 0]
            speed = np.sqrt(vx**2 + vy**2)
            
            trajectory.append([robot.state[0, 0], robot.state[1, 0]])
            velocities.append(speed)
            times.append(step_count * 0.1)
            
            # 목표 속도의 95% 도달 시 완료
            if speed >= 0.95 * target_velocity:
                print(f"  ✅ 목표 속도 도달! (time: {step_count*0.1:.1f}s)")
                break
            
            step_count += 1
            
            if step_count % 20 == 0:
                print(f"     현재 속도: {speed:.3f} m/s")
        
        env.end()
        
        # 결과 분석
        velocities = np.array(velocities)
        times = np.array(times)
        
        # 95% 도달 시간
        idx_95 = np.argmax(velocities >= 0.95 * target_velocity)
        accel_time = times[idx_95] if idx_95 > 0 else times[-1]
        
        # 평균 가속도
        avg_acceleration = velocities[idx_95] / accel_time if accel_time > 0 else 0
        
        # 거리
        trajectory = np.array(trajectory)
        accel_distance = np.linalg.norm(trajectory[-1] - trajectory[0])
        
        results = {
            'target_velocity': target_velocity,
            'final_velocity': velocities[-1],
            'accel_time': accel_time,
            'accel_distance': accel_distance,
            'avg_acceleration': avg_acceleration,
            'velocities': velocities,
            'times': times,
            'trajectory': trajectory
        }
        
        print("\n" + "-"*60)
        print("📊 ACCELERATION TEST 결과:")
        print("-"*60)
        print(f"  목표 속도: {target_velocity:.2f} m/s")
        print(f"  도달 시간 (95%): {accel_time:.2f} s")
        print(f"  가속 거리: {accel_distance:.2f} m")
        print(f"  평균 가속도: {avg_acceleration:.3f} m/s²")
        print("-"*60 + "\n")
        
        self.results['acceleration'] = results
        return results
    
    
    def plot_results(self, save_path='otter_maneuverability_results.png'):
        """결과 시각화"""
        
        fig = plt.figure(figsize=(16, 12))
        
        # 1. Turning Circle Plot
        if 'turning_circle' in self.results:
            ax1 = plt.subplot(2, 3, 1)
            data = self.results['turning_circle']
            traj = data['trajectory']
            
            ax1.plot(traj[:, 1], traj[:, 0], 'b-', linewidth=2, label='Trajectory')
            ax1.plot(traj[0, 1], traj[0, 0], 'go', markersize=10, label='Start')
            ax1.plot(traj[-1, 1], traj[-1, 0], 'ro', markersize=10, label='End')
            
            # 중심점과 선회 원
            center = data['center']
            circle = Circle((center[1], center[0]), data['steady_radius'], 
                          fill=False, color='r', linestyle='--', label='Steady Circle')
            ax1.add_patch(circle)
            ax1.plot(center[1], center[0], 'r+', markersize=15, markeredgewidth=2)
            
            ax1.set_xlabel('East (m)', fontsize=12)
            ax1.set_ylabel('North (m)', fontsize=12)
            ax1.set_title(f'Turning Circle Test @ {data["velocity"]:.1f} m/s', 
                         fontsize=14, fontweight='bold')
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')
            ax1.legend()
            
            # Tactical diameter 표시
            ax1.text(0.02, 0.98, 
                    f"Tactical Diameter: {data['tactical_diameter']:.2f} m\n"
                    f"Advance (90°): {data['advance']:.2f} m\n"
                    f"Transfer (90°): {data['transfer']:.2f} m\n"
                    f"Steady Radius: {data['steady_radius']:.2f} m",
                    transform=ax1.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round', 
                    facecolor='wheat', alpha=0.8))
            
            # 2. Yaw Rate Plot
            ax2 = plt.subplot(2, 3, 2)
            yaw_rates_deg = np.degrees(data['yaw_rates'])
            ax2.plot(yaw_rates_deg, 'b-', linewidth=1.5)
            ax2.axhline(y=data['avg_yaw_rate_deg'], color='r', 
                       linestyle='--', label=f'Avg: {data["avg_yaw_rate_deg"]:.1f}°/s')
            ax2.set_xlabel('Step', fontsize=12)
            ax2.set_ylabel('Yaw Rate (°/s)', fontsize=12)
            ax2.set_title('Yaw Rate during Turn', fontsize=14, fontweight='bold')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            
            # 3. Velocity during Turn
            ax3 = plt.subplot(2, 3, 3)
            ax3.plot(data['velocities'], 'g-', linewidth=1.5)
            ax3.axhline(y=data['velocity'], color='r', 
                       linestyle='--', label=f'Target: {data["velocity"]:.2f} m/s')
            ax3.set_xlabel('Step', fontsize=12)
            ax3.set_ylabel('Speed (m/s)', fontsize=12)
            ax3.set_title('Speed during Turn', fontsize=14, fontweight='bold')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        # 4. Stopping Distance Plot
        if 'stopping' in self.results:
            ax4 = plt.subplot(2, 3, 4)
            data = self.results['stopping']
            traj = data['trajectory']
            
            ax4.plot(traj[:, 1], traj[:, 0], 'r-', linewidth=2)
            ax4.plot(traj[0, 1], traj[0, 0], 'go', markersize=10, label='Start')
            ax4.plot(traj[-1, 1], traj[-1, 0], 'ro', markersize=10, label='Stop')
            
            ax4.set_xlabel('East (m)', fontsize=12)
            ax4.set_ylabel('North (m)', fontsize=12)
            ax4.set_title(f'Stopping Distance Test @ {data["initial_velocity"]:.1f} m/s', 
                         fontsize=14, fontweight='bold')
            ax4.grid(True, alpha=0.3)
            ax4.axis('equal')
            ax4.legend()
            
            ax4.text(0.02, 0.98,
                    f"Stopping Distance: {data['stopping_distance']:.2f} m\n"
                    f"Stopping Time: {data['stopping_time']:.2f} s\n"
                    f"Avg Deceleration: {data['initial_velocity']/data['stopping_time']:.3f} m/s²",
                    transform=ax4.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round',
                    facecolor='wheat', alpha=0.8))
            
            # 5. Velocity during Stopping
            ax5 = plt.subplot(2, 3, 5)
            ax5.plot(data['velocities'], 'r-', linewidth=1.5)
            ax5.set_xlabel('Step', fontsize=12)
            ax5.set_ylabel('Speed (m/s)', fontsize=12)
            ax5.set_title('Speed during Stopping', fontsize=14, fontweight='bold')
            ax5.grid(True, alpha=0.3)
        
        # 6. Acceleration Plot
        if 'acceleration' in self.results:
            ax6 = plt.subplot(2, 3, 6)
            data = self.results['acceleration']
            
            ax6.plot(data['times'], data['velocities'], 'g-', linewidth=2)
            ax6.axhline(y=data['target_velocity'], color='r', 
                       linestyle='--', label=f'Target: {data["target_velocity"]:.2f} m/s')
            ax6.axhline(y=0.95*data['target_velocity'], color='orange',
                       linestyle=':', label='95% Target')
            
            ax6.set_xlabel('Time (s)', fontsize=12)
            ax6.set_ylabel('Speed (m/s)', fontsize=12)
            ax6.set_title('Acceleration Test', fontsize=14, fontweight='bold')
            ax6.grid(True, alpha=0.3)
            ax6.legend()
            
            ax6.text(0.02, 0.98,
                    f"Accel Time (95%): {data['accel_time']:.2f} s\n"
                    f"Accel Distance: {data['accel_distance']:.2f} m\n"
                    f"Avg Acceleration: {data['avg_acceleration']:.3f} m/s²",
                    transform=ax6.transAxes, fontsize=10,
                    verticalalignment='top', bbox=dict(boxstyle='round',
                    facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n📊 결과 그래프 저장: {save_path}")
        plt.close()
        
        
    def run_all_tests(self):
        """전체 테스트 실행"""
        print("\n" + "="*60)
        print("🚢 OTTER USV MANEUVERABILITY TEST SUITE")
        print("="*60)
        
        # Test 1: Turning Circle @ 2.0 m/s
        self.turning_circle_test(velocity=2.0, duration=60.0)
        
        # Test 2: Stopping Distance @ 3.0 m/s
        self.stopping_distance_test(initial_velocity=3.0, duration=30.0)
        
        # Test 3: Acceleration Test
        self.acceleration_test(target_velocity=3.0, duration=20.0)
        
        # 결과 시각화
        self.plot_results()
        
        # 요약 리포트
        self.print_summary_report()
        
        
    def print_summary_report(self):
        """요약 리포트 출력"""
        print("\n" + "="*60)
        print("📋 OTTER USV MANEUVERABILITY SUMMARY REPORT")
        print("="*60)
        
        if 'turning_circle' in self.results:
            data = self.results['turning_circle']
            print(f"\n🔄 Turning Performance @ {data['velocity']:.2f} m/s:")
            print(f"   • Tactical Diameter: {data['tactical_diameter']:.2f} m ({data['tactical_diameter']/2.0:.2f} × L)")
            print(f"   • Steady Turning Radius: {data['steady_radius']:.2f} m ({data['steady_radius']/2.0:.2f} × L)")
            print(f"   • Advance (90°): {data['advance']:.2f} m")
            print(f"   • Transfer (90°): {data['transfer']:.2f} m")
            print(f"   • Maximum Yaw Rate: {data['avg_yaw_rate_deg']:.1f}°/s ({data['avg_yaw_rate_rad']:.3f} rad/s)")
        
        if 'stopping' in self.results:
            data = self.results['stopping']
            print(f"\n🛑 Stopping Performance @ {data['initial_velocity']:.2f} m/s:")
            print(f"   • Stopping Distance: {data['stopping_distance']:.2f} m ({data['stopping_distance']/2.0:.2f} × L)")
            print(f"   • Stopping Time: {data['stopping_time']:.2f} s")
            print(f"   • Average Deceleration: {data['initial_velocity']/data['stopping_time']:.3f} m/s²")
        
        if 'acceleration' in self.results:
            data = self.results['acceleration']
            print(f"\n🚀 Acceleration Performance to {data['target_velocity']:.2f} m/s:")
            print(f"   • Acceleration Time (95%): {data['accel_time']:.2f} s")
            print(f"   • Acceleration Distance: {data['accel_distance']:.2f} m ({data['accel_distance']/2.0:.2f} × L)")
            print(f"   • Average Acceleration: {data['avg_acceleration']:.3f} m/s²")
        
        print("\n" + "="*60)
        print("✅ 모든 테스트 완료!")
        print("="*60 + "\n")


if __name__ == '__main__':
    # 테스트 실행
    tester = OtterManeuverabilityTest()
    tester.run_all_tests()
