#!/usr/bin/env python3
"""
6축 로봇 통합 제어 GUI
- 각도/좌표 + 속도/가속도 제어
- ROS2 통신을 통한 ESP32 마스터 컨트롤러와 연동
- IK(역기구학) 계산 및 웨이포인트 기반 경로 계획
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import math
import threading
import time
import numpy as np
from scipy.optimize import least_squares
import json
import os
from datetime import datetime
import matplotlib
matplotlib.use('TkAgg')  # Tkinter와 호환되는 백엔드 사용
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class CobotKinematics:
    """6축 협동로봇 운동학 클래스"""
    def __init__(self):
        # DH 파라미터
        self.a = [0, 0.2805, 0.2495, 0, 0, 0]  # 링크 길이 (m)
        self.alpha = [-np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]  # 링크 비틀림 (rad)
        self.d = [0.235, 0, 0, 0.258, 0.180, 0.123]  # 조인트 오프셋 (m)
        self.n_joints = 6
        
        # 조인트 오프셋 정의
        self.joint_offsets = [-np.pi/2, -np.pi/2, 0, np.pi/2, np.pi/2, 0]
        
        # 조인트 회전 방향
        self.joint_direction = [1, 1, -1, 1, 1, 1]
        
        # 조인트 동작 범위
        self.joint_limits = np.array([[-135, 135]] * self.n_joints)
        
    def _apply_joint_offset(self, user_angles):
        """사용자 입력 각도에 오프셋 적용"""
        adjusted_angles = np.array(user_angles) * np.array(self.joint_direction)
        return adjusted_angles + np.array(self.joint_offsets)
    
    def _remove_joint_offset(self, dh_angles):
        """DH 각도에서 오프셋 제거"""
        adjusted_angles = np.array(dh_angles) - np.array(self.joint_offsets)
        return adjusted_angles * np.array(self.joint_direction)
    
    def dh_transform(self, a, alpha, d, theta):
        """DH 변환 행렬"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T
    
    def forward_kinematics(self, user_joint_angles):
        """순방향 운동학"""
        dh_angles = self._apply_joint_offset(user_joint_angles)
        T = np.eye(4)
        
        # 각 조인트 위치를 저장할 리스트
        joint_positions = [np.array([0, 0, 0])]  # 베이스 위치
        
        for i in range(self.n_joints):
            T_i = self.dh_transform(self.a[i], self.alpha[i], self.d[i], dh_angles[i])
            T = T @ T_i
            
            # 현재 조인트 위치 저장
            joint_positions.append(T[:3, 3].copy())
        
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        euler_angles = self.rotation_matrix_to_euler(rotation_matrix)
        
        return position, euler_angles, T, joint_positions
    
    def rotation_matrix_to_euler(self, R):
        """회전 행렬을 오일러 각도로 변환"""
        sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
        
        return np.array([x, y, z])
    
    def euler_to_rotation_matrix(self, euler_angles):
        """오일러 각도를 회전 행렬로 변환"""
        rx, ry, rz = euler_angles
        
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        return Rz @ Ry @ Rx
    
    def draw_coordinate_frame(self, ax, origin, rotation_matrix, scale=0.1, label="", alpha=0.8):
        """좌표계를 화살표로 그리기"""
        # X축: 빨강, Y축: 초록, Z축: 파랑
        colors = ['r', 'g', 'b']
        labels = ['X', 'Y', 'Z']
        
        for i in range(3):
            # 회전 행렬의 각 열이 해당 축의 방향
            direction = rotation_matrix[:, i] * scale
            ax.quiver(origin[0], origin[1], origin[2],
                     direction[0], direction[1], direction[2],
                     color=colors[i], arrow_length_ratio=0.3, linewidth=2,
                     alpha=alpha)
            
            # 축 레이블 표시
            end_point = origin + direction * 1.2
            ax.text(end_point[0], end_point[1], end_point[2], 
                   f"{label}{labels[i]}", color=colors[i], fontsize=8, fontweight='bold')
    
    def plot_robot(self, user_joint_angles, ax, show_frames=True, alpha=0.6):
        """로봇 구조 시각화"""
        # 조인트 위치 계산
        _, _, T, joint_positions = self.forward_kinematics(user_joint_angles)
        joint_positions = np.array(joint_positions)
        
        # 로봇 링크 그리기 (파란색)
        ax.plot(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], 
               'b-', linewidth=4, alpha=alpha, label='로봇 링크')
        
        # 조인트 표시 (파란 원)
        ax.scatter(joint_positions[:-1, 0], joint_positions[:-1, 1], joint_positions[:-1, 2],
                  c='blue', marker='o', s=80, alpha=alpha, edgecolors='darkblue', linewidths=1.5)
        
        # 엔드 이펙터 표시 (빨간 다이아몬드)
        ax.scatter(joint_positions[-1, 0], joint_positions[-1, 1], joint_positions[-1, 2], 
                  color='red', s=200, marker='D', alpha=1.0, edgecolors='darkred', linewidths=2,
                  label='엔드 이펙터')
        
        if show_frames:
            # 베이스 좌표계 표시
            base_origin = np.array([0, 0, 0])
            base_rotation = np.eye(3)
            self.draw_coordinate_frame(ax, base_origin, base_rotation, scale=0.1, label="Base_", alpha=alpha)
            
            # 엔드 이펙터 좌표계 표시
            ee_origin = T[:3, 3]
            ee_rotation = T[:3, :3]
            self.draw_coordinate_frame(ax, ee_origin, ee_rotation, scale=0.08, label="EE_", alpha=alpha)
    
    def inverse_kinematics(self, target_position, target_orientation, initial_guess=None):
        """역방향 운동학"""
        if initial_guess is None:
            initial_guess = np.zeros(self.n_joints)
        
        # 초기값을 조인트 범위 내로 클리핑
        initial_guess_deg = np.degrees(initial_guess)
        initial_guess_deg = np.clip(initial_guess_deg, 
                                   self.joint_limits[:, 0], 
                                   self.joint_limits[:, 1])
        initial_guess = np.radians(initial_guess_deg)
        
        def objective_function(user_joint_angles):
            pos, euler, _, _ = self.forward_kinematics(user_joint_angles)
            
            # 위치 오차
            pos_error = target_position - pos
            
            # 방향 오차
            orient_error = target_orientation - euler
            
            # 전체 오차 (위치 오차에 더 큰 가중치)
            error = np.concatenate([pos_error * 1000, orient_error])
            return error
        
        # 조인트 범위를 라디안으로 변환
        lower_bounds = np.radians(self.joint_limits[:, 0])
        upper_bounds = np.radians(self.joint_limits[:, 1])
        
        try:
            result = least_squares(objective_function, initial_guess, 
                                 bounds=(lower_bounds, upper_bounds),
                                 method='trf',
                                 ftol=1e-6, xtol=1e-6, max_nfev=1000)
            
            solution = result.x
            
            # 해의 유효성 검증
            pos, euler, _, _ = self.forward_kinematics(solution)
            pos_error = np.linalg.norm(target_position - pos)
            orient_error = np.linalg.norm(target_orientation - euler)
            
            if pos_error < 0.001 and orient_error < 0.01:
                return solution, True
            else:
                return solution, False
        except:
            return initial_guess, False


class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        
        # 🤖 운동학 객체 생성
        self.robot = CobotKinematics()
        
        # 📊 현재 각도 (초기값)
        self.current_angles = np.zeros(6)
        
        # 📍 웨이포인트 경로 데이터
        self.waypoints = []  # [(x,y,z,rx,ry,rz), ...]
        self.interpolated_points = []
        self.angle_trajectory = []
        
        # ROS2 퍼블리셔 생성
        self.angle_pub = self.create_publisher(Float32MultiArray, 'servo_angles', 10)
        self.coord_pub = self.create_publisher(Float32MultiArray, 'robot_coords', 10)
        self.angle_speed_pub = self.create_publisher(Float32MultiArray, 'servo_angles_with_speed', 10)
        self.coord_speed_pub = self.create_publisher(Float32MultiArray, 'robot_coords_with_speed', 10)
        self.sync_settings_pub = self.create_publisher(Float32MultiArray, 'sync_settings', 10)
        
        # 🚀 보간 전용 고속 모드 퍼블리셔 (ACK 없는 빠른 실행)
        self.interpolation_fast_pub = self.create_publisher(Float32MultiArray, 'interpolation_fast_mode', 10)
        
        # 🎯 새로운 경로 명령 퍼블리셔 (두 점만 전송)
        self.path_command_pub = self.create_publisher(Float32MultiArray, 'path_command', 10)
        
        # ROS2 구독자 생성 (서보 상태 피드백)
        self.servo_status_sub = self.create_subscription(
            Float32MultiArray,
            'servo_status',
            self.servo_status_callback,
            10
        )
        
        # 동기화 완료 결과는 servo_status 토픽을 통해 수신
        
        # GUI 초기화
        self.setup_gui()
        
        # ROS2 타이머 생성
        self.timer = self.create_timer(0.1, self.ros_spin)
        
        # 연결 상태 모니터링 타이머
        self.connection_timer = self.create_timer(1.0, self.check_connection_status)
        
        self.get_logger().info('🤖 로봇 통합 제어 GUI가 시작되었습니다.')
    
    def setup_gui(self):
        """GUI 설정"""
        self.root = tk.Tk()
        self.root.title("6축 로봇 제어 시스템")
        self.root.geometry("1200x800")
        
        # 스타일 설정
        style = ttk.Style()
        style.theme_use('clam')
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 탭 노트북 생성
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 각도 제어 탭
        self.setup_angle_control_tab()
        
        # 좌표 제어 탭
        self.setup_coordinate_control_tab()
        
        # 동기화 설정 탭
        self.setup_sync_settings_tab()
        
        # 경로 제어 탭 (두 점 보간)
        self.setup_path_control_tab()
        
        # 🎯 웨이포인트 경로 계획 탭 (다중 경로점 + IK)
        self.setup_waypoint_planning_tab()
        
        # 상태 모니터링 탭
        self.setup_status_monitoring_tab()
        
        # 하단 버튼 프레임
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=1, column=0, columnspan=2, pady=10)
        
        # 긴급 정지 버튼
        self.emergency_stop_btn = ttk.Button(
            button_frame, 
            text="🚨 긴급 정지", 
            command=self.emergency_stop,
            style='Emergency.TButton'
        )
        self.emergency_stop_btn.grid(row=0, column=0, padx=5)
        
        # 홈 포지션 버튼
        self.home_btn = ttk.Button(
            button_frame, 
            text="🏠 홈 포지션", 
            command=self.go_home
        )
        self.home_btn.grid(row=0, column=1, padx=5)
        
        # 연결 상태 표시
        self.connection_label = ttk.Label(button_frame, text="연결 상태: 대기 중...")
        self.connection_label.grid(row=0, column=2, padx=20)
        
        # 연결 상태 변수 초기화
        self.last_message_time = time.time()
        self.connection_status = "대기 중..."
        
        # 경로 실행 상태 변수
        self.path_executing = False
        self.path_thread = None
        
        # 스타일 설정
        style.configure('Emergency.TButton', foreground='red', font=('Arial', 12, 'bold'))
    
    def setup_angle_control_tab(self):
        """각도 제어 탭 설정"""
        angle_frame = ttk.Frame(self.notebook)
        self.notebook.add(angle_frame, text="각도 제어")
        
        # 기본 각도 제어
        basic_frame = ttk.LabelFrame(angle_frame, text="기본 각도 제어", padding="10")
        basic_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 각 서보별 각도 입력
        self.angle_vars = []
        for i in range(6):
            ttk.Label(basic_frame, text=f"서보 {i+1}:").grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)
            var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(var)
            angle_entry = ttk.Entry(basic_frame, textvariable=var, width=10)
            angle_entry.grid(row=i, column=1, padx=5, pady=2)
            ttk.Label(basic_frame, text="도").grid(row=i, column=2, padx=5, pady=2)
        
        # 기본 각도 제어 버튼
        ttk.Button(basic_frame, text="각도로 이동", command=self.send_angles).grid(row=6, column=0, columnspan=3, pady=10)
        
        # 속도/가속도 포함 각도 제어
        speed_frame = ttk.LabelFrame(angle_frame, text="속도/가속도 포함 각도 제어", padding="10")
        speed_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 각 서보별 각도 입력 (속도/가속도 포함)
        self.angle_speed_vars = []
        for i in range(6):
            ttk.Label(speed_frame, text=f"서보 {i+1}:").grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)
            var = tk.DoubleVar(value=0.0)
            self.angle_speed_vars.append(var)
            angle_entry = ttk.Entry(speed_frame, textvariable=var, width=10)
            angle_entry.grid(row=i, column=1, padx=5, pady=2)
            ttk.Label(speed_frame, text="도").grid(row=i, column=2, padx=5, pady=2)
        
        # 속도 설정
        ttk.Label(speed_frame, text="속도:").grid(row=6, column=0, padx=5, pady=2, sticky=tk.W)
        self.speed_var = tk.DoubleVar(value=50.0)
        speed_entry = ttk.Entry(speed_frame, textvariable=self.speed_var, width=10)
        speed_entry.grid(row=6, column=1, padx=5, pady=2)
        ttk.Label(speed_frame, text="deg/s").grid(row=6, column=2, padx=5, pady=2)
        
        # 가속도 설정
        ttk.Label(speed_frame, text="가속도:").grid(row=7, column=0, padx=5, pady=2, sticky=tk.W)
        self.accel_var = tk.DoubleVar(value=30.0)
        accel_entry = ttk.Entry(speed_frame, textvariable=self.accel_var, width=10)
        accel_entry.grid(row=7, column=1, padx=5, pady=2)
        ttk.Label(speed_frame, text="deg/s²").grid(row=7, column=2, padx=5, pady=2)
        
        # 속도/가속도 포함 각도 제어 버튼
        ttk.Button(speed_frame, text="속도/가속도로 이동", command=self.send_angles_with_speed).grid(row=8, column=0, columnspan=3, pady=10)
        
        # 프리셋 버튼들
        preset_frame = ttk.LabelFrame(angle_frame, text="프리셋 동작", padding="10")
        preset_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        presets = [
            ("홈 포지션", [0, 0, 0, 0, 0, 0]),
            ("테스트 1", [90, -45, 0, 45, -90, 0]),
            ("테스트 2", [45, 45, 45, 45, 45, 45]),
            ("테스트 3", [-90, 90, -45, 45, 0, 0])
        ]
        
        for i, (name, angles) in enumerate(presets):
            ttk.Button(
                preset_frame, 
                text=name, 
                command=lambda a=angles: self.set_preset_angles(a)
            ).grid(row=i//2, column=i%2, padx=5, pady=2)
    
    def setup_coordinate_control_tab(self):
        """좌표 제어 탭 설정"""
        coord_frame = ttk.Frame(self.notebook)
        self.notebook.add(coord_frame, text="좌표 제어")
        
        # 기본 좌표 제어
        basic_frame = ttk.LabelFrame(coord_frame, text="기본 좌표 제어", padding="10")
        basic_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 위치 입력
        ttk.Label(basic_frame, text="X (mm):").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.x_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.x_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(basic_frame, text="Y (mm):").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.y_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.y_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(basic_frame, text="Z (mm):").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.z_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.z_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        
        # 자세 입력
        ttk.Label(basic_frame, text="Roll (도):").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.roll_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.roll_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(basic_frame, text="Pitch (도):").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.pitch_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.pitch_var, width=10).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(basic_frame, text="Yaw (도):").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.yaw_var = tk.DoubleVar(value=0.0)
        ttk.Entry(basic_frame, textvariable=self.yaw_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        # 기본 좌표 제어 버튼 (메모리 절약을 위해 마스터에서 비활성화됨)
        send_coord_btn = ttk.Button(basic_frame, text="좌표로 이동 (비활성화됨)", command=self.send_coordinates, state='disabled')
        send_coord_btn.grid(row=6, column=0, columnspan=2, pady=10)
        
        # 안내 메시지
        ttk.Label(basic_frame, text="⚠️ 속도/가속도 포함 좌표 제어를 사용하세요", 
                  foreground="red", font=('Arial', 8)).grid(row=7, column=0, columnspan=2)
        
        # 속도/가속도 포함 좌표 제어
        speed_frame = ttk.LabelFrame(coord_frame, text="속도/가속도 포함 좌표 제어", padding="10")
        speed_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 좌표 입력 (속도/가속도 포함)
        ttk.Label(speed_frame, text="X (mm):").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.x_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.x_speed_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(speed_frame, text="Y (mm):").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.y_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.y_speed_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(speed_frame, text="Z (mm):").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.z_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.z_speed_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(speed_frame, text="Roll (도):").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.roll_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.roll_speed_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(speed_frame, text="Pitch (도):").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.pitch_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.pitch_speed_var, width=10).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(speed_frame, text="Yaw (도):").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.yaw_speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(speed_frame, textvariable=self.yaw_speed_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        # 속도 설정
        ttk.Label(speed_frame, text="속도:").grid(row=6, column=0, padx=5, pady=2, sticky=tk.W)
        self.coord_speed_var = tk.DoubleVar(value=50.0)
        ttk.Entry(speed_frame, textvariable=self.coord_speed_var, width=10).grid(row=6, column=1, padx=5, pady=2)
        ttk.Label(speed_frame, text="deg/s").grid(row=6, column=2, padx=5, pady=2)
        
        # 가속도 설정
        ttk.Label(speed_frame, text="가속도:").grid(row=7, column=0, padx=5, pady=2, sticky=tk.W)
        self.coord_accel_var = tk.DoubleVar(value=30.0)
        ttk.Entry(speed_frame, textvariable=self.coord_accel_var, width=10).grid(row=7, column=1, padx=5, pady=2)
        ttk.Label(speed_frame, text="deg/s²").grid(row=7, column=2, padx=5, pady=2)
        
        # 속도/가속도 포함 좌표 제어 버튼
        ttk.Button(speed_frame, text="속도/가속도로 이동", command=self.send_coordinates_with_speed).grid(row=8, column=0, columnspan=3, pady=10)
        
        # 좌표 프리셋 버튼들
        coord_preset_frame = ttk.LabelFrame(coord_frame, text="좌표 프리셋", padding="10")
        coord_preset_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        coord_presets = [
            ("홈 포지션", [0, 0, 0, 0, 0, 0]),
            ("위치 1", [100, 100, 100, 0, 0, 0]),
            ("위치 2", [200, 0, 150, 0, 0, 90]),
            ("위치 3", [0, 200, 200, 0, 0, 180])
        ]
        
        for i, (name, coords) in enumerate(coord_presets):
            ttk.Button(
                coord_preset_frame, 
                text=name, 
                command=lambda c=coords: self.set_preset_coordinates(c)
            ).grid(row=i//2, column=i%2, padx=5, pady=2)
    
    def setup_sync_settings_tab(self):
        """동기화 설정 탭 설정"""
        sync_frame = ttk.Frame(self.notebook)
        self.notebook.add(sync_frame, text="동기화 설정")
        
        # 동기화 설정
        settings_frame = ttk.LabelFrame(sync_frame, text="동기화 파라미터", padding="10")
        settings_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 속도 설정
        ttk.Label(settings_frame, text="기본 속도:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.sync_speed_var = tk.DoubleVar(value=50.0)
        ttk.Entry(settings_frame, textvariable=self.sync_speed_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(settings_frame, text="deg/s").grid(row=0, column=2, padx=5, pady=2)
        
        # 가속도 설정
        ttk.Label(settings_frame, text="기본 가속도:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.sync_accel_var = tk.DoubleVar(value=30.0)
        ttk.Entry(settings_frame, textvariable=self.sync_accel_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(settings_frame, text="deg/s²").grid(row=1, column=2, padx=5, pady=2)
        
        # 타임아웃 설정
        ttk.Label(settings_frame, text="타임아웃:").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.sync_timeout_var = tk.IntVar(value=10)
        ttk.Entry(settings_frame, textvariable=self.sync_timeout_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(settings_frame, text="초").grid(row=2, column=2, padx=5, pady=2)
        
        # 최소 속도 비율 설정
        ttk.Label(settings_frame, text="최소 속도 비율:").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.sync_min_ratio_var = tk.DoubleVar(value=0.1)
        ttk.Entry(settings_frame, textvariable=self.sync_min_ratio_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        ttk.Label(settings_frame, text="(0.01-1.0)").grid(row=3, column=2, padx=5, pady=2)
        
        # 설정 적용 버튼
        ttk.Button(settings_frame, text="설정 적용", command=self.apply_sync_settings).grid(row=4, column=0, columnspan=3, pady=10)
        
        # 개별 서보 속도 설정
        individual_frame = ttk.LabelFrame(sync_frame, text="개별 서보 속도 설정", padding="10")
        individual_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 각 서보별 속도/가속도 설정
        self.individual_speed_vars = []
        self.individual_accel_vars = []
        
        for i in range(6):
            ttk.Label(individual_frame, text=f"서보 {i+1}:").grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)
            
            # 속도
            speed_var = tk.DoubleVar(value=50.0)
            self.individual_speed_vars.append(speed_var)
            ttk.Entry(individual_frame, textvariable=speed_var, width=8).grid(row=i, column=1, padx=2, pady=2)
            ttk.Label(individual_frame, text="deg/s").grid(row=i, column=2, padx=2, pady=2)
            
            # 가속도
            accel_var = tk.DoubleVar(value=30.0)
            self.individual_accel_vars.append(accel_var)
            ttk.Entry(individual_frame, textvariable=accel_var, width=8).grid(row=i, column=3, padx=2, pady=2)
            ttk.Label(individual_frame, text="deg/s²").grid(row=i, column=4, padx=2, pady=2)
        
        # 개별 설정 적용 버튼
        ttk.Button(individual_frame, text="개별 설정 적용", command=self.apply_individual_settings).grid(row=6, column=0, columnspan=5, pady=10)
    
    def setup_status_monitoring_tab(self):
        """상태 모니터링 탭 설정"""
        status_frame = ttk.Frame(self.notebook)
        self.notebook.add(status_frame, text="상태 모니터링")
        
        # 서보 상태 표시
        status_display_frame = ttk.LabelFrame(status_frame, text="서보 상태", padding="10")
        status_display_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 서보 상태 테이블 (기본 상태)
        columns = ('서보', '현재각도', '목표각도', '오차', '상태')
        self.status_tree = ttk.Treeview(status_display_frame, columns=columns, show='headings', height=8)
        
        for col in columns:
            self.status_tree.heading(col, text=col)
            self.status_tree.column(col, width=100, anchor='center')
        
        self.status_tree.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 스크롤바
        scrollbar = ttk.Scrollbar(status_display_frame, orient=tk.VERTICAL, command=self.status_tree.yview)
        scrollbar.grid(row=0, column=2, sticky=(tk.N, tk.S))
        self.status_tree.configure(yscrollcommand=scrollbar.set)
        
        # 상태 업데이트 버튼
        ttk.Button(status_display_frame, text="상태 새로고침", command=self.refresh_status).grid(row=1, column=0, columnspan=2, pady=10)
        
        # 동기화 결과 표시 (상세 결과)
        result_display_frame = ttk.LabelFrame(status_frame, text="동기화 완료 결과", padding="10")
        result_display_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 동기화 결과 테이블
        result_columns = ('서보', '현재각도', '목표각도', '실제속도', '실제가속도', '정확도', '상태')
        self.result_tree = ttk.Treeview(result_display_frame, columns=result_columns, show='headings', height=8)
        
        for col in result_columns:
            self.result_tree.heading(col, text=col)
            self.result_tree.column(col, width=90, anchor='center')
        
        self.result_tree.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 결과 스크롤바
        result_scrollbar = ttk.Scrollbar(result_display_frame, orient=tk.VERTICAL, command=self.result_tree.yview)
        result_scrollbar.grid(row=0, column=2, sticky=(tk.N, tk.S))
        self.result_tree.configure(yscrollcommand=result_scrollbar.set)
        
        # 결과 지우기 버튼
        ttk.Button(result_display_frame, text="결과 지우기", command=self.clear_results).grid(row=1, column=0, columnspan=2, pady=10)
        
        # 로그 표시
        log_frame = ttk.LabelFrame(status_frame, text="시스템 로그", padding="10")
        log_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.log_text = tk.Text(log_frame, height=15, width=50)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        
        # 로그 지우기 버튼
        ttk.Button(log_frame, text="로그 지우기", command=self.clear_log).grid(row=1, column=0, columnspan=2, pady=5)
    
    def send_angles(self):
        """기본 각도 제어 메시지 전송"""
        angles = [var.get() for var in self.angle_vars]
        
        msg = Float32MultiArray()
        msg.data = angles
        
        self.angle_pub.publish(msg)
        self.log_message(f"각도 제어 전송: {angles}")
        
        # 연결 상태를 "전송 중"으로 표시
        self.connection_label.config(text="연결 상태: 📤 전송 중...", foreground="blue")
    
    def send_angles_with_speed(self):
        """속도/가속도 포함 각도 제어 메시지 전송"""
        angles = [var.get() for var in self.angle_speed_vars]
        speed = self.speed_var.get()
        accel = self.accel_var.get()
        
        msg = Float32MultiArray()
        msg.data = angles + [speed, accel]
        
        print(f"GUI 전송: 각도={angles}, 속도={speed}, 가속도={accel}")
        print(f"메시지 데이터 크기: {len(msg.data)}")
        print(f"메시지 데이터: {msg.data}")
        
        self.angle_speed_pub.publish(msg)
        self.log_message(f"각도+속도+가속도 제어 전송: 각도={angles}, 속도={speed}, 가속도={accel}")
        
        # 연결 상태를 "전송 중"으로 표시
        self.connection_label.config(text="연결 상태: 📤 전송 중...", foreground="blue")
    
    def send_coordinates(self):
        """기본 좌표 제어 메시지 전송"""
        coords = [
            self.x_var.get(),
            self.y_var.get(),
            self.z_var.get(),
            self.roll_var.get(),
            self.pitch_var.get(),
            self.yaw_var.get()
        ]
        
        msg = Float32MultiArray()
        msg.data = coords
        
        print(f"GUI 전송: 좌표={coords}")
        print(f"메시지 데이터 크기: {len(msg.data)}")
        print(f"메시지 데이터: {msg.data}")
        
        self.coord_pub.publish(msg)
        self.log_message(f"좌표 제어 전송: {coords}")
        
        # 연결 상태를 "전송 중"으로 표시
        self.connection_label.config(text="연결 상태: 📤 전송 중...", foreground="blue")
    
    def send_coordinates_with_speed(self):
        """속도/가속도 포함 좌표 제어 메시지 전송"""
        coords = [
            self.x_speed_var.get(),
            self.y_speed_var.get(),
            self.z_speed_var.get(),
            self.roll_speed_var.get(),
            self.pitch_speed_var.get(),
            self.yaw_speed_var.get()
        ]
        speed = self.coord_speed_var.get()
        accel = self.coord_accel_var.get()
        
        msg = Float32MultiArray()
        msg.data = coords + [speed, accel]
        
        print(f"GUI 전송: 좌표={coords}, 속도={speed}, 가속도={accel}")
        print(f"메시지 데이터 크기: {len(msg.data)}")
        print(f"메시지 데이터: {msg.data}")
        
        self.coord_speed_pub.publish(msg)
        self.log_message(f"좌표+속도+가속도 제어 전송: 좌표={coords}, 속도={speed}, 가속도={accel}")
        
        # 연결 상태를 "전송 중"으로 표시
        self.connection_label.config(text="연결 상태: 📤 전송 중...", foreground="blue")
    
    def apply_sync_settings(self):
        """동기화 설정 적용"""
        speed = self.sync_speed_var.get()
        accel = self.sync_accel_var.get()
        timeout = self.sync_timeout_var.get()
        min_ratio = self.sync_min_ratio_var.get()
        
        msg = Float32MultiArray()
        msg.data = [speed, accel, float(timeout), min_ratio]
        
        self.sync_settings_pub.publish(msg)
        self.log_message(f"동기화 설정 적용: 속도={speed}, 가속도={accel}, 타임아웃={timeout}, 최소비율={min_ratio}")
    
    def apply_individual_settings(self):
        """개별 서보 설정 적용"""
        for i in range(6):
            speed = self.individual_speed_vars[i].get()
            accel = self.individual_accel_vars[i].get()
            
            # 개별 서보 설정은 CAN 메시지로 직접 전송
            # 여기서는 로그만 출력
            self.log_message(f"서보 {i+1} 개별 설정: 속도={speed}, 가속도={accel}")
    
    def setup_path_control_tab(self):
        """경로 제어 탭 설정 (두 점 사이 보간 이동)"""
        path_frame = ttk.Frame(self.notebook)
        self.notebook.add(path_frame, text="경로 제어")
        
        # 시작점 설정
        start_frame = ttk.LabelFrame(path_frame, text="시작점 좌표", padding="10")
        start_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(start_frame, text="X (mm):").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_x_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_x_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(start_frame, text="Y (mm):").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_y_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_y_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(start_frame, text="Z (mm):").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_z_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_z_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(start_frame, text="Roll (도):").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_roll_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_roll_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(start_frame, text="Pitch (도):").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_pitch_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_pitch_var, width=10).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(start_frame, text="Yaw (도):").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.start_yaw_var = tk.DoubleVar(value=0.0)
        ttk.Entry(start_frame, textvariable=self.start_yaw_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        # 끝점 설정
        end_frame = ttk.LabelFrame(path_frame, text="끝점 좌표", padding="10")
        end_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(end_frame, text="X (mm):").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_x_var = tk.DoubleVar(value=100.0)
        ttk.Entry(end_frame, textvariable=self.end_x_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(end_frame, text="Y (mm):").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_y_var = tk.DoubleVar(value=100.0)
        ttk.Entry(end_frame, textvariable=self.end_y_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(end_frame, text="Z (mm):").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_z_var = tk.DoubleVar(value=100.0)
        ttk.Entry(end_frame, textvariable=self.end_z_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(end_frame, text="Roll (도):").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_roll_var = tk.DoubleVar(value=0.0)
        ttk.Entry(end_frame, textvariable=self.end_roll_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(end_frame, text="Pitch (도):").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_pitch_var = tk.DoubleVar(value=0.0)
        ttk.Entry(end_frame, textvariable=self.end_pitch_var, width=10).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(end_frame, text="Yaw (도):").grid(row=5, column=0, padx=5, pady=2, sticky=tk.W)
        self.end_yaw_var = tk.DoubleVar(value=0.0)
        ttk.Entry(end_frame, textvariable=self.end_yaw_var, width=10).grid(row=5, column=1, padx=5, pady=2)
        
        # 경로 파라미터 설정
        param_frame = ttk.LabelFrame(path_frame, text="경로 파라미터", padding="10")
        param_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        ttk.Label(param_frame, text="보간 포인트 개수:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.interpolation_points_var = tk.IntVar(value=10)
        ttk.Entry(param_frame, textvariable=self.interpolation_points_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="개 (권장: 5-20)").grid(row=0, column=2, padx=5, pady=2)
        
        ttk.Label(param_frame, text="최소 전송 간격:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.path_interval_var = tk.DoubleVar(value=0.05)  # 50ms (중간 속도 보간)
        ttk.Entry(param_frame, textvariable=self.path_interval_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="초 (보간: 0.01~0.2초, 권장: 0.05초)").grid(row=1, column=2, padx=5, pady=2)
        
        ttk.Label(param_frame, text="이동 속도:").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.path_speed_var = tk.DoubleVar(value=30.0)
        ttk.Entry(param_frame, textvariable=self.path_speed_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="deg/s").grid(row=2, column=2, padx=5, pady=2)
        
        ttk.Label(param_frame, text="가속도:").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.path_accel_var = tk.DoubleVar(value=20.0)
        ttk.Entry(param_frame, textvariable=self.path_accel_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        ttk.Label(param_frame, text="deg/s²").grid(row=3, column=2, padx=5, pady=2)
        
        # 🚀 실행 모드 선택
        ttk.Label(param_frame, text="실행 모드:").grid(row=4, column=0, padx=5, pady=2, sticky=tk.W)
        self.fast_mode_var = tk.BooleanVar(value=True)
        fast_mode_check = ttk.Checkbutton(
            param_frame, 
            text="고속 모드 (ACK 없음)", 
            variable=self.fast_mode_var
        )
        fast_mode_check.grid(row=4, column=1, columnspan=2, padx=5, pady=2, sticky=tk.W)
        
        # 제어 버튼들
        control_frame = ttk.Frame(path_frame)
        control_frame.grid(row=2, column=0, columnspan=2, pady=10)
        
        self.path_start_btn = ttk.Button(
            control_frame, 
            text="▶️ 경로 실행", 
            command=self.start_path_execution
        )
        self.path_start_btn.grid(row=0, column=0, padx=5)
        
        self.path_stop_btn = ttk.Button(
            control_frame, 
            text="⏹️ 경로 정지", 
            command=self.stop_path_execution,
            state='disabled'
        )
        self.path_stop_btn.grid(row=0, column=1, padx=5)
        
        ttk.Button(
            control_frame, 
            text="↻ 왕복 실행", 
            command=self.start_path_roundtrip
        ).grid(row=0, column=2, padx=5)
        
        # 경로 상태 표시
        self.path_status_label = ttk.Label(control_frame, text="경로 상태: 대기 중", foreground="gray")
        self.path_status_label.grid(row=1, column=0, columnspan=3, pady=5)
        
        # 경로 프리셋
        preset_frame = ttk.LabelFrame(path_frame, text="경로 프리셋", padding="10")
        preset_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        path_presets = [
            ("수평 이동", [0, 0, 100, 0, 0, 0], [200, 0, 100, 0, 0, 0]),
            ("수직 이동", [0, 0, 50, 0, 0, 0], [0, 0, 200, 0, 0, 0]),
            ("대각선 이동", [0, 0, 0, 0, 0, 0], [150, 150, 150, 0, 0, 0]),
            ("원호 이동", [100, 0, 100, 0, 0, 0], [100, 200, 100, 0, 0, 90])
        ]
        
        for i, (name, start, end) in enumerate(path_presets):
            ttk.Button(
                preset_frame, 
                text=name, 
                command=lambda s=start, e=end: self.set_path_preset(s, e)
            ).grid(row=i//2, column=i%2, padx=5, pady=2)
        
        # 📌 중요 안내 메시지
        info_frame = ttk.LabelFrame(path_frame, text="⚠️ 중요 안내", padding="10")
        info_frame.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        info_text = (
            "• 🚀 고속 모드 (권장): ACK 처리 없이 빠른 실행, 최소 간격 0.3초\n"
            "• 🔄 일반 모드: 동기화 + ACK 처리, 안정적이지만 느림, 최소 간격 0.8초\n"
            "• 전송 간격: IK 계산 시간 + 서보 이동 시간 고려\n"
            "• 적절한 포인트 개수: 5-20개 (고속 모드는 더 많이 가능)\n"
            "• 안전한 테스트: 고속 모드 + 5포인트 + 0.5초 간격부터 시작"
        )
        
        info_label = ttk.Label(info_frame, text=info_text, foreground="darkblue", 
                              font=('Arial', 9), justify=tk.LEFT)
        info_label.grid(row=0, column=0, sticky=tk.W)
    
    def set_preset_angles(self, angles):
        """프리셋 각도 설정"""
        for i, angle in enumerate(angles):
            if i < len(self.angle_vars):
                self.angle_vars[i].set(angle)
                self.angle_speed_vars[i].set(angle)
    
    def set_preset_coordinates(self, coords):
        """프리셋 좌표 설정"""
        if len(coords) >= 6:
            self.x_var.set(coords[0])
            self.y_var.set(coords[1])
            self.z_var.set(coords[2])
            self.roll_var.set(coords[3])
            self.pitch_var.set(coords[4])
            self.yaw_var.set(coords[5])
            
            self.x_speed_var.set(coords[0])
            self.y_speed_var.set(coords[1])
            self.z_speed_var.set(coords[2])
            self.roll_speed_var.set(coords[3])
            self.pitch_speed_var.set(coords[4])
            self.yaw_speed_var.set(coords[5])
    
    def set_path_preset(self, start_coords, end_coords):
        """경로 프리셋 설정"""
        self.start_x_var.set(start_coords[0])
        self.start_y_var.set(start_coords[1])
        self.start_z_var.set(start_coords[2])
        self.start_roll_var.set(start_coords[3])
        self.start_pitch_var.set(start_coords[4])
        self.start_yaw_var.set(start_coords[5])
        
        self.end_x_var.set(end_coords[0])
        self.end_y_var.set(end_coords[1])
        self.end_z_var.set(end_coords[2])
        self.end_roll_var.set(end_coords[3])
        self.end_pitch_var.set(end_coords[4])
        self.end_yaw_var.set(end_coords[5])
        
        self.log_message(f"경로 프리셋 설정: 시작={start_coords}, 끝={end_coords}")
    
    def interpolate_path(self, start, end, num_points):
        """두 점 사이를 선형 보간하여 경로 생성"""
        start_array = np.array(start)
        end_array = np.array(end)
        
        # 선형 보간
        path = []
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            point = start_array + t * (end_array - start_array)
            path.append(point.tolist())
        
        return path
    
    def execute_path(self, path, interval, speed, accel, fast_mode=False):
        """❌ 이 함수는 더 이상 사용되지 않습니다. 삭제 예정입니다."""
        print("=" * 60)
        print("⚠️ execute_path() 함수가 호출되었습니다!")
        print("❌ 이 함수는 deprecated되었습니다!")
        print("✅ start_path_execution()을 대신 사용하세요!")
        print("=" * 60)
        self.log_message("❌ 이전 방식의 경로 실행은 더 이상 지원되지 않습니다.")
        self.log_message("✅ 마스터에서 보간 처리를 하도록 변경되었습니다.")
        self.log_message("📌 경로 실행 버튼을 다시 클릭해주세요.")
        
        # 바로 종료
        return
    
    def start_path_execution(self):
        """경로 실행 시작 - 새로운 방식: 두 점만 마스터에 전송"""
        import sys
        sys.stdout.write("=" * 60 + "\n")
        sys.stdout.write("🚀 start_path_execution() 함수 호출됨!\n")
        sys.stdout.write("=" * 60 + "\n")
        sys.stdout.write(f"현재 path_executing 상태: {self.path_executing}\n")
        sys.stdout.flush()
        
        print("=" * 60)
        print("🚀 start_path_execution() 함수 호출됨!")
        print("=" * 60)
        print(f"현재 path_executing 상태: {self.path_executing}")
        
        if self.path_executing:
            print("⚠️ 이미 경로가 실행 중입니다.")
            self.log_message("이미 경로가 실행 중입니다.")
            return
        
        # 파라미터 가져오기
        num_points = self.interpolation_points_var.get()
        interval = self.path_interval_var.get()
        speed = self.path_speed_var.get()
        accel = self.path_accel_var.get()
        fast_mode = self.fast_mode_var.get()
        
        # ✅ 값 검증 및 디버그 출력
        print(f"📋 파라미터: num_points={num_points}, interval={interval}, speed={speed}, accel={accel}, fast_mode={fast_mode}")
        print(f"📋 interval 상세: 타입={type(interval)}, 값={interval}, 변수값={self.path_interval_var.get()}")
        
        # 🔧 안전성 검증
        if num_points < 2 or num_points > 100:
            print(f"❌ 보간 포인트 오류: {num_points}")
            self.log_message("❌ 보간 포인트는 2-100개 사이여야 합니다.")
            return
        
        # ✅ interval 값 검증 및 보정
        if interval < 0.01:
            print(f"⚠️ interval 값이 너무 작음 ({interval}), 0.01로 보정")
            interval = 0.01
        if interval > 2.0:
            print(f"⚠️ interval 값이 너무 큼 ({interval}), 2.0으로 보정")
            interval = 2.0
        
        # ✅ 검증 완료 - 보정된 interval 사용
        
        # 시작점과 끝점 가져오기
        start = [
            self.start_x_var.get(),
            self.start_y_var.get(),
            self.start_z_var.get(),
            self.start_roll_var.get(),
            self.start_pitch_var.get(),
            self.start_yaw_var.get()
        ]
        
        end = [
            self.end_x_var.get(),
            self.end_y_var.get(),
            self.end_z_var.get(),
            self.end_roll_var.get(),
            self.end_pitch_var.get(),
            self.end_yaw_var.get()
        ]
        
        print(f"📍 시작점: {start}")
        print(f"📍 끝점: {end}")
        
        # 🎯 새로운 방식: 두 점만 마스터에 전송
        print("📤 send_path_command() 호출 중...")
        self.send_path_command(start, end, num_points, interval, speed, accel, fast_mode)
        print("✅ send_path_command() 호출 완료")
        print("=" * 60)
        
        # GUI 상태 업데이트
        self.path_executing = True
        self.path_start_btn.config(state='disabled')
        self.path_stop_btn.config(state='normal')
        
        # 예상 실행 시간 계산
        estimated_time = num_points * interval
        self.path_status_label.config(
            text=f"경로 실행 중... (예상: {estimated_time:.1f}초)", 
            foreground="blue"
        )
        
        mode_text = "🚀 고속" if fast_mode else "🔄 일반"
        self.log_message(f"✅ 경로 명령 전송 ({mode_text} 모드):")
        self.log_message(f"  - 시작점: {start}")
        self.log_message(f"  - 끝점: {end}")
        self.log_message(f"  - 보간 포인트: {num_points}개")
        self.log_message(f"  - 전송 간격: {interval}초")
        self.log_message(f"  - 예상 시간: {estimated_time:.1f}초")
        self.log_message(f"  - 속도: {speed} deg/s, 가속도: {accel} deg/s²")
        self.log_message("  - 마스터에서 보간 계산 및 실행 중...")
        
        # 일정 시간 후 자동으로 완료 표시 (실제 완료는 마스터에서 처리됨)
        estimated_ms = int(estimated_time * 1000)
        self.root.after(estimated_ms + 500, self.path_execution_complete)
    
    def send_path_command(self, start_coords, end_coords, num_points, interval, speed, accel, fast_mode):
        """두 점 사이 경로를 마스터에 위임"""
        msg = Float32MultiArray()
        # 데이터: [start_x, start_y, start_z, start_roll, start_pitch, start_yaw,
        #          end_x, end_y, end_z, end_roll, end_pitch, end_yaw,
        #          num_points, interval, speed, accel, fast_mode]
        msg.data = start_coords + end_coords + [num_points, interval, speed, accel, float(fast_mode)]
        
        # 디버그 출력
        import sys
        sys.stdout.write("=" * 80 + "\n")
        sys.stdout.write("📤 [GUI → 마스터] 경로 명령 전송\n")
        sys.stdout.write("=" * 80 + "\n")
        sys.stdout.flush()
        
        print("=" * 80)
        print("📤 [GUI → 마스터] 경로 명령 전송")
        print("=" * 80)
        print(f"시작점: {start_coords}")
        print(f"끝점: {end_coords}")
        print(f"파라미터: num_points={num_points}, interval={interval}, speed={speed}, accel={accel}, fast_mode={fast_mode}")
        print(f"총 데이터 크기: {len(msg.data)} (예상: 17)")
        print("\n전체 데이터 배열:")
        for i, val in enumerate(msg.data):
            print(f"  [{i:2d}] = {val:10.2f}")
        print("=" * 80)
        
        self.log_message(f"📤 path_command 메시지 전송: {len(msg.data)}개 데이터")
        self.path_command_pub.publish(msg)
        
        print("✅ 경로 명령 발행 완료")
        print("=" * 80)
    
    def path_execution_complete(self):
        """경로 실행 완료 처리"""
        if not self.path_executing:
            return  # 이미 정지됨
        
        self.path_executing = False
        self.path_start_btn.config(state='normal')
        self.path_stop_btn.config(state='disabled')
        self.path_status_label.config(text="경로 실행 완료!", foreground="green")
        self.log_message("✅ 경로 실행이 완료되었습니다.")
    
    def stop_path_execution(self):
        """경로 실행 정지"""
        self.path_executing = False
        self.path_start_btn.config(state='normal')
        self.path_stop_btn.config(state='disabled')
        self.path_status_label.config(text="경로 실행 정지됨", foreground="red")
        self.log_message("경로 실행 정지 요청")
    
    def start_path_roundtrip(self):
        """왕복 경로 실행 - 새로운 방식: 마스터에서 처리"""
        if self.path_executing:
            self.log_message("이미 경로가 실행 중입니다.")
            return
        
        # 파라미터 가져오기
        num_points = self.interpolation_points_var.get()
        interval = self.path_interval_var.get()
        speed = self.path_speed_var.get()
        accel = self.path_accel_var.get()
        fast_mode = self.fast_mode_var.get()
        
        # 🔧 안전성 검증
        if num_points < 2 or num_points > 100:
            self.log_message("❌ 보간 포인트는 2-100개 사이여야 합니다.")
            return
        
        if interval < 0.01 or interval > 2.0:
            self.log_message("❌ 전송 간격은 0.01-2.0초 사이여야 합니다.")
            return
        
        # 시작점과 끝점 가져오기
        start = [
            self.start_x_var.get(),
            self.start_y_var.get(),
            self.start_z_var.get(),
            self.start_roll_var.get(),
            self.start_pitch_var.get(),
            self.start_yaw_var.get()
        ]
        
        end = [
            self.end_x_var.get(),
            self.end_y_var.get(),
            self.end_z_var.get(),
            self.end_roll_var.get(),
            self.end_pitch_var.get(),
            self.end_yaw_var.get()
        ]
        
        # 왕복 경로: 시작->끝 (첫 번째 경로)
        self.send_path_command(start, end, num_points, interval, speed, accel, fast_mode)
        
        # GUI 상태 업데이트
        self.path_executing = True
        self.path_start_btn.config(state='disabled')
        self.path_stop_btn.config(state='normal')
        
        # 예상 실행 시간 계산 (왕복은 2배)
        estimated_time = num_points * interval * 2
        self.path_status_label.config(
            text=f"왕복 경로 실행 중... (예상: {estimated_time:.1f}초)", 
            foreground="blue"
        )
        
        mode_text = "🚀 고속" if fast_mode else "🔄 일반"
        
        self.log_message(f"✅ 왕복 경로 시작 ({mode_text} 모드):")
        self.log_message(f"  - 1차: 시작점 → 끝점 ({num_points}개 보간)")
        self.log_message(f"  - 2차: 끝점 → 시작점 (자동, 마스터에서 처리)")
        self.log_message(f"  - 총 보간 포인트: {num_points * 2}개")
        self.log_message(f"  - 전송 간격: {interval}초")
        self.log_message(f"  - 예상 시간: {estimated_time:.1f}초")
        
        # TODO: 왕복 경로를 완전히 구현하려면 마스터에서 왕복 명령을 지원해야 함
        # 현재는 첫 번째 경로만 전송됨
        self.log_message("⚠️ 왕복 경로는 현재 첫 번째 경로만 실행됩니다.")
        
        # 일정 시간 후 자동으로 완료 표시
        estimated_ms = int(estimated_time * 1000)
        self.root.after(estimated_ms + 500, self.path_execution_complete)
    
    def emergency_stop(self):
        """긴급 정지"""
        # 경로 실행 중지
        self.stop_path_execution()
        
        # 모든 서보를 0도로 이동
        self.set_preset_angles([0, 0, 0, 0, 0, 0])
        self.send_angles()
        self.log_message("🚨 긴급 정지 실행!")
        messagebox.showwarning("긴급 정지", "긴급 정지가 실행되었습니다!")
    
    def go_home(self):
        """홈 포지션으로 이동"""
        self.set_preset_angles([0, 0, 0, 0, 0, 0])
        self.send_angles()
        self.log_message("홈 포지션으로 이동")
    
    def servo_status_callback(self, msg):
        """서보 상태 피드백 수신"""
        # 연결 상태 업데이트
        self.last_message_time = time.time()
        self.connection_status = "연결됨"
        
        if len(msg.data) >= 18:  # 각 서보당 3개 값 (현재각도, 목표각도, 오차)
            self.update_status_display(msg.data)
            self.log_message(f"서보 상태 수신: {len(msg.data)}개 데이터")
            
        if len(msg.data) >= 24:  # 동기화 완료 결과 (각 서보당 4개 값)
            self.update_result_display(msg.data)
            self.log_message(f"동기화 완료 결과 수신: {len(msg.data)}개 데이터")
    
    def update_status_display(self, data):
        """상태 표시 업데이트"""
        # 기존 항목 삭제
        for item in self.status_tree.get_children():
            self.status_tree.delete(item)
        
        # 새 상태 추가
        for i in range(6):
            if i * 3 + 2 < len(data):
                current_angle = data[i * 3]
                target_angle = data[i * 3 + 1]
                error = data[i * 3 + 2]
                status = "✅" if abs(error) < 2.0 else "⚠️"
                
                self.status_tree.insert('', 'end', values=(
                    f"서보 {i+1}",
                    f"{current_angle:.1f}°",
                    f"{target_angle:.1f}°",
                    f"{error:.1f}°",
                    status
                ))
    
    def refresh_status(self):
        """상태 새로고침"""
        self.log_message("상태 새로고침 요청")
        # 실제로는 서보 상태 요청 메시지를 전송해야 함
    
    def log_message(self, message):
        """로그 메시지 추가"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
    
    def clear_log(self):
        """로그 지우기"""
        self.log_text.delete(1.0, tk.END)
    
    def update_result_display(self, data):
        """동기화 완료 결과 표시 업데이트"""
        # 기존 항목 삭제
        for item in self.result_tree.get_children():
            self.result_tree.delete(item)
        
        # 새 결과 추가 (각 서보당 4개 값: 현재각도, 목표각도, 실제속도, 실제가속도)
        for i in range(6):
            if i * 4 + 3 < len(data):
                current_angle = data[i * 4]
                target_angle = data[i * 4 + 1]
                actual_speed = data[i * 4 + 2]
                actual_accel = data[i * 4 + 3]
                error = abs(current_angle - target_angle)
                status = "✅" if error < 2.0 else "⚠️"
                
                self.result_tree.insert('', 'end', values=(
                    f"서보 {i+1}",
                    f"{current_angle:.1f}°",
                    f"{target_angle:.1f}°",
                    f"{actual_speed:.1f}",
                    f"{actual_accel:.1f}",
                    f"{error:.1f}°",
                    status
                ))
    
    def clear_results(self):
        """동기화 결과 지우기"""
        for item in self.result_tree.get_children():
            self.result_tree.delete(item)
        self.log_message("동기화 결과가 지워졌습니다.")
    
    def check_connection_status(self):
        """연결 상태 확인"""
        current_time = time.time()
        time_since_last_message = current_time - self.last_message_time
        
        if time_since_last_message < 5.0:  # 5초 이내에 메시지 수신
            if self.connection_status != "연결됨":
                self.connection_status = "연결됨"
                self.connection_label.config(text="연결 상태: ✅ 연결됨", foreground="green")
        else:  # 5초 이상 메시지 없음
            if self.connection_status != "연결 끊김":
                self.connection_status = "연결 끊김"
                self.connection_label.config(text="연결 상태: ❌ 연결 끊김", foreground="red")
    
    def setup_waypoint_planning_tab(self):
        """웨이포인트 기반 경로 계획 탭 설정"""
        waypoint_frame = ttk.Frame(self.notebook)
        self.notebook.add(waypoint_frame, text="🎯 웨이포인트 경로")
        
        # 좌표 입력 섹션
        coord_input_frame = ttk.LabelFrame(waypoint_frame, text="경로점 입력", padding="10")
        coord_input_frame.grid(row=0, column=0, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # X, Y, Z 입력
        ttk.Label(coord_input_frame, text="X (m):").grid(row=0, column=0, sticky=tk.W)
        self.wp_x_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_x_entry.grid(row=0, column=1, padx=5)
        self.wp_x_entry.insert(0, "0.3")
        
        ttk.Label(coord_input_frame, text="Y (m):").grid(row=0, column=2, sticky=tk.W, padx=(10,0))
        self.wp_y_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_y_entry.grid(row=0, column=3, padx=5)
        self.wp_y_entry.insert(0, "0.0")
        
        ttk.Label(coord_input_frame, text="Z (m):").grid(row=0, column=4, sticky=tk.W, padx=(10,0))
        self.wp_z_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_z_entry.grid(row=0, column=5, padx=5)
        self.wp_z_entry.insert(0, "0.5")
        
        # Roll, Pitch, Yaw 입력
        ttk.Label(coord_input_frame, text="Roll (deg):").grid(row=1, column=0, sticky=tk.W, pady=(5,0))
        self.wp_rx_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_rx_entry.grid(row=1, column=1, padx=5, pady=(5,0))
        self.wp_rx_entry.insert(0, "0")
        
        ttk.Label(coord_input_frame, text="Pitch (deg):").grid(row=1, column=2, sticky=tk.W, padx=(10,0), pady=(5,0))
        self.wp_ry_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_ry_entry.grid(row=1, column=3, padx=5, pady=(5,0))
        self.wp_ry_entry.insert(0, "0")
        
        ttk.Label(coord_input_frame, text="Yaw (deg):").grid(row=1, column=4, sticky=tk.W, padx=(10,0), pady=(5,0))
        self.wp_rz_entry = ttk.Entry(coord_input_frame, width=10)
        self.wp_rz_entry.grid(row=1, column=5, padx=5, pady=(5,0))
        self.wp_rz_entry.insert(0, "0")
        
        # 버튼들
        ttk.Button(coord_input_frame, text="경로점 추가", command=self.add_waypoint).grid(
            row=2, column=0, columnspan=2, pady=(10,0), sticky=tk.W+tk.E)
        ttk.Button(coord_input_frame, text="현재 위치", command=self.get_current_position).grid(
            row=2, column=2, columnspan=2, pady=(10,0), padx=(5,0), sticky=tk.W+tk.E)
        ttk.Button(coord_input_frame, text="Home 위치", command=self.goto_home_position).grid(
            row=2, column=4, columnspan=2, pady=(10,0), padx=(5,0), sticky=tk.W+tk.E)
        
        # 경로점 목록
        waypoint_list_frame = ttk.LabelFrame(waypoint_frame, text="경로점 목록", padding="10")
        waypoint_list_frame.grid(row=0, column=1, padx=10, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S), rowspan=2)
        
        scrollbar = ttk.Scrollbar(waypoint_list_frame, orient=tk.VERTICAL)
        self.waypoint_listbox = tk.Listbox(waypoint_list_frame, height=12, width=40, yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.waypoint_listbox.yview)
        
        self.waypoint_listbox.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 목록 관리 버튼
        btn_frame = ttk.Frame(waypoint_list_frame)
        btn_frame.grid(row=1, column=0, columnspan=2, pady=(5,0))
        
        ttk.Button(btn_frame, text="선택 삭제", command=self.delete_waypoint).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="전체 삭제", command=self.clear_waypoints).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="위로", command=self.move_waypoint_up).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="아래로", command=self.move_waypoint_down).pack(side=tk.LEFT, padx=2)
        
        # 💾 저장/불러오기 버튼
        save_load_frame = ttk.Frame(waypoint_list_frame)
        save_load_frame.grid(row=2, column=0, columnspan=2, pady=(5,0))
        
        ttk.Button(save_load_frame, text="💾 경로 저장", command=self.save_waypoints).pack(side=tk.LEFT, padx=2)
        ttk.Button(save_load_frame, text="📂 경로 불러오기", command=self.load_waypoints).pack(side=tk.LEFT, padx=2)
        
        # 제어 파라미터
        param_frame = ttk.LabelFrame(waypoint_frame, text="경로 파라미터", padding="10")
        param_frame.grid(row=1, column=0, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        # 보간 간격
        ttk.Label(param_frame, text="보간 간격 (m):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.wp_interpolation_step = ttk.Entry(param_frame, width=15)
        self.wp_interpolation_step.grid(row=0, column=1, padx=5, pady=5)
        self.wp_interpolation_step.insert(0, "0.01")
        
        # 속도
        ttk.Label(param_frame, text="속도 (deg/s):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.wp_speed_entry = ttk.Entry(param_frame, width=15)
        self.wp_speed_entry.grid(row=1, column=1, padx=5, pady=5)
        self.wp_speed_entry.insert(0, "50")
        
        # 가속도
        ttk.Label(param_frame, text="가속도 (deg/s²):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.wp_accel_entry = ttk.Entry(param_frame, width=15)
        self.wp_accel_entry.grid(row=2, column=1, padx=5, pady=5)
        self.wp_accel_entry.insert(0, "30")
        
        # 딜레이
        ttk.Label(param_frame, text="포인트간 딜레이 (ms):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.wp_delay_entry = ttk.Entry(param_frame, width=15)
        self.wp_delay_entry.grid(row=3, column=1, padx=5, pady=5)
        self.wp_delay_entry.insert(0, "50")
        
        # 경로 타입 선택
        ttk.Label(param_frame, text="경로 타입:").grid(row=4, column=0, sticky=tk.W, pady=5)
        self.path_type_var = tk.StringVar(value="linear")
        path_type_combo = ttk.Combobox(param_frame, textvariable=self.path_type_var, width=13, state='readonly')
        path_type_combo['values'] = ('linear', 'circular', 'bezier')
        path_type_combo.grid(row=4, column=1, padx=5, pady=5)
        
        # 실행 버튼
        exec_frame = ttk.Frame(waypoint_frame)
        exec_frame.grid(row=2, column=0, columnspan=2, pady=10)
        
        ttk.Button(exec_frame, text="📐 경로 생성 (보간 + IK)", 
                  command=self.generate_trajectory, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Button(exec_frame, text="📋 데이터 확인", 
                  command=self.show_trajectory_data, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(exec_frame, text="🎨 3D 시각화", 
                  command=self.visualize_3d_path, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(exec_frame, text="▶️ 경로 실행", 
                  command=self.execute_waypoint_trajectory, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(exec_frame, text="⏹️ 정지", 
                  command=self.stop_waypoint_execution, width=12).pack(side=tk.LEFT, padx=5)
        
        # 📊 진행 상황 프로그레스바
        progress_frame = ttk.Frame(waypoint_frame)
        progress_frame.grid(row=3, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        self.wp_progress = ttk.Progressbar(progress_frame, length=400, mode='determinate')
        self.wp_progress.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        
        # 상태 표시
        self.wp_status_label = ttk.Label(progress_frame, text="상태: 대기 중", foreground="gray")
        self.wp_status_label.pack(side=tk.LEFT, padx=10)
        
        # 📊 경로 통계 정보
        stats_frame = ttk.LabelFrame(waypoint_frame, text="경로 통계", padding="10")
        stats_frame.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky=(tk.W, tk.E))
        
        self.stats_text = tk.Text(stats_frame, height=4, width=60, state=tk.DISABLED)
        self.stats_text.pack(fill=tk.BOTH, expand=True)
    
    def add_waypoint(self):
        """경로점 추가"""
        try:
            x = float(self.wp_x_entry.get())
            y = float(self.wp_y_entry.get())
            z = float(self.wp_z_entry.get())
            rx = float(self.wp_rx_entry.get())
            ry = float(self.wp_ry_entry.get())
            rz = float(self.wp_rz_entry.get())
            
            waypoint = (x, y, z, rx, ry, rz)
            self.waypoints.append(waypoint)
            
            self.waypoint_listbox.insert(tk.END, 
                f"P{len(self.waypoints)}: ({x:.3f}, {y:.3f}, {z:.3f}), ({rx:.1f}°, {ry:.1f}°, {rz:.1f}°)")
            
            self.log_message(f"경로점 추가: {waypoint}")
            
        except ValueError:
            messagebox.showerror("입력 오류", "올바른 숫자를 입력하세요.")
    
    def delete_waypoint(self):
        """선택된 경로점 삭제"""
        selection = self.waypoint_listbox.curselection()
        if selection:
            idx = selection[0]
            self.waypoints.pop(idx)
            self.waypoint_listbox.delete(idx)
            # 번호 재정렬
            self.refresh_waypoint_list()
            self.log_message(f"경로점 {idx+1} 삭제됨")
    
    def clear_waypoints(self):
        """모든 경로점 삭제"""
        self.waypoints.clear()
        self.waypoint_listbox.delete(0, tk.END)
        self.log_message("모든 경로점 삭제됨")
    
    def move_waypoint_up(self):
        """선택된 경로점을 위로 이동"""
        selection = self.waypoint_listbox.curselection()
        if selection and selection[0] > 0:
            idx = selection[0]
            self.waypoints[idx], self.waypoints[idx-1] = self.waypoints[idx-1], self.waypoints[idx]
            self.refresh_waypoint_list()
            self.waypoint_listbox.selection_set(idx-1)
    
    def move_waypoint_down(self):
        """선택된 경로점을 아래로 이동"""
        selection = self.waypoint_listbox.curselection()
        if selection and selection[0] < len(self.waypoints) - 1:
            idx = selection[0]
            self.waypoints[idx], self.waypoints[idx+1] = self.waypoints[idx+1], self.waypoints[idx]
            self.refresh_waypoint_list()
            self.waypoint_listbox.selection_set(idx+1)
    
    def refresh_waypoint_list(self):
        """경로점 목록 새로고침"""
        self.waypoint_listbox.delete(0, tk.END)
        for i, (x, y, z, rx, ry, rz) in enumerate(self.waypoints):
            self.waypoint_listbox.insert(tk.END, 
                f"P{i+1}: ({x:.3f}, {y:.3f}, {z:.3f}), ({rx:.1f}°, {ry:.1f}°, {rz:.1f}°)")
    
    def get_current_position(self):
        """현재 위치 가져오기 (FK 사용)"""
        pos, orient, _, _ = self.robot.forward_kinematics(self.current_angles)
        
        self.wp_x_entry.delete(0, tk.END)
        self.wp_x_entry.insert(0, f"{pos[0]:.3f}")
        
        self.wp_y_entry.delete(0, tk.END)
        self.wp_y_entry.insert(0, f"{pos[1]:.3f}")
        
        self.wp_z_entry.delete(0, tk.END)
        self.wp_z_entry.insert(0, f"{pos[2]:.3f}")
        
        self.wp_rx_entry.delete(0, tk.END)
        self.wp_rx_entry.insert(0, f"{np.degrees(orient[0]):.1f}")
        
        self.wp_ry_entry.delete(0, tk.END)
        self.wp_ry_entry.insert(0, f"{np.degrees(orient[1]):.1f}")
        
        self.wp_rz_entry.delete(0, tk.END)
        self.wp_rz_entry.insert(0, f"{np.degrees(orient[2]):.1f}")
        
        self.log_message(f"현재 위치: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    
    def goto_home_position(self):
        """Home 위치로 설정"""
        pos, orient, _, _ = self.robot.forward_kinematics(np.zeros(6))
        
        self.wp_x_entry.delete(0, tk.END)
        self.wp_x_entry.insert(0, f"{pos[0]:.3f}")
        
        self.wp_y_entry.delete(0, tk.END)
        self.wp_y_entry.insert(0, f"{pos[1]:.3f}")
        
        self.wp_z_entry.delete(0, tk.END)
        self.wp_z_entry.insert(0, f"{pos[2]:.3f}")
        
        self.wp_rx_entry.delete(0, tk.END)
        self.wp_rx_entry.insert(0, f"{np.degrees(orient[0]):.1f}")
        
        self.wp_ry_entry.delete(0, tk.END)
        self.wp_ry_entry.insert(0, f"{np.degrees(orient[1]):.1f}")
        
        self.wp_rz_entry.delete(0, tk.END)
        self.wp_rz_entry.insert(0, f"{np.degrees(orient[2]):.1f}")
        
        self.log_message("Home 위치로 설정됨")
    
    def generate_trajectory(self):
        """경로 생성 (보간 + IK)"""
        if len(self.waypoints) < 1:
            messagebox.showwarning("경고", "최소 1개의 경로점이 필요합니다.")
            return
        
        try:
            interp_step = float(self.wp_interpolation_step.get())
        except ValueError:
            messagebox.showerror("오류", "올바른 보간 간격을 입력하세요.")
            return
        
        # 프로그레스바 초기화
        self.wp_progress['value'] = 0
        self.wp_progress['maximum'] = 100
        
        self.wp_status_label.config(text="상태: 경로 생성 중...", foreground="blue")
        self.log_message("="*50)
        self.log_message("📐 경로 생성 시작...")
        
        # 경로 타입 확인
        path_type = self.path_type_var.get()
        self.log_message(f"   - 경로 타입: {path_type}")
        self.log_message(f"   - 보간 간격: {interp_step} m")
        
        # 현재 위치를 시작점으로
        current_pos, current_orient, _, _ = self.robot.forward_kinematics(self.current_angles)
        
        # 모든 경로점을 좌표+자세 배열로 변환
        all_points = []
        
        # 현재 위치 추가
        all_points.append([current_pos[0], current_pos[1], current_pos[2], 
                          current_orient[0], current_orient[1], current_orient[2]])
        
        # 웨이포인트 추가
        for waypoint in self.waypoints:
            x, y, z, rx_deg, ry_deg, rz_deg = waypoint
            all_points.append([x, y, z, 
                             np.radians(rx_deg), 
                             np.radians(ry_deg), 
                             np.radians(rz_deg)])
        
        # 프로그레스바 업데이트: 보간 생성 (0-30%)
        self.wp_progress['value'] = 10
        self.root.update_idletasks()
        
        # 경로 타입에 따른 보간
        if path_type == "circular" and len(all_points) >= 3:
            self.log_message("   - 원호 보간 사용")
            self.interpolated_points = self.interpolate_circular(all_points)
        elif path_type == "bezier" and len(all_points) >= 3:
            self.log_message("   - 베지어 보간 사용")
            self.interpolated_points = self.interpolate_bezier(all_points)
        else:
            self.log_message("   - 직선 보간 사용")
            self.interpolated_points = self.interpolate_linear(all_points)
        
        self.wp_progress['value'] = 30
        self.root.update_idletasks()
        
        self.log_message(f"✅ 보간점 생성 완료: {len(self.interpolated_points)}개")
        
        # IK 계산
        self.angle_trajectory = []
        prev_angles = self.current_angles.copy()
        
        success_count = 0
        total_points = len(self.interpolated_points)
        
        for i, point in enumerate(self.interpolated_points):
            pos = point[:3]
            orient = point[3:]
            
            # IK 수행
            angles, success = self.robot.inverse_kinematics(pos, orient, prev_angles)
            
            if success:
                success_count += 1
            
            self.angle_trajectory.append(np.degrees(angles))
            prev_angles = angles
            
            # 프로그레스바 업데이트: IK 계산 (30-100%)
            progress = 30 + int((i + 1) / total_points * 70)
            self.wp_progress['value'] = progress
            
            # 10개마다 GUI 업데이트
            if i % 10 == 0:
                self.wp_status_label.config(
                    text=f"상태: IK 계산 중... ({i+1}/{total_points})", 
                    foreground="blue"
                )
                self.root.update_idletasks()
        
        # 완료
        self.wp_progress['value'] = 100
        
        success_rate = (success_count / total_points * 100) if total_points > 0 else 0
        self.log_message(f"✅ IK 계산 완료: {success_count}/{total_points} ({success_rate:.1f}%)")
        self.log_message(f"📊 총 각도 궤적 포인트: {len(self.angle_trajectory)}개")
        self.log_message("="*50)
        self.log_message("✅ 경로 생성 완료!")
        
        # 통계 정보 업데이트
        self.update_path_statistics()
        
        self.wp_status_label.config(
            text=f"상태: 생성 완료 ({len(self.angle_trajectory)}개, {success_rate:.0f}% 성공)", 
            foreground="green"
        )
    
    def execute_waypoint_trajectory(self):
        """웨이포인트 경로 실행"""
        if len(self.angle_trajectory) == 0:
            messagebox.showwarning("경고", "먼저 경로를 생성하세요.")
            return
        
        # 별도 스레드에서 실행
        self.path_executing = True
        self.path_thread = threading.Thread(target=self._execute_waypoint_trajectory_thread, daemon=True)
        self.path_thread.start()
    
    def _execute_waypoint_trajectory_thread(self):
        """웨이포인트 경로 실행 스레드"""
        try:
            speed = float(self.wp_speed_entry.get())
            accel = float(self.wp_accel_entry.get())
            delay_ms = float(self.wp_delay_entry.get())
        except ValueError:
            self.log_message("✗ 제어 파라미터 오류")
            self.path_executing = False
            return
        
        # 프로그레스바 초기화
        self.wp_progress['value'] = 0
        self.wp_progress['maximum'] = len(self.angle_trajectory)
        
        self.log_message("="*50)
        self.log_message("▶️ 웨이포인트 경로 실행 시작...")
        self.log_message(f"총 포인트: {len(self.angle_trajectory)}개")
        self.log_message(f"속도: {speed} deg/s, 가속도: {accel} deg/s²")
        self.log_message(f"딜레이: {delay_ms} ms")
        
        start_time = time.time()
        
        for i, angles in enumerate(self.angle_trajectory):
            if not self.path_executing:
                self.log_message("⚠ 실행 중단됨")
                break
            
            # ROS로 전송
            self.angle_speed_pub.publish(Float32MultiArray(data=list(angles) + [speed, accel]))
            
            # 현재 각도 업데이트
            self.current_angles = np.radians(angles)
            
            # 프로그레스바 업데이트
            self.wp_progress['value'] = i + 1
            
            # 진행 상황 업데이트
            progress = (i+1) / len(self.angle_trajectory) * 100
            elapsed = time.time() - start_time
            remaining = (elapsed / (i+1)) * (len(self.angle_trajectory) - i - 1)
            
            self.wp_status_label.config(
                text=f"상태: 실행 중... ({i+1}/{len(self.angle_trajectory)}, {progress:.0f}%, 남은시간: {remaining:.1f}초)", 
                foreground="blue"
            )
            
            if i % 10 == 0 or i == len(self.angle_trajectory) - 1:
                self.log_message(f"진행: {i+1}/{len(self.angle_trajectory)} ({progress:.1f}%)")
            
            # 딜레이
            time.sleep(delay_ms / 1000.0)
        
        self.path_executing = False
        total_time = time.time() - start_time
        self.log_message(f"✅ 웨이포인트 경로 실행 완료! (총 {total_time:.1f}초)")
        self.wp_status_label.config(text="상태: 실행 완료", foreground="green")
        self.wp_progress['value'] = len(self.angle_trajectory)
    
    def stop_waypoint_execution(self):
        """웨이포인트 경로 실행 정지"""
        if self.path_executing:
            self.path_executing = False
            self.log_message("⏹️ 실행 정지 요청됨...")
            self.wp_status_label.config(text="상태: 정지됨", foreground="red")
        else:
            self.log_message("실행 중이 아닙니다.")
    
    def save_waypoints(self):
        """💾 웨이포인트 경로 저장"""
        if len(self.waypoints) == 0:
            messagebox.showwarning("경고", "저장할 경로점이 없습니다.")
            return
        
        # 기본 파일명 생성
        default_filename = f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        # 파일 저장 대화상자
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialfile=default_filename,
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        # 저장할 데이터 구성
        data = {
            "version": "2.0",
            "timestamp": datetime.now().isoformat(),
            "waypoints": self.waypoints,
            "parameters": {
                "interpolation_step": self.wp_interpolation_step.get(),
                "speed": self.wp_speed_entry.get(),
                "accel": self.wp_accel_entry.get(),
                "delay": self.wp_delay_entry.get(),
                "path_type": self.path_type_var.get()
            },
            "current_angles": self.current_angles.tolist()
        }
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            self.log_message(f"✅ 경로 저장 완료: {os.path.basename(filename)}")
            self.log_message(f"   - 경로점: {len(self.waypoints)}개")
            messagebox.showinfo("저장 완료", f"경로가 저장되었습니다:\n{filename}")
        except Exception as e:
            self.log_message(f"❌ 경로 저장 실패: {str(e)}")
            messagebox.showerror("저장 실패", f"경로 저장 중 오류 발생:\n{str(e)}")
    
    def load_waypoints(self):
        """📂 웨이포인트 경로 불러오기"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 버전 확인
            version = data.get("version", "1.0")
            
            # 경로점 불러오기
            loaded_waypoints = data.get("waypoints", [])
            if not loaded_waypoints:
                messagebox.showwarning("경고", "경로점이 없는 파일입니다.")
                return
            
            # 기존 경로점 삭제 확인
            if self.waypoints:
                if not messagebox.askyesno("확인", "기존 경로점을 삭제하고 불러오시겠습니까?"):
                    return
            
            # 경로점 적용
            self.waypoints.clear()
            for wp in loaded_waypoints:
                # 튜플로 변환
                if isinstance(wp, list):
                    self.waypoints.append(tuple(wp))
                else:
                    self.waypoints.append(wp)
            
            self.refresh_waypoint_list()
            
            # 파라미터 불러오기 (있으면)
            if "parameters" in data:
                params = data["parameters"]
                
                self.wp_interpolation_step.delete(0, tk.END)
                self.wp_interpolation_step.insert(0, params.get("interpolation_step", "0.01"))
                
                self.wp_speed_entry.delete(0, tk.END)
                self.wp_speed_entry.insert(0, params.get("speed", "50"))
                
                self.wp_accel_entry.delete(0, tk.END)
                self.wp_accel_entry.insert(0, params.get("accel", "30"))
                
                self.wp_delay_entry.delete(0, tk.END)
                self.wp_delay_entry.insert(0, params.get("delay", "50"))
                
                if "path_type" in params:
                    self.path_type_var.set(params["path_type"])
            
            self.log_message(f"✅ 경로 불러오기 완료: {os.path.basename(filename)}")
            self.log_message(f"   - 경로점: {len(self.waypoints)}개")
            self.log_message(f"   - 버전: {version}")
            messagebox.showinfo("불러오기 완료", f"경로가 불러와졌습니다:\n{len(self.waypoints)}개 경로점")
            
        except json.JSONDecodeError:
            self.log_message("❌ 경로 불러오기 실패: 잘못된 JSON 형식")
            messagebox.showerror("불러오기 실패", "잘못된 파일 형식입니다.")
        except Exception as e:
            self.log_message(f"❌ 경로 불러오기 실패: {str(e)}")
            messagebox.showerror("불러오기 실패", f"경로 불러오기 중 오류 발생:\n{str(e)}")
    
    def interpolate_circular(self, points):
        """원호 보간"""
        if len(points) < 3:
            return self.interpolate_linear(points)
        
        interpolated = []
        
        for i in range(len(points) - 2):
            p0 = np.array(points[i][:3])
            p1 = np.array(points[i+1][:3])
            p2 = np.array(points[i+2][:3])
            
            # 3점을 지나는 원호 생성 (단순화된 버전)
            # 중간점을 통한 2차 베지어 곡선으로 근사
            steps = 10
            for t in np.linspace(0, 1, steps):
                # Quadratic Bezier
                pos = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
                
                # 자세는 선형 보간
                orient0 = np.array(points[i][3:])
                orient2 = np.array(points[i+2][3:])
                orient = (1-t) * orient0 + t * orient2
                
                interpolated.append(np.concatenate([pos, orient]))
        
        return interpolated
    
    def interpolate_linear(self, points):
        """직선 보간"""
        try:
            interp_step = float(self.wp_interpolation_step.get())
        except ValueError:
            interp_step = 0.01
        
        interpolated = []
        
        for i in range(len(points) - 1):
            start = np.array(points[i])
            end = np.array(points[i+1])
            
            distance = np.linalg.norm(end[:3] - start[:3])
            num_steps = max(int(distance / interp_step), 1)
            
            for j in range(num_steps + 1):
                t = j / num_steps
                point = (1 - t) * start + t * end
                interpolated.append(point)
        
        return interpolated
    
    def interpolate_bezier(self, points):
        """베지어 곡선 보간"""
        if len(points) < 3:
            return self.interpolate_linear(points)
        
        try:
            interp_step = float(self.wp_interpolation_step.get())
        except ValueError:
            interp_step = 0.01
        
        interpolated = []
        
        # 3차 베지어 곡선 (Cubic Bezier)
        for i in range(len(points) - 1):
            p0 = np.array(points[i])
            p3 = np.array(points[i+1])
            
            # 제어점 생성 (1/3, 2/3 지점)
            p1 = p0 + (p3 - p0) / 3
            p2 = p0 + (p3 - p0) * 2 / 3
            
            # 거리 기반 세그먼트 수 계산
            distance = np.linalg.norm(p3[:3] - p0[:3])
            num_steps = max(int(distance / interp_step), 1)
            
            for j in range(num_steps + 1):
                t = j / num_steps
                # Cubic Bezier formula
                point = (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3
                interpolated.append(point)
        
        return interpolated
    
    def update_path_statistics(self):
        """경로 통계 정보 업데이트"""
        if not hasattr(self, 'stats_text'):
            return
        
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        
        stats = []
        stats.append(f"📍 경로점 개수: {len(self.waypoints)}개")
        stats.append(f"📐 보간점 개수: {len(self.interpolated_points)}개")
        stats.append(f"🎯 각도 궤적: {len(self.angle_trajectory)}개")
        
        if len(self.waypoints) >= 2:
            # 총 경로 길이 계산
            total_distance = 0
            for i in range(len(self.waypoints) - 1):
                p1 = np.array(self.waypoints[i][:3])
                p2 = np.array(self.waypoints[i+1][:3])
                total_distance += np.linalg.norm(p2 - p1)
            stats.append(f"📏 총 경로 길이: {total_distance:.3f} m")
            
            # 예상 실행 시간
            if self.angle_trajectory:
                try:
                    delay_ms = float(self.wp_delay_entry.get())
                    estimated_time = len(self.angle_trajectory) * delay_ms / 1000.0
                    stats.append(f"⏱️ 예상 시간: {estimated_time:.1f}초")
                except:
                    pass
        
        self.stats_text.insert(tk.END, "\n".join(stats))
        self.stats_text.config(state=tk.DISABLED)
    
    def visualize_3d_path(self):
        """🎨 3D 경로 시각화"""
        if len(self.waypoints) == 0:
            messagebox.showwarning("경고", "시각화할 경로점이 없습니다.")
            return
        
        self.log_message("="*50)
        self.log_message("🎨 3D 경로 시각화 생성 중...")
        
        # 새 창 생성
        viz_window = tk.Toplevel(self.root)
        viz_window.title("🎨 3D 경로 시각화")
        viz_window.geometry("900x700")
        
        # Figure 생성
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 현재 위치
        current_pos, _, _, current_joint_positions = self.robot.forward_kinematics(self.current_angles)
        
        # 🤖 1. 현재 로봇 구조 표시 (파란색 링크)
        self.robot.plot_robot(self.current_angles, ax, show_frames=True, alpha=0.7)
        
        # 경로점 추출
        waypoint_positions = []
        for wp in self.waypoints:
            waypoint_positions.append([wp[0], wp[1], wp[2]])
        waypoint_positions = np.array(waypoint_positions)
        
        # 2. 경로점 표시 (빨간 점)
        ax.scatter(waypoint_positions[:, 0], 
                  waypoint_positions[:, 1], 
                  waypoint_positions[:, 2],
                  c='red', marker='o', s=100, label='경로점',
                  edgecolors='darkred', linewidths=1.5)
        
        # 경로점 번호 표시
        for i, wp in enumerate(waypoint_positions):
            ax.text(wp[0], wp[1], wp[2], f'  P{i+1}', 
                   fontsize=10, color='darkred', weight='bold')
        
        # 3. 경로점 연결선 (파란 선)
        all_points = np.vstack([[current_pos[0], current_pos[1], current_pos[2]], 
                                waypoint_positions])
        ax.plot(all_points[:, 0], all_points[:, 1], all_points[:, 2],
               'b-', linewidth=2, alpha=0.6, label='경로점 연결')
        
        # 4. 보간점 표시 (있으면)
        if len(self.interpolated_points) > 0:
            interp_positions = np.array([p[:3] for p in self.interpolated_points])
            ax.plot(interp_positions[:, 0], 
                   interp_positions[:, 1], 
                   interp_positions[:, 2],
                   'c-', linewidth=1, alpha=0.8, label=f'보간 경로 ({len(self.interpolated_points)}개)')
            
            # 보간점 표시 (작은 점, 10개마다)
            step = max(len(self.interpolated_points) // 20, 1)
            sample_points = interp_positions[::step]
            ax.scatter(sample_points[:, 0], 
                      sample_points[:, 1], 
                      sample_points[:, 2],
                      c='cyan', marker='.', s=20, alpha=0.5)
            
            # 🤖 경로 상의 로봇 자세 표시 (10개 샘플)
            if len(self.angle_trajectory) > 0:
                step = max(len(self.angle_trajectory) // 10, 1)
                for i in range(0, len(self.angle_trajectory), step):
                    angles_rad = np.radians(self.angle_trajectory[i])
                    self.robot.plot_robot(angles_rad, ax, show_frames=False, alpha=0.2)
        
        # 5. 작업 공간 표시 (반투명 박스)
        # 로봇의 대략적인 작업 공간
        workspace_limits = {
            'x': [0.1, 0.6],
            'y': [-0.4, 0.4],
            'z': [0.0, 0.8]
        }
        
        # 작업 공간 경계 박스 그리기
        from itertools import product
        corners = list(product([workspace_limits['x'][0], workspace_limits['x'][1]],
                              [workspace_limits['y'][0], workspace_limits['y'][1]],
                              [workspace_limits['z'][0], workspace_limits['z'][1]]))
        
        # 박스의 12개 엣지 그리기
        edges = [
            [corners[0], corners[1]], [corners[2], corners[3]],
            [corners[4], corners[5]], [corners[6], corners[7]],
            [corners[0], corners[2]], [corners[1], corners[3]],
            [corners[4], corners[6]], [corners[5], corners[7]],
            [corners[0], corners[4]], [corners[1], corners[5]],
            [corners[2], corners[6]], [corners[3], corners[7]]
        ]
        
        for edge in edges:
            points = np.array(edge)
            ax.plot(points[:, 0], points[:, 1], points[:, 2],
                   'gray', linestyle='--', linewidth=0.5, alpha=0.3)
        
        # 6. 축 설정
        ax.set_xlabel('X (m)', fontsize=10, weight='bold')
        ax.set_ylabel('Y (m)', fontsize=10, weight='bold')
        ax.set_zlabel('Z (m)', fontsize=10, weight='bold')
        
        # 7. 제목 및 범례
        path_type = self.path_type_var.get()
        title = f'🤖 로봇 경로 시뮬레이션 - {path_type.upper()} 보간\n'
        title += f'경로점: {len(self.waypoints)}개'
        if len(self.interpolated_points) > 0:
            title += f' | 보간점: {len(self.interpolated_points)}개'
        if len(self.angle_trajectory) > 0:
            title += f' | 각도 궤적: {len(self.angle_trajectory)}개'
        
        ax.set_title(title, fontsize=12, weight='bold', pad=20)
        ax.legend(loc='upper left', fontsize=8)
        
        # 8. 그리드 및 배경
        ax.grid(True, alpha=0.3)
        ax.set_facecolor('#f0f0f0')
        
        # 9. 축 범위 자동 조정 (약간 여유 있게)
        if len(all_points) > 0:
            margin = 0.1
            ax.set_xlim(all_points[:, 0].min() - margin, all_points[:, 0].max() + margin)
            ax.set_ylim(all_points[:, 1].min() - margin, all_points[:, 1].max() + margin)
            ax.set_zlim(max(0, all_points[:, 2].min() - margin), all_points[:, 2].max() + margin)
        
        # 10. 동일한 스케일 (선택적)
        # ax.set_box_aspect([1,1,1])  # 정육면체 비율
        
        # Canvas에 Figure 추가
        canvas = FigureCanvasTkAgg(fig, master=viz_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 툴바 추가 (확대, 회전 등)
        from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
        toolbar = NavigationToolbar2Tk(canvas, viz_window)
        toolbar.update()
        
        # 정보 패널
        info_frame = ttk.Frame(viz_window)
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 범례 설명
        legend_items = []
        legend_items.append("🤖 파란 링크: 현재 로봇 자세")
        legend_items.append("🔴 빨간 다이아몬드: 엔드 이펙터")
        legend_items.append("📍 빨간 원: 경로점")
        if len(self.interpolated_points) > 0:
            legend_items.append("🔷 청록 선: 보간 경로")
        if len(self.angle_trajectory) > 0:
            legend_items.append("👻 반투명 링크: 경로 상 로봇 자세")
        
        info_text = " | ".join(legend_items)
        ttk.Label(info_frame, text=info_text, foreground="darkblue", font=('Arial', 8)).pack()
        
        info_text2 = "💡 팁: 마우스 드래그로 회전, 휠로 확대/축소, 화살표는 좌표계 (빨강=X, 초록=Y, 파랑=Z)"
        ttk.Label(info_frame, text=info_text2, foreground="blue", font=('Arial', 8)).pack()
        
        # 통계 정보
        stats_frame = ttk.Frame(viz_window)
        stats_frame.pack(fill=tk.X, padx=10, pady=5)
        
        stats = []
        stats.append(f"📍 경로점: {len(self.waypoints)}개")
        if len(self.interpolated_points) > 0:
            stats.append(f"📐 보간점: {len(self.interpolated_points)}개")
        if len(self.angle_trajectory) > 0:
            stats.append(f"🎯 각도 궤적: {len(self.angle_trajectory)}개")
        
        # 총 경로 길이
        if len(self.waypoints) >= 1:
            total_distance = np.linalg.norm(current_pos - waypoint_positions[0])
            for i in range(len(waypoint_positions) - 1):
                total_distance += np.linalg.norm(waypoint_positions[i+1] - waypoint_positions[i])
            stats.append(f"📏 총 경로 길이: {total_distance:.3f} m")
        
        stats_text = " | ".join(stats)
        ttk.Label(stats_frame, text=stats_text, font=('Arial', 9)).pack()
        
        # 버튼 프레임
        button_frame = ttk.Frame(viz_window)
        button_frame.pack(fill=tk.X, padx=10, pady=5)
        
        def save_plot():
            """플롯 이미지 저장"""
            filename = filedialog.asksaveasfilename(
                defaultextension=".png",
                initialfile=f"path_visualization_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
                filetypes=[("PNG files", "*.png"), ("PDF files", "*.pdf"), ("All files", "*.*")]
            )
            if filename:
                fig.savefig(filename, dpi=300, bbox_inches='tight')
                self.log_message(f"✅ 시각화 이미지 저장: {os.path.basename(filename)}")
                messagebox.showinfo("저장 완료", f"이미지가 저장되었습니다:\n{filename}")
        
        def reset_view():
            """뷰 리셋"""
            ax.view_init(elev=20, azim=45)
            canvas.draw()
        
        def toggle_path_robots():
            """경로 상 로봇 표시 토글"""
            ax.clear()
            
            # 다시 그리기 (경로 상 로봇 제외)
            self.robot.plot_robot(self.current_angles, ax, show_frames=True, alpha=0.7)
            
            # 경로점
            ax.scatter(waypoint_positions[:, 0], waypoint_positions[:, 1], waypoint_positions[:, 2],
                      c='red', marker='o', s=100, label='경로점', edgecolors='darkred', linewidths=1.5)
            for i, wp in enumerate(waypoint_positions):
                ax.text(wp[0], wp[1], wp[2], f'  P{i+1}', fontsize=10, color='darkred', weight='bold')
            
            # 경로점 연결선
            all_pts = np.vstack([[current_pos[0], current_pos[1], current_pos[2]], waypoint_positions])
            ax.plot(all_pts[:, 0], all_pts[:, 1], all_pts[:, 2], 'b-', linewidth=2, alpha=0.6, label='경로점 연결')
            
            # 보간 경로
            if len(self.interpolated_points) > 0:
                interp_pos = np.array([p[:3] for p in self.interpolated_points])
                ax.plot(interp_pos[:, 0], interp_pos[:, 1], interp_pos[:, 2],
                       'c-', linewidth=1, alpha=0.8, label=f'보간 경로')
            
            # 작업 공간
            for edge in edges:
                points = np.array(edge)
                ax.plot(points[:, 0], points[:, 1], points[:, 2], 'gray', linestyle='--', linewidth=0.5, alpha=0.3)
            
            ax.set_xlabel('X (m)', fontsize=10, weight='bold')
            ax.set_ylabel('Y (m)', fontsize=10, weight='bold')
            ax.set_zlabel('Z (m)', fontsize=10, weight='bold')
            ax.set_title(title, fontsize=12, weight='bold', pad=20)
            ax.legend(loc='upper left', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_facecolor('#f0f0f0')
            
            if len(all_points) > 0:
                margin = 0.1
                ax.set_xlim(all_points[:, 0].min() - margin, all_points[:, 0].max() + margin)
                ax.set_ylim(all_points[:, 1].min() - margin, all_points[:, 1].max() + margin)
                ax.set_zlim(max(0, all_points[:, 2].min() - margin), all_points[:, 2].max() + margin)
            
            canvas.draw()
            messagebox.showinfo("표시 변경", "경로 상 로봇 표시가 제거되었습니다.\n다시 클릭하면 원래대로 돌아갑니다.")
        
        ttk.Button(button_frame, text="💾 이미지 저장", command=save_plot).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🔄 뷰 리셋", command=reset_view).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🤖 경로 로봇 숨김", command=toggle_path_robots).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="❌ 닫기", command=viz_window.destroy).pack(side=tk.RIGHT, padx=5)
        
        # 로그 메시지
        self.log_message("✅ 3D 시각화 생성 완료")
        self.log_message(f"   - 경로점: {len(self.waypoints)}개")
        if len(self.interpolated_points) > 0:
            self.log_message(f"   - 보간점: {len(self.interpolated_points)}개")
        self.log_message("="*50)
    
    def show_trajectory_data(self):
        """📋 보간점 및 각도 데이터 확인 창"""
        if len(self.waypoints) == 0:
            messagebox.showwarning("경고", "확인할 경로점이 없습니다.")
            return
        
        self.log_message("="*50)
        self.log_message("📋 경로 데이터 뷰어 열기...")
        
        # 새 창 생성
        data_window = tk.Toplevel(self.root)
        data_window.title("📋 경로 데이터 뷰어")
        data_window.geometry("1100x700")
        
        # 탭 노트북 생성
        notebook = ttk.Notebook(data_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 탭 1: 경로점 (Waypoints)
        waypoint_tab = ttk.Frame(notebook)
        notebook.add(waypoint_tab, text=f"📍 경로점 ({len(self.waypoints)}개)")
        
        # 경로점 테이블
        wp_columns = ('번호', 'X (m)', 'Y (m)', 'Z (m)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)')
        wp_tree = ttk.Treeview(waypoint_tab, columns=wp_columns, show='headings', height=20)
        
        for col in wp_columns:
            wp_tree.heading(col, text=col)
            if col == '번호':
                wp_tree.column(col, width=60, anchor='center')
            else:
                wp_tree.column(col, width=100, anchor='center')
        
        # 스크롤바
        wp_scrollbar = ttk.Scrollbar(waypoint_tab, orient=tk.VERTICAL, command=wp_tree.yview)
        wp_tree.configure(yscrollcommand=wp_scrollbar.set)
        
        wp_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        wp_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 데이터 추가
        for i, wp in enumerate(self.waypoints):
            wp_tree.insert('', 'end', values=(
                f'P{i+1}',
                f'{wp[0]:.4f}',
                f'{wp[1]:.4f}',
                f'{wp[2]:.4f}',
                f'{wp[3]:.2f}',
                f'{wp[4]:.2f}',
                f'{wp[5]:.2f}'
            ))
        
        # 탭 2: 보간점 (Interpolated Points)
        if len(self.interpolated_points) > 0:
            interp_tab = ttk.Frame(notebook)
            notebook.add(interp_tab, text=f"📐 보간점 ({len(self.interpolated_points)}개)")
            
            # 보간점 테이블
            interp_columns = ('번호', 'X (m)', 'Y (m)', 'Z (m)', 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)')
            interp_tree = ttk.Treeview(interp_tab, columns=interp_columns, show='headings', height=20)
            
            for col in interp_columns:
                interp_tree.heading(col, text=col)
                if col == '번호':
                    interp_tree.column(col, width=60, anchor='center')
                else:
                    interp_tree.column(col, width=120, anchor='center')
            
            # 스크롤바
            interp_scrollbar = ttk.Scrollbar(interp_tab, orient=tk.VERTICAL, command=interp_tree.yview)
            interp_tree.configure(yscrollcommand=interp_scrollbar.set)
            
            interp_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            interp_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            
            # 데이터 추가
            for i, point in enumerate(self.interpolated_points):
                interp_tree.insert('', 'end', values=(
                    i+1,
                    f'{point[0]:.6f}',
                    f'{point[1]:.6f}',
                    f'{point[2]:.6f}',
                    f'{point[3]:.6f}',
                    f'{point[4]:.6f}',
                    f'{point[5]:.6f}'
                ))
        
        # 탭 3: 각도 궤적 (Angle Trajectory)
        if len(self.angle_trajectory) > 0:
            angle_tab = ttk.Frame(notebook)
            notebook.add(angle_tab, text=f"🎯 각도 궤적 ({len(self.angle_trajectory)}개)")
            
            # 각도 테이블
            angle_columns = ('번호', '서보1 (°)', '서보2 (°)', '서보3 (°)', '서보4 (°)', '서보5 (°)', '서보6 (°)', 'IK 상태')
            angle_tree = ttk.Treeview(angle_tab, columns=angle_columns, show='headings', height=20)
            
            for col in angle_columns:
                angle_tree.heading(col, text=col)
                if col == '번호':
                    angle_tree.column(col, width=60, anchor='center')
                elif col == 'IK 상태':
                    angle_tree.column(col, width=80, anchor='center')
                else:
                    angle_tree.column(col, width=100, anchor='center')
            
            # 스크롤바
            angle_scrollbar = ttk.Scrollbar(angle_tab, orient=tk.VERTICAL, command=angle_tree.yview)
            angle_tree.configure(yscrollcommand=angle_scrollbar.set)
            
            angle_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            angle_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            
            # 데이터 추가
            for i, angles in enumerate(self.angle_trajectory):
                # 각도 범위 체크
                all_valid = all(-135 <= a <= 135 for a in angles)
                status = '✅' if all_valid else '⚠️'
                
                angle_tree.insert('', 'end', values=(
                    i+1,
                    f'{angles[0]:.2f}',
                    f'{angles[1]:.2f}',
                    f'{angles[2]:.2f}',
                    f'{angles[3]:.2f}',
                    f'{angles[4]:.2f}',
                    f'{angles[5]:.2f}',
                    status
                ))
        
        # 하단 정보 및 버튼 프레임
        bottom_frame = ttk.Frame(data_window)
        bottom_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 통계 정보
        stats_text = f"📊 총계: 경로점 {len(self.waypoints)}개 | "
        if len(self.interpolated_points) > 0:
            stats_text += f"보간점 {len(self.interpolated_points)}개 | "
        if len(self.angle_trajectory) > 0:
            stats_text += f"각도 궤적 {len(self.angle_trajectory)}개"
        
        ttk.Label(bottom_frame, text=stats_text, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)
        
        # 버튼들
        button_frame = ttk.Frame(data_window)
        button_frame.pack(fill=tk.X, padx=10, pady=5)
        
        def export_to_csv():
            """CSV로 내보내기"""
            if len(self.angle_trajectory) == 0:
                messagebox.showwarning("경고", "내보낼 데이터가 없습니다.")
                return
            
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                initialfile=f"trajectory_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )
            
            if not filename:
                return
            
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    # 헤더
                    f.write("번호,서보1,서보2,서보3,서보4,서보5,서보6,X,Y,Z,Roll,Pitch,Yaw\n")
                    
                    # 데이터
                    for i, angles in enumerate(self.angle_trajectory):
                        f.write(f"{i+1}")
                        for a in angles:
                            f.write(f",{a:.4f}")
                        
                        # 보간점 좌표도 함께 저장 (있으면)
                        if i < len(self.interpolated_points):
                            point = self.interpolated_points[i]
                            for p in point:
                                f.write(f",{p:.6f}")
                        else:
                            f.write(",,,,,,")
                        
                        f.write("\n")
                
                self.log_message(f"✅ CSV 내보내기 완료: {os.path.basename(filename)}")
                messagebox.showinfo("내보내기 완료", f"데이터가 내보내졌습니다:\n{filename}")
            except Exception as e:
                self.log_message(f"❌ CSV 내보내기 실패: {str(e)}")
                messagebox.showerror("내보내기 실패", f"오류 발생:\n{str(e)}")
        
        def copy_to_clipboard():
            """클립보드에 복사"""
            if len(self.angle_trajectory) == 0:
                messagebox.showwarning("경고", "복사할 데이터가 없습니다.")
                return
            
            # 현재 선택된 탭 확인
            current_tab = notebook.index(notebook.select())
            
            if current_tab == 0:  # 경로점
                data_text = "번호\tX\tY\tZ\tRoll\tPitch\tYaw\n"
                for i, wp in enumerate(self.waypoints):
                    data_text += f"P{i+1}\t{wp[0]:.4f}\t{wp[1]:.4f}\t{wp[2]:.4f}\t{wp[3]:.2f}\t{wp[4]:.2f}\t{wp[5]:.2f}\n"
            elif current_tab == 1 and len(self.interpolated_points) > 0:  # 보간점
                data_text = "번호\tX\tY\tZ\tRoll\tPitch\tYaw\n"
                for i, point in enumerate(self.interpolated_points):
                    data_text += f"{i+1}\t{point[0]:.6f}\t{point[1]:.6f}\t{point[2]:.6f}\t{point[3]:.6f}\t{point[4]:.6f}\t{point[5]:.6f}\n"
            elif len(self.angle_trajectory) > 0:  # 각도 궤적
                data_text = "번호\t서보1\t서보2\t서보3\t서보4\t서보5\t서보6\n"
                for i, angles in enumerate(self.angle_trajectory):
                    data_text += f"{i+1}\t{angles[0]:.2f}\t{angles[1]:.2f}\t{angles[2]:.2f}\t{angles[3]:.2f}\t{angles[4]:.2f}\t{angles[5]:.2f}\n"
            else:
                messagebox.showwarning("경고", "복사할 데이터가 없습니다.")
                return
            
            data_window.clipboard_clear()
            data_window.clipboard_append(data_text)
            messagebox.showinfo("복사 완료", "데이터가 클립보드에 복사되었습니다.\nExcel이나 텍스트 에디터에 붙여넣기 가능합니다.")
            self.log_message("✅ 클립보드에 복사됨")
        
        def show_statistics():
            """상세 통계 표시"""
            if len(self.angle_trajectory) == 0:
                messagebox.showinfo("통계", "각도 궤적이 생성되지 않았습니다.")
                return
            
            # 통계 계산
            angles_array = np.array(self.angle_trajectory)
            
            stats_msg = "📊 각도 궤적 통계\n\n"
            
            for servo_idx in range(6):
                servo_angles = angles_array[:, servo_idx]
                stats_msg += f"서보 {servo_idx+1}:\n"
                stats_msg += f"  최소: {servo_angles.min():.2f}°\n"
                stats_msg += f"  최대: {servo_angles.max():.2f}°\n"
                stats_msg += f"  평균: {servo_angles.mean():.2f}°\n"
                stats_msg += f"  범위: {servo_angles.max() - servo_angles.min():.2f}°\n"
                
                # 범위 초과 체크
                out_of_range = np.sum((servo_angles < -135) | (servo_angles > 135))
                if out_of_range > 0:
                    stats_msg += f"  ⚠️ 범위 초과: {out_of_range}개 포인트\n"
                
                stats_msg += "\n"
            
            # 총 이동량 계산
            total_movement = 0
            for servo_idx in range(6):
                servo_angles = angles_array[:, servo_idx]
                movement = np.sum(np.abs(np.diff(servo_angles)))
                total_movement += movement
            
            stats_msg += f"총 서보 이동량: {total_movement:.2f}°\n"
            stats_msg += f"평균 포인트당 이동: {total_movement/len(self.angle_trajectory):.2f}°\n"
            
            messagebox.showinfo("상세 통계", stats_msg)
        
        ttk.Button(button_frame, text="📋 클립보드 복사", command=copy_to_clipboard).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="💾 CSV 내보내기", command=export_to_csv).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="📊 상세 통계", command=show_statistics).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="❌ 닫기", command=data_window.destroy).pack(side=tk.RIGHT, padx=5)
        
        # 도움말
        help_frame = ttk.Frame(data_window)
        help_frame.pack(fill=tk.X, padx=10, pady=5)
        
        help_text = "💡 팁: 탭을 전환하여 경로점, 보간점, 각도 데이터를 확인할 수 있습니다. 클립보드 복사 후 Excel에 붙여넣기 가능합니다."
        ttk.Label(help_frame, text=help_text, foreground="blue", font=('Arial', 9)).pack()
        
        self.log_message("✅ 경로 데이터 뷰어 열림")
        self.log_message(f"   - 경로점: {len(self.waypoints)}개")
        if len(self.interpolated_points) > 0:
            self.log_message(f"   - 보간점: {len(self.interpolated_points)}개")
        if len(self.angle_trajectory) > 0:
            self.log_message(f"   - 각도 궤적: {len(self.angle_trajectory)}개")
        self.log_message("="*50)
    
    def visualize_3d_path_embedded(self):
        """🎨 3D 경로 시각화 (GUI 내장형 - 선택사항)"""
        # 이 메서드는 GUI 내에 직접 임베드하는 버전입니다.
        # 필요시 사용 가능
        pass
    
    def ros_spin(self):
        """ROS2 스핀 (별도 스레드에서 실행)"""
        rclpy.spin_once(self, timeout_sec=0.01)
    
    def run(self):
        """GUI 실행"""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        gui = RobotControlGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
