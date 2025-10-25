#!/usr/bin/env python3
"""
6축 로봇 제어 GUI
- 각도/좌표 + 속도/가속도 제어
- ROS2 통신을 통한 ESP32 마스터 컨트롤러와 연동
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
import tkinter as tk
from tkinter import ttk, messagebox
import math
import threading
import time
import numpy as np

class RobotControlGUI(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        
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
        
        self.get_logger().info('로봇 제어 GUI가 시작되었습니다.')
    
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
        
        # 경로 제어 탭 (새로 추가)
        self.setup_path_control_tab()
        
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
        self.path_interval_var = tk.DoubleVar(value=0.05)  # 50ms (브로드캐스트 보간 최적값)
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
        """경로를 순차적으로 실행"""
        self.path_executing = True
        self.path_start_btn.config(state='disabled')
        self.path_stop_btn.config(state='normal')
        
        total_points = len(path)
        mode_name = "고속" if fast_mode else "일반"
        
        self.log_message(f"📍 {mode_name} 모드로 경로 실행 중...")
        
        for i, point in enumerate(path):
            if not self.path_executing:
                self.log_message("경로 실행이 중지되었습니다.")
                break
            
            # 🚀 실행 모드에 따라 다른 토픽 사용
            msg = Float32MultiArray()
            
            if fast_mode:
                # 고속 모드: ACK 없는 빠른 실행
                # 데이터: [x, y, z, roll, pitch, yaw, speed, accel]
                msg.data = point + [speed, accel]
                self.interpolation_fast_pub.publish(msg)
                
                # 고속 모드는 IK 계산 시간만 고려 (ACK 대기 없음)
                wait_time = max(interval, 0.01)  # 최소 10ms (브로드캐스트 보간 최적화)
                
            else:
                # 일반 모드: 동기화 + ACK 처리
                msg.data = point + [speed, accel]
                self.coord_speed_pub.publish(msg)
                
                # 일반 모드는 전체 처리 시간 고려
                ik_processing_time = 0.5
                
                if i > 0:
                    prev_point = path[i-1]
                    max_coord_diff = max(abs(point[j] - prev_point[j]) for j in range(3))
                    estimated_move_time = max_coord_diff / 100.0 if speed > 0 else 1.0
                else:
                    estimated_move_time = 1.0
                
                wait_time = max(interval, ik_processing_time + estimated_move_time + 0.2)
            
            # 상태 업데이트
            progress = (i + 1) / total_points * 100
            self.path_status_label.config(
                text=f"경로 실행 중 ({mode_name}): {i+1}/{total_points} ({progress:.1f}%)", 
                foreground="blue"
            )
            self.log_message(f"  포인트 {i+1}/{total_points}: {[round(p, 2) for p in point[:6]]}")
            
            # 다음 포인트까지 대기
            time.sleep(wait_time)
        
        # 실행 완료
        self.path_executing = False
        self.path_start_btn.config(state='normal')
        self.path_stop_btn.config(state='disabled')
        self.path_status_label.config(text=f"경로 실행 완료! ({mode_name} 모드)", foreground="green")
        self.log_message(f"✅ {mode_name} 모드 경로 실행이 완료되었습니다.")
    
    def start_path_execution(self):
        """경로 실행 시작 - 새로운 방식: 두 점만 마스터에 전송"""
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
        
        # 🎯 새로운 방식: 두 점만 마스터에 전송
        self.send_path_command(start, end, num_points, interval, speed, accel, fast_mode)
        
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
        self.path_command_pub.publish(msg)
    
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
