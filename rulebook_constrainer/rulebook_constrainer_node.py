#!/usr/bin/env python3

import os
import json
import yaml
import math
import threading
from typing import Dict, Any, Optional, List, Tuple
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS2 메시지 및 서비스
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import PointStamped

# 로컬 모듈
from .utils.mask_generator import MaskGenerator
from .utils.geometry_utils import GeometryUtils

class RulebookConstrainerNode(Node):
    """규칙서 기반 제약 마스크 생성 노드"""
    
    def __init__(self):
        super().__init__('rulebook_constrainer_node')
        
        # 콜백 그룹 설정
        self.callback_group = ReentrantCallbackGroup()
        
        # 파라미터 선언 및 로드
        self._declare_parameters()
        self._load_parameters()
        
        # 내부 상태
        self.rulebook_data = {}
        self.landmarks = {}
        self.map_metadata = None
        self.lock = threading.RLock()
        
        # 유틸리티 클래스
        self.mask_generator = MaskGenerator(
            resolution=self.mask_resolution,
            robot_radius=self.robot_footprint_radius
        )
        self.geometry_utils = GeometryUtils()
        
        # TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 퍼블리셔 생성
        self._create_publishers()
        
        # 서비스 생성
        self._create_services()
        
        # 구독자 생성 (지도 정보 획득용)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 초기 규칙서 로딩
        self._load_rulebook()
        
        # 주기적 퍼블리시 타이머
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_masks,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Rulebook Constrainer Node initialized")
    
    def _declare_parameters(self):
        """ROS2 파라미터 선언"""
        # 기본 설정
        self.declare_parameter('rulebook_yaml', '')
        self.declare_parameter('mask_resolution', 0.05)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('robot_footprint_radius', 0.22)
        
        # 필터 설정
        self.declare_parameter('filters.keepout', True)
        self.declare_parameter('filters.speed', True) 
        self.declare_parameter('filters.lane_preference', 'right')
        
        # 마스크 생성 설정
        self.declare_parameter('mask_params.keepout_value', 254)
        self.declare_parameter('mask_params.free_space_value', 0)
        self.declare_parameter('mask_params.unknown_value', 255)
        self.declare_parameter('mask_params.speed_scale_factor', 100)
        
        # 시각화 설정
        self.declare_parameter('visualization.enabled', True)
        self.declare_parameter('visualization.marker_lifetime', 0.0)
        self.declare_parameter('visualization.keepout_color', [1.0, 0.0, 0.0, 0.3])
        self.declare_parameter('visualization.speed_color', [1.0, 1.0, 0.0, 0.3])
        self.declare_parameter('visualization.lane_color', [0.0, 0.0, 1.0, 0.3])
        
        # 맵 기본값
        self.declare_parameter('map_params.default_map_width', 2048)
        self.declare_parameter('map_params.default_map_height', 2048)
        self.declare_parameter('map_params.default_origin_x', -51.2)
        self.declare_parameter('map_params.default_origin_y', -51.2)
        
        # 에러 처리
        self.declare_parameter('tf_timeout', 5.0)
        self.declare_parameter('max_retry_attempts', 3)
        self.declare_parameter('retry_delay', 1.0)
    
    def _load_parameters(self):
        """파라미터 값 로드"""
        self.rulebook_yaml_path = self.get_parameter('rulebook_yaml').get_parameter_value().string_value
        self.mask_resolution = self.get_parameter('mask_resolution').get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.robot_footprint_radius = self.get_parameter('robot_footprint_radius').get_parameter_value().double_value
        
        # 필터 설정
        self.enable_keepout = self.get_parameter('filters.keepout').get_parameter_value().bool_value
        self.enable_speed = self.get_parameter('filters.speed').get_parameter_value().bool_value
        self.lane_preference = self.get_parameter('filters.lane_preference').get_parameter_value().string_value
        
        # 시각화 설정
        self.visualization_enabled = self.get_parameter('visualization.enabled').get_parameter_value().bool_value
        
        # 맵 기본값
        self.default_width = self.get_parameter('map_params.default_map_width').get_parameter_value().integer_value
        self.default_height = self.get_parameter('map_params.default_map_height').get_parameter_value().integer_value
        self.default_origin_x = self.get_parameter('map_params.default_origin_x').get_parameter_value().double_value
        self.default_origin_y = self.get_parameter('map_params.default_origin_y').get_parameter_value().double_value
        
        # 에러 처리
        self.tf_timeout = self.get_parameter('tf_timeout').get_parameter_value().double_value
        self.max_retry_attempts = self.get_parameter('max_retry_attempts').get_parameter_value().integer_value
        self.retry_delay = self.get_parameter('retry_delay').get_parameter_value().double_value
    
    def _create_publishers(self):
        """퍼블리셔 생성"""
        self.keepout_mask_pub = self.create_publisher(
            OccupancyGrid, '/rule_mask_keepout', 10)
        
        self.speed_mask_pub = self.create_publisher(
            OccupancyGrid, '/rule_mask_speed', 10)
            
        self.lane_mask_pub = self.create_publisher(
            OccupancyGrid, '/rule_mask_lane_preference', 10)
        
        if self.visualization_enabled:
            self.visualization_pub = self.create_publisher(
                MarkerArray, '/rule_visualization', 10)
    
    def _create_services(self):
        """서비스 생성"""
        # 규칙 JSON 반환 서비스
        self.json_service = self.create_service(
            Trigger,
            '/rulebook/json',
            self._handle_json_request,
            callback_group=self.callback_group
        )
        
        # 규칙서 다시 로딩 서비스  
        self.reload_service = self.create_service(
            Trigger,
            '/rulebook/reload',
            self._handle_reload_request,
            callback_group=self.callback_group
        )
    
    def _map_callback(self, msg: OccupancyGrid):
        """맵 정보 업데이트"""
        with self.lock:
            self.map_metadata = msg.info
        
        self.get_logger().info(f"Map metadata updated: {msg.info.width}x{msg.info.height}, "
                             f"resolution: {msg.info.resolution}")
    
    def _load_rulebook(self) -> bool:
        """YAML 규칙서 로드"""
        if not self.rulebook_yaml_path:
            self.get_logger().warn("No rulebook YAML path specified")
            return False
            
        if not os.path.exists(self.rulebook_yaml_path):
            self.get_logger().error(f"Rulebook file not found: {self.rulebook_yaml_path}")
            return False
        
        try:
            with open(self.rulebook_yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            with self.lock:
                self.rulebook_data = data
                self.landmarks = data.get('landmarks', {})
            
            self.get_logger().info(f"Rulebook loaded successfully: {self.rulebook_yaml_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load rulebook: {e}")
            return False
    
    def _publish_masks(self):
        """마스크 퍼블리시"""
        if not self.rulebook_data:
            return
        
        try:
            # 맵 메타데이터 확인
            width, height, origin_x, origin_y, frame_id = self._get_map_info()
            
            rules = self.rulebook_data.get('rules', {})
            landmarks = self.landmarks
            
            # Keepout 마스크
            if self.enable_keepout:
                keepout_mask = self.mask_generator.generate_keepout_mask(
                    rules, landmarks, width, height, origin_x, origin_y, frame_id)
                self.keepout_mask_pub.publish(keepout_mask)
            
            # 속도 마스크
            if self.enable_speed:
                speed_mask = self.mask_generator.generate_speed_mask(
                    rules, landmarks, width, height, origin_x, origin_y, frame_id)
                self.speed_mask_pub.publish(speed_mask)
            
            # 차선 선호도 마스크
            if self.lane_preference != "none":
                lane_mask = self.mask_generator.generate_lane_preference_mask(
                    rules, landmarks, width, height, origin_x, origin_y, 
                    self.lane_preference, frame_id)
                self.lane_mask_pub.publish(lane_mask)
            
            # 시각화 마커
            if self.visualization_enabled:
                markers = self._create_visualization_markers(rules, landmarks, frame_id)
                self.visualization_pub.publish(markers)
                
        except Exception as e:
            self.get_logger().error(f"Failed to publish masks: {e}")
    
    def _get_map_info(self) -> Tuple[int, int, float, float, str]:
        """맵 정보 반환 (폭, 높이, 원점 x, y, 프레임)"""
        with self.lock:
            if self.map_metadata:
                return (
                    self.map_metadata.width,
                    self.map_metadata.height, 
                    self.map_metadata.origin.position.x,
                    self.map_metadata.origin.position.y,
                    "map"
                )
            else:
                # 기본값 사용
                return (
                    self.default_width,
                    self.default_height,
                    self.default_origin_x,
                    self.default_origin_y,
                    "map"
                )
    
    def _create_visualization_markers(self, 
                                    rules: Dict[str, Any], 
                                    landmarks: Dict[str, List[float]],
                                    frame_id: str) -> MarkerArray:
        """시각화 마커 생성"""
        marker_array = MarkerArray()
        marker_id = 0
        
        try:
            # Keepout 영역 마커
            keepout_zones = rules.get('keepout_zones', [])
            for zone in keepout_zones:
                marker = self._create_zone_marker(zone, landmarks, frame_id, marker_id, 
                                                [1.0, 0.0, 0.0, 0.3])  # 빨간색
                if marker:
                    marker_array.markers.append(marker)
                    marker_id += 1
            
            # 속도 제한 영역 마커
            speed_zones = rules.get('speed_zones', [])
            for zone in speed_zones:
                marker = self._create_zone_marker(zone, landmarks, frame_id, marker_id,
                                                [1.0, 1.0, 0.0, 0.3])  # 노란색
                if marker:
                    marker_array.markers.append(marker)
                    marker_id += 1
            
            # 차선 선호도 마커
            lane_preferences = rules.get('lane_preferences', [])
            for lane in lane_preferences:
                marker = self._create_lane_marker(lane, landmarks, frame_id, marker_id,
                                                [0.0, 0.0, 1.0, 0.3])  # 파란색
                if marker:
                    marker_array.markers.append(marker)
                    marker_id += 1
                    
        except Exception as e:
            self.get_logger().
