#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple, Any, Optional
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import rclpy
from .geometry_utils import GeometryUtils

class MaskGenerator:
    """규칙 기반 마스크 생성기"""
    
    def __init__(self, resolution: float = 0.05, robot_radius: float = 0.22):
        """
        Args:
            resolution: 마스크 해상도 (m/pixel)
            robot_radius: 로봇 반경 (m)
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.geometry_utils = GeometryUtils()
        
    def generate_keepout_mask(self, 
                            rules: Dict[str, Any], 
                            landmarks: Dict[str, List[float]],
                            width: int, 
                            height: int,
                            origin_x: float, 
                            origin_y: float,
                            frame_id: str = "map") -> OccupancyGrid:
        """Keepout 영역 마스크 생성"""
        
        # 빈 마스크 초기화 (0 = 자유공간, 254 = keepout)
        mask_data = np.zeros((height, width), dtype=np.int8)
        
        keepout_zones = rules.get('keepout_zones', [])
        
        for zone in keepout_zones:
            try:
                zone_type = zone.get('type', '')
                
                if zone_type == 'polygon':
                    coords = self._resolve_coordinates(zone.get('coordinates', []), landmarks)
                    if len(coords) >= 3:
                        # 로봇 반경만큼 확장
                        expanded_coords = self.geometry_utils.expand_polygon(coords, self.robot_radius)
                        self._fill_polygon_in_mask(mask_data, expanded_coords, origin_x, origin_y, 254)
                        
                elif zone_type == 'circle':
                    center = self._resolve_coordinates([zone.get('center', [0, 0])], landmarks)[0]
                    radius = zone.get('radius', 0.0)
                    
                    # 로봇 반경만큼 확장
                    expanded_center, expanded_radius = self.geometry_utils.expand_circle(
                        center, radius, self.robot_radius)
                    self._fill_circle_in_mask(mask_data, expanded_center, expanded_radius, 
                                            origin_x, origin_y, 254)
                    
            except Exception as e:
                print(f"Warning: Failed to process keepout zone {zone.get('name', 'unknown')}: {e}")
                continue
        
        return self._create_occupancy_grid(mask_data, origin_x, origin_y, frame_id)
    
    def generate_speed_mask(self, 
                          rules: Dict[str, Any],
                          landmarks: Dict[str, List[float]], 
                          width: int,
                          height: int,
                          origin_x: float,
                          origin_y: float,
                          frame_id: str = "map") -> OccupancyGrid:
        """속도 제한 마스크 생성"""
        
        # 기본값으로 초기화 (0 = 속도 제한 없음)
        mask_data = np.zeros((height, width), dtype=np.int8)
        
        speed_zones = rules.get('speed_zones', [])
        
        for zone in speed_zones:
            try:
                zone_type = zone.get('type', '')
                max_speed = zone.get('max_speed', 1.0)  # m/s
                
                # 속도를 0-100 범위로 정규화 (100 = 1.0 m/s)
                speed_value = min(100, max(0, int(max_speed * 100)))
                
                if zone_type == 'polygon':
                    coords = self._resolve_coordinates(zone.get('coordinates', []), landmarks)
                    if len(coords) >= 3:
                        self._fill_polygon_in_mask(mask_data, coords, origin_x, origin_y, speed_value)
                        
                elif zone_type == 'circle':
                    center = self._resolve_coordinates([zone.get('center', [0, 0])], landmarks)[0]
                    radius = zone.get('radius', 0.0)
                    self._fill_circle_in_mask(mask_data, center, radius, origin_x, origin_y, speed_value)
                    
            except Exception as e:
                print(f"Warning: Failed to process speed zone {zone.get('name', 'unknown')}: {e}")
                continue
        
        return self._create_occupancy_grid(mask_data, origin_x, origin_y, frame_id)
    
    def generate_lane_preference_mask(self,
                                    rules: Dict[str, Any],
                                    landmarks: Dict[str, List[float]],
                                    width: int,
                                    height: int, 
                                    origin_x: float,
                                    origin_y: float,
                                    preference: str = "right",
                                    frame_id: str = "map") -> OccupancyGrid:
        """차선 선호도 마스크 생성"""
        
        # 기본값으로 초기화 (50 = 중립)
        mask_data = np.full((height, width), 50, dtype=np.int8)
        
        if preference == "none":
            return self._create_occupancy_grid(mask_data, origin_x, origin_y, frame_id)
        
        lane_preferences = rules.get('lane_preferences', [])
        preference_value = 25 if preference == "right" else 75  # 낮을수록 선호
        
        for lane in lane_preferences:
            try:
                if lane.get('type') == 'line' and lane.get('preference') == preference:
                    start = self._resolve_coordinates([lane.get('start', [0, 0])], landmarks)[0]
                    end = self._resolve_coordinates([lane.get('end', [0, 0])], landmarks)[0]
                    width_m = lane.get('width', 1.0)
                    
                    self._fill_line_corridor_in_mask(mask_data, start, end, width_m, 
                                                   origin_x, origin_y, preference_value)
                    
            except Exception as e:
                print(f"Warning: Failed to process lane preference {lane.get('name', 'unknown')}: {e}")
                continue
        
        return self._create_occupancy_grid(mask_data, origin_x, origin_y, frame_id)
    
    def _resolve_coordinates(self, 
                           coords: List[Any], 
                           landmarks: Dict[str, List[float]]) -> List[Tuple[float, float]]:
        """좌표 또는 랜드마크 참조를 실제 좌표로 변환"""
        resolved = []
        
        for coord in coords:
            if isinstance(coord, list) and len(coord) >= 2:
                # 직접 좌표
                resolved.append((float(coord[0]), float(coord[1])))
            elif isinstance(coord, str) and coord in landmarks:
                # 랜드마크 참조
                landmark_coord = landmarks[coord]
                resolved.append((float(landmark_coord[0]), float(landmark_coord[1])))
            else:
                print(f"Warning: Invalid coordinate or landmark reference: {coord}")
                
        return resolved
    
    def _fill_polygon_in_mask(self,
                            mask: np.ndarray,
                            polygon: List[Tuple[float, float]], 
                            origin_x: float,
                            origin_y: float,
                            value: int):
        """폴리곤 영역을 마스크에 채우기"""
        if len(polygon) < 3:
            return
            
        height, width = mask.shape
        
        # 각 픽셀에 대해 폴리곤 내부 확인
        for y in range(height):
            for x in range(width):
                world_x, world_y = self.geometry_utils.map_to_world_coordinates(
                    x, y, origin_x, origin_y, self.resolution)
                
                if self.geometry_utils.point_in_polygon((world_x, world_y), polygon):
                    mask[y, x] = value
    
    def _fill_circle_in_mask(self,
                           mask: np.ndarray,
                           center: Tuple[float, float],
                           radius: float,
                           origin_x: float,
                           origin_y: float, 
                           value: int):
        """원형 영역을 마스크에 채우기"""
        height, width = mask.shape
        
        # 효율적인 처리를 위해 경계 상자 계산
        min_x, max_x = self.geometry_utils.world_to_map_coordinates(
            center[0] - radius, center[1], origin_x, origin_y, self.resolution)[0], \
                      self.geometry_utils.world_to_map_coordinates(
            center[0] + radius, center[1], origin_x, origin_y, self.resolution)[0]
        min_y, max_y = self.geometry_utils.world_to_map_coordinates(
            center[0], center[1] - radius, origin_x, origin_y, self.resolution)[1], \
                      self.geometry_utils.world_to_map_coordinates(
            center[0], center[1] + radius, origin_x, origin_y, self.resolution)[1]
        
        # 경계 확인
        min_x, max_x = max(0, min_x), min(width - 1, max_x)
        min_y, max_y = max(0, min_y), min(height - 1, max_y)
        
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                world_x, world_y = self.geometry_utils.map_to_world_coordinates(
                    x, y, origin_x, origin_y, self.resolution)
                    
                if self.geometry_utils.point_in_circle((world_x, world_y), center, radius):
                    mask[y, x] = value
    
    def _fill_line_corridor_in_mask(self,
                                  mask: np.ndarray,
                                  start: Tuple[float, float],
                                  end: Tuple[float, float],
                                  corridor_width: float,
                                  origin_x: float,
                                  origin_y: float,
                                  value: int):
        """선형 복도 영역을 마스크에 채우기"""
        height, width = mask.shape
        half_width = corridor_width / 2.0
        
        for y in range(height):
            for x in range(width):
                world_x, world_y = self.geometry_utils.map_to_world_coordinates(
                    x, y, origin_x, origin_y, self.resolution)
                
                distance = self.geometry_utils.distance_to_line((world_x, world_y), start, end)
                if distance <= half_width:
                    mask[y, x] = value
    
    def _create_occupancy_grid(self,
                             mask_data: np.ndarray,
                             origin_x: float,
                             origin_y: float,
                             frame_id: str) -> OccupancyGrid:
        """OccupancyGrid 메시지 생성"""
        height, width = mask_data.shape
        
        grid = OccupancyGrid()
        
        # 헤더 설정
        grid.header = Header()
        grid.header.stamp = rclpy.clock.Clock().now().to_msg()
        grid.header.frame_id = frame_id
        
        # 메타정보 설정
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        
        # 원점 설정
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # 데이터 변환 (행 우선 순서로 평탄화)
        grid.data = mask_data.flatten().tolist()
        
        return grid
