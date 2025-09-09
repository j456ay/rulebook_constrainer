#!/usr/bin/env python3

import math
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point, Polygon as ROSPolygon
from shapely.geometry import Point as ShapelyPoint, Polygon as ShapelyPolygon, LineString
from shapely.ops import transform
from shapely.prepared import prep

class GeometryUtils:
    """기하학적 계산을 위한 유틸리티 클래스"""
    
    @staticmethod
    def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        """점이 폴리곤 내부에 있는지 확인"""
        try:
            shapely_point = ShapelyPoint(point)
            shapely_polygon = ShapelyPolygon(polygon)
            return shapely_polygon.contains(shapely_point)
        except Exception:
            return False
    
    @staticmethod
    def point_in_circle(point: Tuple[float, float], center: Tuple[float, float], radius: float) -> bool:
        """점이 원 내부에 있는지 확인"""
        try:
            dx = point[0] - center[0]
            dy = point[1] - center[1]
            distance = math.sqrt(dx*dx + dy*dy)
            return distance <= radius
        except Exception:
            return False
    
    @staticmethod
    def distance_to_line(point: Tuple[float, float], line_start: Tuple[float, float], 
                        line_end: Tuple[float, float]) -> float:
        """점에서 선분까지의 최단 거리"""
        try:
            shapely_point = ShapelyPoint(point)
            shapely_line = LineString([line_start, line_end])
            return shapely_point.distance(shapely_line)
        except Exception:
            return float('inf')
    
    @staticmethod
    def expand_polygon(polygon: List[Tuple[float, float]], distance: float) -> List[Tuple[float, float]]:
        """폴리곤을 주어진 거리만큼 확장 (로봇 반경 고려)"""
        try:
            shapely_polygon = ShapelyPolygon(polygon)
            expanded = shapely_polygon.buffer(distance)
            
            if expanded.is_empty:
                return polygon
                
            # MultiPolygon인 경우 가장 큰 폴리곤 선택
            if hasattr(expanded, 'geoms'):
                expanded = max(expanded.geoms, key=lambda p: p.area)
            
            # 외곽선 좌표 반환
            if hasattr(expanded, 'exterior'):
                coords = list(expanded.exterior.coords)
                return [(float(x), float(y)) for x, y in coords[:-1]]  # 마지막 중복 제거
            else:
                return polygon
                
        except Exception as e:
            print(f"Warning: Failed to expand polygon: {e}")
            return polygon
    
    @staticmethod  
    def expand_circle(center: Tuple[float, float], radius: float, expansion: float) -> Tuple[Tuple[float, float], float]:
        """원을 주어진 거리만큼 확장"""
        try:
            new_radius = max(0.0, radius + expansion)
            return center, new_radius
        except Exception:
            return center, radius
    
    @staticmethod
    def world_to_map_coordinates(world_x: float, world_y: float, 
                                origin_x: float, origin_y: float, resolution: float) -> Tuple[int, int]:
        """월드 좌표를 맵 픽셀 좌표로 변환"""
        try:
            map_x = int((world_x - origin_x) / resolution)
            map_y = int((world_y - origin_y) / resolution)
            return map_x, map_y
        except Exception:
            return 0, 0
    
    @staticmethod
    def map_to_world_coordinates(map_x: int, map_y: int,
                                origin_x: float, origin_y: float, resolution: float) -> Tuple[float, float]:
        """맵 픽셀 좌표를 월드 좌표로 변환"""
        try:
            world_x = origin_x + (map_x + 0.5) * resolution
            world_y = origin_y + (map_y + 0.5) * resolution
            return world_x, world_y
        except Exception:
            return 0.0, 0.0
    
    @staticmethod
    def validate_polygon(polygon: List[Tuple[float, float]]) -> bool:
        """폴리곤이 유효한지 확인"""
        try:
            if len(polygon) < 3:
                return False
            shapely_polygon = ShapelyPolygon(polygon)
            return shapely_polygon.is_valid
        except Exception:
            return False
    
    @staticmethod
    def calculate_polygon_area(polygon: List[Tuple[float, float]]) -> float:
        """폴리곤 면적 계산"""
        try:
            shapely_polygon = ShapelyPolygon(polygon)
            return shapely_polygon.area
        except Exception:
            return 0.0
    
    @staticmethod
    def get_polygon_centroid(polygon: List[Tuple[float, float]]) -> Tuple[float, float]:
        """폴리곤 중심점 계산"""
        try:
            shapely_polygon = ShapelyPolygon(polygon)
            centroid = shapely_polygon.centroid
            return (float(centroid.x), float(centroid.y))
        except Exception:
            # 실패 시 단순 평균 계산
            if polygon:
                avg_x = sum(p[0] for p in polygon) / len(polygon)
                avg_y = sum(p[1] for p in polygon) / len(polygon)
                return (avg_x, avg_y)
            return (0.0, 0.0)
    
    @staticmethod
    def clip_coordinates_to_bounds(coords: List[Tuple[float, float]], 
                                  min_x: float, min_y: float, 
                                  max_x: float, max_y: float) -> List[Tuple[float, float]]:
        """좌표를 지정된 경계 내로 클리핑"""
        clipped = []
        for x, y in coords:
            clipped_x = max(min_x, min(max_x, x))
            clipped_y = max(min_y, min(max_y, y))
            clipped.append((clipped_x, clipped_y))
        return clipped
