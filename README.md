# Rulebook Constrainer

ROS2 패키지로 YAML 규칙서를 로딩하여 지도 기반 규칙 마스크를 생성하고 Nav2와 프레임워크 A(LLM) 통합을 지원합니다.

## 주요 기능

- YAML 규칙서에서 keepout 영역, 속도 제한, 차선 선호도 등을 파싱
- Nav2 costmap filter layer용 OccupancyGrid 마스크 생성 및 발행
- LLM 프레임워크 A 통합을 위한 규칙 JSON 서비스 제공
- 실시간 규칙 갱신 지원

## 토픽 및 서비스

### 발행 토픽
- `/rule_mask_keepout` (nav_msgs/OccupancyGrid): Keepout 영역 마스크
- `/rule_mask_speed` (nav_msgs/OccupancyGrid): 속도 제한 영역 마스크
- `/rule_mask_lane_preference` (nav_msgs/OccupancyGrid): 차선 선호도 마스크
- `/rule_visualization` (visualization_msgs/MarkerArray): RViz 시각화 마커

### 서비스
- `/rulebook/json` (std_srvs/Trigger): LLM 프롬프트용 규칙 JSON 반환
- `/rulebook/reload` (std_srvs/Trigger): 규칙서 파일 다시 로딩

## 파라미터

- `rulebook_yaml`: 규칙서 YAML 파일 경로
- `mask_resolution`: 마스크 해상도 (m/pixel)
- `publish_rate_hz`: 마스크 발행 주기 (Hz)
- `filters.keepout`: Keepout 마스크 활성화 여부
- `filters.speed`: 속도 마스크 활성화 여부
- `filters.lane_preference`: 차선 선호도 ("right", "left", "none")
- `robot_footprint_radius`: 로봇 반경 (m)

## 실행 순서

### 1. Nav2 Bringup 및 SLAM/지도 준비
```bash
# TurtleBot3 환경 설정
export TURTLEBOT3_MODEL=waffle_pi

# Gazebo 시뮬레이션 (또는 실제 로봇)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# SLAM (새 지도 생성 시)
ros2 launch turtlebot3_cartographer cartographer.launch.py

# 또는 기존 지도 사용
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=your_map.yaml
```

### 2. Rulebook Constrainer 패키지 실행
```bash
# 패키지 빌드
colcon build --packages-select rulebook_constrainer

# 환경 소싱
source install/setup.bash

# 노드 실행
ros2 launch rulebook_constrainer rulebook_constrainer.launch.py
```

### 3. RViz에서 마스크 확인
```bash
# RViz 실행 (Nav2용 설정 사용)
ros2 launch nav2_bringup rviz_launch.py

# 추가 토픽 시각화:
# - /rule_mask_keepout (Map으로 표시)
# - /rule_mask_speed (Map으로 표시)
# - /rule_visualization (MarkerArray로 표시)
```

### 4. Nav2 Costmap Filter Server 설정 예시

Nav2 costmap에 rule mask를 적용하려면 costmap 설정에 filter layer를 추가하세요:

```yaml
# nav2_params.yaml에 추가
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer", "keepout_filter", "speed_filter"]
      
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/rule_mask_keepout"
        
      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        enabled: True
        filter_info_topic: "/rule_mask_speed"
        base_speed_to_max_speed_ratio: 0.2

global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter", "speed_filter"]
      
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/rule_mask_keepout"
        
      speed_filter:
        plugin: "nav2_costmap_2d::SpeedFilter"
        enabled: True
        filter_info_topic: "/rule_mask_speed"
        base_speed_to_max_speed_ratio: 0.2
```

## 프레임워크 llm_candidate_planner_core 통합

LLM 경로 계획기에서 규칙 정보를 사용하려면:

```python
# 프레임워크 "llm_candidate_planner_core"에서 사용 예시
import rclpy
from std_srvs.srv import Trigger

# 서비스 클라이언트 생성
rulebook_client = node.create_client(Trigger, '/rulebook/json')

# 규칙 JSON 요청
request = Trigger.Request()
future = rulebook_client.call_async(request)

# 응답에서 규칙 JSON 추출
response = future.result()
if response.success:
    rules_json = response.message  # JSON 문자열
    # LLM 프롬프트에 rules_json을 추가하여 경로 계획 시 고려
```

## YAML 규칙서 형식

```yaml
# example_rulebook.yaml
map_frame: "map"
rules:
  keepout_zones:
    - type: "polygon"
      name: "exhibition_area"
      coordinates: [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]
      reason: "박물관 전시 앞"
    - type: "circle"
      name: "emergency_exit"
      center: [5.0, 5.0]
      radius: 1.5
      reason: "비상구 확보"
  
  speed_zones:
    - type: "polygon"
      name: "school_zone"
      coordinates: [[10.0, 10.0], [15.0, 10.0], [15.0, 15.0], [10.0, 15.0]]
      max_speed: 0.3
      reason: "학교 앞 저속 구역"
  
  lane_preferences:
    - type: "line"
      name: "main_corridor"
      start: [0.0, 0.0]
      end: [20.0, 0.0]
      preference: "right"
      reason: "복도 우측통행"

landmarks:
  museum_entrance: [2.0, 2.0]
  school_gate: [12.5, 12.5]
```

## 테스트 및 검증

```bash
# 서비스 테스트
ros2 service call /rulebook/json std_srvs/srv/Trigger

# 토픽 확인
ros2 topic echo /rule_mask_keepout --once
ros2 topic list | grep rule_mask

# 파라미터 확인
ros2 param list | grep rulebook_constrainer
```

