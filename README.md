# F1tenth-racing-obstacle-avoidance
1) 개요
 "우분투환경에서의 ROS 2를 이용한 F1tenth 장애물 회피 프로젝트"
 지능로봇공학과 2학년 2학기 프로그래핑 프로젝트 과목에서 진행한 F1TENTH 자율주행 시뮬레이션 프로젝트입니다. 장애물을 인식하고 회피하며 주행하는 자율주행 알고리즘을 구현한 것입니다.
 Pure Pursuit 경로 추정 알고리즘과 APF(Artificial Potential Field) 장애물 회피 알고리즘을 결합한 하이브리드 제어기를 ROS 2와 C++로 직접 구현했습니다..

2) 데모 영상


3) 개발환경
 OS: ubuntu 22.04
 ROS 2 Humble
 Simulator: F1tenth Simulator
 언어:C++

4) 빌드및 실행방법
 # 워크스페이스 빌드
 cd ~/racecar_simulator
 cbr
 sis
 # terminal 1 
 ros2 launch racecar_simulator simulator.launch.py
 # terminal 2
 ros2 run pure_pursuit pure_pursuit_node
 # terminal 3
 ros2 run pure_pursuit controller_node

5) 핵심 기능 및 구현
 # 장애물 인식
 # 장애물 회피 알고리즘

6) 배운 점 및 어려웠던 점
   다음에는 중앙라인만 따라가는것이아닌 인코스 및 가속주행을하여 RAP타임을 줄이면서 장애물 회피까지 가능한 알고리즘을 구현하는 것이 목표이다.
