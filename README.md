 ========== 터미널 1: 매핑 시작 ==========
 
ros2 launch turtlebot3_zed_nav turtlebot3_zed_mapping.launch.py



 ========== 터미널 2 (새 SSH 세션): 키보드 조종 ==========
 
ros2 run turtlebot3_teleop teleop_keyboard


 터미널 1, 2에서 Ctrl+C로 종료
 데이터베이스가 자동 저장됨: ~/.ros/rtabmap_burger.db

확인

ls -lh ~/.ros/rtabmap_burger.db



 ========== 터미널 1: 네비게이션 시작 ==========
 
ros2 launch turtlebot3_zed_nav turtlebot3_zed_navigation.launch.py




✅ [rtabmap]: Database loaded: ~/.ros/rtabmap_burger.db

✅ [rtabmap]: RTAB-Map started in localization mode

⏳ [rtabmap]: Loop closure detection...

✅ [rtabmap]: Loop closure detected! ID=XX  ← 이게 나와야 함!




Loop closure 안 나오면:

bash# 로봇을 천천히 360도 회전시키기

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}" --onc



 테스트
 
ros2 run turtlebot3_zed_nav send_goal 1.0 0.5 0
