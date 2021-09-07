# Social_Robot_Moveit
  ``` bash
  소셜 로봇 시뮬레이션 확인
  ```


# Moveit setting

1. Moveit 설치 - moveit assistance 설치
  ``` bash
  http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html 
  ```
2. Social Robot moveit git setting
  ``` bash
  git clone https://github.com/kimun608/Social_Robot_Moveit.git
  ```



# 소셜 로봇 동작 추출
1. Moveit assistance 실행
  ``` bash
  roslaunch moveit_setup_assistant setup_assistant.launch
  ```
2. 소셜 로봇 URDF 불러오기
  ``` bash
  ![image](https://user-images.githubusercontent.com/60381453/132274084-c560dce0-4411-49f3-a02b-77d64080ae6e.png)
  Edit Existing Moveit을 이용하여 NAME_WORKSPACE/src/Social_Robot_Moveit/ 을 선택하여 불러옴
  ```
3. 목표 joint 설정
  ``` bash
  ![image](https://user-images.githubusercontent.com/60381453/132274436-85c45e28-ef23-4b56-b2c8-9c6fee9e6937.png)
  Robot Poses에 들어가 해당 로봇 목표 joint 설정 값 가져오기
  ```
3. 목표 값을 text, joint 값으로 변환
  ``` bash
  목표 값을 /src/Social_Robot_Moveit/src/excution.py 파일에 goal joint에 입력하여 json, text 파일 추출
  ```
4. 설정된 joint 값을 Robotcare package에 넣음(robotcare 패키지가 있을 경우, 없으면 시뮬레이션으로 확인 가능)
  ``` bash
  text 파일을 /src/socialrobot_ros/social_motion/social_motion_res/joint
  json 파일을 /src/socialrobot_ros/social_motion/social_motion_res/motion 
  위 폴더에 넣음
  ```


# 추출 동작 로봇에서 실행
1. service call로 확인
  ``` bash
  ex)
  rosservice call /social_motion_player/play_motion "file_name: 'PROFILE_NAME'
  text: ''
  with_home: false" 
  
  service call로 확인
  ```


# 시뮬레이션에서 추출 동작 및 로봇 동작 확인
1. service call로 확인
  ``` bash
  34234
  ```
