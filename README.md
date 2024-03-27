# follow-cart


## OS: ubuntu 22.04
## ROS2 배포판: ROS2 iron

![image](https://github.com/follow-cart/follow-cart/assets/76193871/056d14de-6243-4535-b3ef-a6cb60f31de8)

![image](https://github.com/follow-cart/follow-cart/assets/76193871/a6551c5f-80e9-4152-8e06-5a3fed32636a)

![image](https://github.com/follow-cart/follow-cart/assets/76193871/0757e9ca-d7d1-430f-8781-faf2cc3d42d4)


## 개발환경 설정
1. 워크스페이스 경로 통일(home/workspace/follow_cart_ws/src)
2. src 폴더 내에서 아래 명령 실행


```
pip install -r requirements.txt
```

3. aws warehouse world package 설치
- home 디렉토리에 .rosinstall 파일 생성
- 아래 코드 .rosinstall 파일에 추가

```
- git: {local-name: src/aws-robomaker-small-warehouse-world, uri: 'https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git', version: ros2}
```

- follow_cart_ws 디렉토리에서 아래 명령어들 순서대로 실행

```
rosws update
```

```
rosdep install --from-paths . --ignore-src -r -y
```

```
source /usr/share/gazebo/setup.sh
```

```
colcon build
```
