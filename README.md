# follow-cart


## OS: ubuntu 22.04
## ROS2 iron


1. 파일 경로 통일(home/workspace/follow_cart_ws/src)
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
