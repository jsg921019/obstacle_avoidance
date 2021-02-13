## [1기 정승균] Do not Collide 과제



### Ⅰ.  실행

---

#### 1. launch file

* PID + Stanley + Frenet 시뮬레이션 : `$ roslaunch simul stanleysimul.launch`
[<img src="https://img.youtube.com/vi/AKHuYRYmus0/0.jpg" alt="Watch the video" style="zoom:50%;" />](https://youtu.be/AKHuYRYmus0)

* PID + Pure Pursuit + Frenet 시뮬레이션 : `$ roslaunch simul ppsimul.launch`
[<img src="https://img.youtube.com/vi/e6x0UKBfd3c/0.jpg" alt="Watch the video" style="zoom:50%;" />](https://youtu.be/e6x0UKBfd3c)







### Ⅱ. stanleysimul.py / ppsimul.py

---



#### 1. Obstacles / Path 설정

---

```python
######## load reference path ########

rospack = rospkg.RosPack()
path = rospack.get_path("map_server")

with open(path + "/src/ref_path.pkl", "rb") as f:
    ref_path = pickle.load(f)


######## load obstacles info ########

path = rospack.get_path("obstacles")

with open(path + "/src/obstacles.pkl", "rb") as f:
    obstacles = pickle.load(f)
xy, yaw = np.hsplit(obstacles, [2])
yaw = np.column_stack([np.cos(yaw), np.sin(yaw)])
obstacles = np.vstack([xy -1.4*yaw, xy +1.4*yaw])
```

* 기존 reference path를 합쳐서 pickle로 저장하여 편하게 로드할 수 있도록 함
* obstacle의 좌표는 refrence path의 일정 s좌표마다 d좌표값을 +- 1 씩 주어 Frenet의 get_cartesian으로 변환하여 얻음
* 이 좌표는 장애물의 중앙에 대한 값이므로  더 엄밀한 탐지를 위하여 yaw값을 사용하여 양끝의 값을 얻음



#### 2. 목표 설정

---

```python
######## Target ########

target_speed = 20.0 / 3.6
start_x, start_y, start_yaw = ref_path["x"][30], ref_path["y"][30], ref_path["yaw"][30]
target_x, target_y = ref_path["x"][430], ref_path["y"][430]
```

* 속도는 주어진 대로 20km/h ≒ 5.56 m/s 로 설정
* 시작 위치는 reference path의 30번째 지점으로 설정
* 목표 위치는 reeference path의 430번째 지점으로 설정



#### 3. 인스턴스 생성

---

```python
######## Instances ########

# Marker Converter
path2marker = PathMarker()
paths2markerarray = PathsMarkerArray()
text2marker = TextMarker()

# driving car instance
car = KinematicBicycle(start_x, start_y, start_yaw)

# publishers
car.init_marker_pub(topic="driving", frame_id="map", ns="driving", id=1)
car.init_odom_pub(name="odom", child_frame_id="car1", frame_id="map")
paths_pub = rospy.Publisher("paths", MarkerArray, queue_size=1)

# controller instance
longitudinal_controller = PID_Controller(Kp=0.5, Kd=0, Ki=0.0003)
lateral_controller = Stanley(k=0.8, ks=0.5, kd=0, L=2.8)

# path finding instance
path_finder = Frenet(ref_path, start_x, start_y, start_yaw)
```

* Marker Converter들은 marker를 쉽게 생성해주는 객체. 예를 들어 PathMarker을 FrenetPath 객체를 Marker로 변환해준다.

* 자율주행하는 차는 KinematicBicycle 모델을 사용하였다.

* KinematicBicycle 객체안에 내부적으로 현재 상태를 odom이나 marker를 발행할 수 있도록 하였고 이를 활성화하였다.

* 찾은 경로의 경우 MarkerArray에 담아서 발행하도록 함

* longitudinal_controller의 경우 PID를 사용하였고 실험 결과 P제어만으로도 충분하였다. Kp값을 키우면 키울수록 목표 속도에 빠르게 접근하지만 실제 차에서 는 급발진이나 다름없기 때문에 적당히 0.5로 설정하였다.

* lateral_controller는 Stanley와 PurePursuit를 사용하였다.

  * Stanley의 경우 운이 좋았던건지 따로 튜닝을 안해도 잘 굴러갔다. ks 와 kd는 각각 저속, 고속에 대한 보정값인데 이 시뮬레이션에서는 속도가 정해져있기 때문에 크게 의미는 없었다.

  * PurePursuit 같은 경우는 lookahead distance를 크게 하면 장애물을 잘 회피지 못하였으므로 속도 5m/s 를 기주으로 5m앞을 lookahead distance로 잡도록 설정하였다(k=1).

* Path 생성은 Frenet Frame을 사용하였다.



#### 4. Main 

---

```python
######## Main ########
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    # find optimal path from Frenet
    paths, optimal_path = path_finder.find_path(car.x + car.L * np.cos(car.yaw), car.y + car.L * np.sin(car.yaw), obstacles)
    ma = []
    if optimal_path:
        ma.append(paths2markerarray.convert(paths))
        ma.append(path2marker.convert(optimal_path))
```

* 집 컴퓨터가 느리므로 rate을 10으로 변경 (기존: 100).
* path_finder 객체의 find_path 메소드로 차 위치 정보와 장애물 정보를 토대로 가능한 경로들 paths와 최적의 path인 optimal_path 계산
  * Stanley의 경우 차 앞바퀴쪽 위치 정보를 사용
  * PurePursuit의 경우 차 뒷바뀌쪽 위치 정보를 사용
* 존재하면 MarkerArray 에 담음



```python
    # update car
    ai = longitudinal_controller.feedback(car.v - target_speed, 0.1)
    if optimal_path:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, optimal_path.x, optimal_path.y, optimal_path.yaw)
    else:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, ref_path["x"], ref_path["y"], ref_path["yaw"])
    car.update(ai, di)
    ma.append(text2marker.convert("speed : %.2f" %car.v, 5))
```

* pid로 error = 현재속도-목표속도, dt=0.1 로 하여 가속도 ai를 구함
* optimal_path가 존재하면 그 경로를 바탕으로 lateral_controller로 조향각 di를 구함
* 존재하지 않으면 기존 경로를 바탕으로 lateral_controller를 사용. 실제로는 멈추거나 그래야겠지만 디버깅 목적으로 이렇게 설정하였다.
* 차의 현재 속도를 나타내는 TextMarker 생성. 별 필요는 없는데 Text Marker라는 것이 있길래 신기해서 사용하였다.



```python
    # check if near target
    if np.hypot(car.x - target_x, car.y - target_y) < 3 :  
        car.x, car.y, car.yaw, car.v, car.a = start_x, start_y, start_yaw, 0, 0
        path_finder.reset(start_x, start_y, start_yaw)
```
*  현재 차의 위치와 목표 위치가 가까우면 초기화



```python
    # publish car / paths / ui for visualization
    car.publish_marker()
    car.publish_odom()
    paths_pub.publish(ma)

    rate.sleep()
```

* 목표 지점이 아니면 발행할것들 발행하고 sleep