
# 🧪 Class 1 Lab – Advanced ROS Noetic with Python (TurtleBot3)

## 📋 Preparation (Do Once Before Exercises)

```bash
# Create workspace and package
cd ~
# assuming you dont have src folder (skip if you have it)
mkdir -p catkin_ws/src

cd catkin_ws/src
catkin_create_pkg class_1 std_msgs rospy geometry_msgs nav_msgs std_srvs message_generation
cd ~/catkin_ws && catkin_make
source devel/setup.bash


cd catkin_ws/src/class_1/src
touch dynamic_speed.py distance_monitor.py robot_status_publisher.py
chmod +x *.py
```

---

## ✅ Exercise 1: Dynamic Speed Publisher

**Objective**: Toggle robot forward speed every 5 seconds.

### 📂 `src/dynamic_speed.py`

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def dynamic_speed():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('dynamic_speed', anonymous=True)
    slow_speed = 0.1
    fast_speed = 0.4
    toggle_period = rospy.get_param('~toggle_period', 5.0)
    rate = rospy.Rate(10)
    start_time = rospy.get_time()
    use_slow_speed = True

    while not rospy.is_shutdown():
        elapsed = rospy.get_time() - start_time
        if elapsed >= toggle_period:
            use_slow_speed = not use_slow_speed
            start_time = rospy.get_time()

        twist = Twist()
        twist.linear.x = slow_speed if use_slow_speed else fast_speed
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        dynamic_speed()
    except rospy.ROSInterruptException:
        pass
```

### ▶️ Run

```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Terminal 2
cd ~/catkin_ws && source devel/setup.bash
rosrun class_1 dynamic_speed.py
```

---

## ✅ Exercise 2: Distance from Odometry

**Objective**: Subscribe to `/odom`, compute distance, and publish it.

### 📂 `src/distance_monitor.py`

```python
#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse

initial_pose = None
current_pose = None

def odom_callback(msg):
    global initial_pose, current_pose
    pos = msg.pose.pose.position
    current_pose = pos
    if initial_pose is None:
        initial_pose = pos

def reset_distance_callback(req):
    global initial_pose, current_pose
    if current_pose:
        initial_pose = current_pose
    return EmptyResponse()

def distance_monitor():
    rospy.init_node('distance_monitor')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    pub = rospy.Publisher('/traveled_distance', Float32, queue_size=10)
    rospy.Service('/reset_distance', Empty, reset_distance_callback)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if initial_pose and current_pose:
            dx = current_pose.x - initial_pose.x
            dy = current_pose.y - initial_pose.y
            dist = math.sqrt(dx**2 + dy**2)
            pub.publish(dist)
        rate.sleep()

if __name__ == '__main__':
    distance_monitor()
```

### ▶️ Run

```bash
rosrun class_1 distance_monitor.py
```

### 🔁 Reset Service

```bash
rosservice call /reset_distance
```

---

## ✅ Exercise 3: Launch File

### 📂 `launch/turtlebot_launch.launch`

```xml
<launch>
  <param name="toggle_period" value="5.0" />
  <node pkg="class_1" type="dynamic_speed.py" name="dynamic_speed" output="screen" />
  <node pkg="class_1" type="distance_monitor.py" name="distance_monitor" output="screen" />
</launch>
```

### ▶️ Run

```bash
roslaunch class_1 turtlebot_launch.launch
```

---

## ✅ Exercise 4: Reset Distance with Service

Already implemented in `distance_monitor.py`.

### ▶️ Reset During Run

```bash
rosservice call /reset_distance
```

---

## ✅ Exercise 5: Custom Message – `RobotStatus`

### 📂 `msg/RobotStatus.msg`

```text
string status_message
float32 traveled_distance
float32 current_speed
```

### 🛠 Modify `package.xml`

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

### 🛠 Modify `CMakeLists.txt`

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES RobotStatus.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```

### 🧱 Rebuild

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 📂 `src/robot_status_publisher.py`

```python
#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from class_1.msg import RobotStatus

initial_pose = None
current_pose = None
current_speed = 0.0

def odom_cb(msg):
    global current_pose, initial_pose
    pos = msg.pose.pose.position
    current_pose = pos
    if initial_pose is None:
        initial_pose = pos

def cmd_cb(msg):
    global current_speed
    current_speed = abs(msg.linear.x)

def get_distance():
    if initial_pose and current_pose:
        dx = current_pose.x - initial_pose.x
        dy = current_pose.y - initial_pose.y
        return math.sqrt(dx**2 + dy**2)
    return 0.0

def publisher():
    rospy.init_node('robot_status_publisher')
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('/cmd_vel', Twist, cmd_cb)
    pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = RobotStatus()
        msg.traveled_distance = get_distance()
        msg.current_speed = current_speed
        msg.status_message = "Moving fast" if current_speed >= 0.25 else "Moving slowly"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    publisher()
```

### ▶️ Run

```bash
rosrun class_1 robot_status_publisher.py
```

### 📡 Visualize Output

```bash
rostopic echo /robot_status
```

---

## ✅ Summary Table

| Exercise | Command |
|----------|---------|
| Gazebo Sim | `roslaunch turtlebot3_gazebo turtlebot3_world.launch` |
| Ex1 – Dynamic Speed | `rosrun class_1 dynamic_speed.py` |
| Ex2 – Distance Monitor | `rosrun class_1 distance_monitor.py` |
| Ex3 – Combined Launch | `roslaunch class_1 turtlebot_launch.launch` |
| Ex4 – Reset Distance | `rosservice call /reset_distance` |
| Ex5 – Custom Status Msg | `rosrun class_1 robot_status_publisher.py` |
