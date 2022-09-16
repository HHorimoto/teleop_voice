# teleop_voice

**This package controls your robot with voice.**

## Requirement
+ Ubuntu 18.04
+ ROS (Melodic)
+ Python 2.7.x

## Set Up
1. Download `teleop_voice` package.

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/HHorimoto/teleop_voice.git
$ cd ~/catkin_ws
$ catkin_make
```

2. Download `visualization-rwt` package. it is for voice recognition

```shell
$ apt-get install ros-$ROS_DISTRO-visualization-rwt
```

THANK YOU SO MUCH FOR [visualization-rwt](https://github.com/tork-a/visualization_rwt)

## How To Use

1. Launch `visualization-rwt` launch file.

```shell
$ roslaunch rwt_speech_recognition rwt_speech_recognition.launch
```

2. Go to below link and speak what you want.

```shell
http://localhost:8000/rwt_speech_recognition/
```

+ This is available voice.
```shell
go forward
go back
turn left
turn right
```

3. Launch `teleop_voice` launch file.

```shell
$ roslaunch teleop_voice teleop_voice.launch
```

### Parameters

+ ***low_th*** : Minimum confidence threshold.
    default : `0.5`

+ ***linear_vel*** : Seep of liner.
    default : `0.2` [m]

+ ***angular_vel*** : Seep of angular.
    default : `0.349066` [rad]

+ ***topic_vel*** : Topic name of velocity.
    default : `/cmd_vel`

+ ***topic_voice*** : Topic name of voice recoginition from `visualization-rwt`.
    default : `/Tablet/voice`