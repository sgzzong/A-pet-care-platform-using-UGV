# Manipulator_RL_reach_Policy-Ditillation
Lightening the AI model to reach the target point of the robot arm with policy Distillation

#### Notice
>this repository is based on CUN-bjy/policy-distillation-baselines

#### Evironment
>os : Ubuntu 20.04 LTS<br/>
>python V : 3.8.10<br/>
>algo : ppo<br/>
>robot : ned2<br/>
>env : GAZEBO<br/>
>Task : Single target reach

## installation(option)
  moveit install : sudo apt install ros-noetic-moveit

  joint controller install : sudo apt-get install ros-noetic-joint-trajectory-controller

  ros-bridge install : sudo apt-get install ros-noetic-rosbridge-server

  trajectory install : sudo apt-get install ros-noetic-joint-trajectory-controller

## Demonstration
Model size : Teacher model ( Hidden layer : 256 * 128 * 64 ) >> Distilled Student model ( Hidden layer : 128 * 64 * 32 )

Training Time : 90 minutes >> 20 minutes

## Overview
<div align="center">
  <img src="https://github.com/sgzzong/Ned2_RL_reach_Policy-Ditillation/assets/86835751/15bff64d-5b06-407f-8641-da0caa9d2e78">
</div>
![image](https://github.com/sgzzong/Manipulator_RL_reach_Policy-Ditillation/assets/86835751/afaac763-26f9-4d62-ab60-e063352deb9b)

## Installation
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/sgzzong/Ned2_RL_reach_Policy-Ditillation.git
  virtualenv venv
  source venv/bin/active
  install -r requirements.txt
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash # update workspace
  ```

## Manipulator(ned2 6DOF) environment setting with GAZEBO
```bash
roslaunch ned2_moveit demo_gazebo.launch #defalut 10x speed
#If you want to change the speed, modify the physical parameters in Gazebo
```
## Teacher model reinforcement learning
Learning the teacher model to be used for policy distillation

If you want to change the network size, modify the layer on NN.py and the Config.py file
```bash
python PPO_Robotic_arm_training.py # learning teacher model
python PPO_Robotic_arm_Inference.py # Demonstration of the Learned Teacher Model
```

## Policy Distillation
Student model learning using teacher model knowledge

If you want to change the network size, modify the layer on NN.py and the Config.py file

```bash
#check the teacher model path
python Policy_distillation.py # Distlling
python model_Inference.py #  Demonstration of the Distilled Student Model
```
## Experiment result
<div align="center">
  <img src="https://github.com/sgzzong/Manipulator_RL_reach_Policy-Ditillation/assets/86835751/ef7a1b23-c5e6-40bf-97b8-4978311be200">
</div>
