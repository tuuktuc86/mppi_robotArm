# mppi_robotArm
2DOF Robotic Arm Control System Using MPPI Controller

## 개요
robot arm system에 MPC(model Predictive Control)의 일종인 mppi 방법을 적용하는 프로젝트입니다.

기존 PID 제어기로 동작하는 2DOF 로봇팔에 mppi controller를 달아서 동작하도록 개발하였습니다. mppi_robotArm 폴더에 있습니다. <br>
참고한 코드의 깃허브 주소는 다음과 같습니다.<br>
https://github.com/MizuhoAOKI/python_simple_mppi <br>
sample_mppi 폴더가 이에 해당합니다.<br>
robot arm 소스코드는 origin_controller/Arm 폴더에 저장되어 있습니다. 원본 코드는 matlab 코드인데 해당 버전은 python으로 수정한 버전입니다.<br>

## 함수
기존 controller는 다음과 같은 함수가 있습니다.

|함수 이름|기능|
|---|---|
|Arm_Dynamic|로봇팔의 동역학 모델을 사용하여 토크를 계산하는 함수로, 제어에 필요한 다이내믹 모델링을 수행합니다. 이는 주로 인버스 다이내믹스(Inverse Dynamics) 제어에서 사용될 수 있습니다.|
|Forward_Kinematic|로봇팔의 관절 각도를 받아 로봇팔의 위치를 계산하는 함수입니다. 이는 로봇팔의 위치를 추적하거나 센서값을 이용해 로봇팔의 위치를 알아내는데 사용될 수 있습니다.|
|Inverse_Kinematic| 로봇팔의 목표 위치를 받아 관절 각도를 계산하는 함수입니다. 이는 로봇팔이 특정 위치로 이동하도록 하는 역기구학적(Inverse Kinematics) 제어에 사용됩니다.|
|Feedback_linearization| 로봇팔의 동역학 모델을 사용하여 피드백 선형화(feedback linearization)를 수행하여 토크를 계산하는 함수입니다. 이는 주로 비선형 제어 기법 중 하나로, 선형 제어기를 사용하여 비선형 시스템을 제어하기 위해 사용됩니다.|
|Controller| 주어진 관절 각도, 각 속도, 목표 위치 및 목표 속도를 사용하여 제어 입력을 계산하는 함수입니다. 이 함수는 주어진 목표에 대해 피드백 제어를 수행합니다.|


우리는 mppi controller를 사용하여 다음과 같은 함수만 사용합니다.<br>
Arm_Dynamic 함수, Forward_Kinematic 함수, Controller 함수, mppi controller 클래스<br>

## sampling

|sampling1|sampling2|
|---|---|
|<img src="https://github.com/tuuktuc86/mppi_robotArm/blob/master/project_img/sampling_traj_2.png">|<img src="https://github.com/tuuktuc86/mppi_robotArm/blob/master/project_img/sampling_traj_1.png">|

## 특이사항


## 동영상

|sampling1|sampling2|
|---|---|
|<img src="https://github.com/tuuktuc86/mppi_robotArm/blob/master/project_img/mppi_traj-ezgif.com-video-to-gif-converter.gif">|<img src="https://github.com/tuuktuc86/mppi_robotArm/blob/master/project_img/original_controller_0.25-2024-04-30_22.42.33-ezgif.com-video-to-gif-converter.gif">|

