# ExoSuits
---
**This project is about Exosuit for bimanual robot teleoperation**

![ExoSuit Demo](assets/example_short.gif)

# Updates
---
**Current updates**
> `v2🎉` 24 November 2025 
- New ExoSuit structure 
- Data collection now require only one user due too updated data collection framework

> `v1.3` 21 October 2025 
"Data collection" can be started with joycon

> `v1.2` 10 October 2025
Now you can collect data from real robot KUAVO, including 7DOF both hands position and grip control. Check below

> `v1.1` 13 Jun 2025
Now you can read angle data by script in: /ExoSuit/exo_publisher/src/servo_angle_publisher/scripts/dual_arm_angle_publisher.py

**Future tasks:**
> Data collection in simulation


# **Data collection on real robot | KUAVO** #
---
# **On KUAVO** #

>1. Use SSH to connect to KUAVO. To enter password type 3 times space (three spaces)
```bash 
ssh lab@192.168.3.9
```

>2. go to /home/lab/syp/kuavo-ros-control

>2.1 Launch three terminals 

>3. Inside each terminal enter `KUAVO's` $ROS_MASTER_URI and $ROS_IP. You can check it by using 

```bash
echo $ROS_IP
```
```bash
echo $ROS_MASTER_URI
```
 $ROS_MASTER_URI also can be displayed in output of roscore

>4. Inside first terminal run 

```bash
 source devel/setup.bash
```

```bash
roscore
```

>5. Inside 2nd terminal: 

```bash
 source devel/setup.bash
```
then :

```bash
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true
```

Then follow instructions inside terminal: 
- Type `o`, hit enter
- If Kuavo "sit" , make feets touch the ground and then type `o`, hit enter 
- If no errors in output - type `v`, hit enter, Kuavo must stand up

>6. Inside 3rd terminal:

```bash
 source devel/setup.bash
```
then 

```bash
python /home/lab/syp/kuavo-ros-control/src/hand_eye_calibration/kuavo_hand_eye_calibration/scripts/pose_ik_solver.py
```

# **On Radxa** #

**!Check if joycons are connected!**

>1. To start data collection, First need to change $ROS_MASTER_URI and $ROS_IP of microcomputer. $ROS_MASTER_URI is the same as on Kuavo.  You can check $ROS_IP by using 

```bash
echo $ROS_IP
```
>2. Open `/ExoSuit/real_robot_data_collection/start_collection.sh`, change $ROS_MASTER_URI and $ROS_IP to correct one on line 8-9

```
    #=========REAL ROBOT APPLICATION=========
    ROS_MASTER_URI="http://192.168.3.9:11311"
    ROS_IP="192.168.3.52" 
    #========================================
```

>3. Inside `/ExoSuit/real_robot_data_collection/` launch

```bash
./start_collection.sh
```
You will start `tmux` session with 4  terminals, each related to data collection, servo angle publishing, grip state publishing and topics display
- to start/stop servo angle publishing press "Y" on right Joycon.
- to start/stop grip state publishing press "A" on right Joycon. To open/close robot's grip hit ZR/ZL on joycons.
- to start/stop data collection press "X" on right Joycon. All records are stored inside `/ExoSuit/records/`.
