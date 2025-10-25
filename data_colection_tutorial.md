## **Data collection on real robot|KUAVO** ##


## **On KUAVO** ##
>1. Use SSH to connect to KUAVO. To enter password type 3 times space (three spaces)
```bash 
ssh lab@192.168.3.9
```

>2. Launch two terminals 

>2.1 go to /home/lab/syp/kuavo-ros-control

>3. Inside each enter `KUAVO's` $ROS_MASTER_URI and 's $ROS_IP

Current KUAVO's URI:
```bash 
export ROS_MASTER_URI=http://192.168.3.3:11311
```
KUAVO's IP:
```bash 
export ROS_IP=192.168.3.9
```

>4. Inside 1st terminal: 

```bash
 source devel/setup.bash
```
then :

```bash
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true
```

then type `o` hit enter , `o` hit enter , `v` hit enter

>5. Inside 2nd terminal:

```bash
 source devel/setup.bash
```
then 

```bash
python /home/lab/syp/kuavo-ros-control/src/hand_eye_calibration/kuavo_hand_eye_calibration/scripts/pose_ik_solver.py
```

## **On Radxa** ##
>1. Connect radxa to local net using Wlan 

>2. Use SSH to connect to it , password the same as username `radxa`

```bash
ssh radxa@192.168.3.52
```

>3. Launch 3 terminals via using docker 

1st terminal:

```bash 
docker start -ai exosuit_nodes
```

2nd and following  terminals: 

```bash 
docker exec -it exosuit_nodes bash 
```

>4. Connect USB servo-readers to PC, you can check connected ID's using


```bash 
ls /dev/ttyACM*
```
Left hand should be `/dev/ttyACM0` , right `/dev/ttyACM1`

>5. Give permission if needed 


```bash 
sudo chmod 666 /dev/ttyACM*
```

>6. **BEFORE USING ROS** enter `KUAVO's` $ROS_MASTER_URI and computer's $ROS_IP inside each terminal

Current KUAVO's URI:
```bash 
export ROS_MASTER_URI=http://192.168.3.3:11311
```
Radxa IP:
```bash 
export ROS_IP=192.168.3.52
```

>7. Source workspace `/home/ExoSuit/exo_publisher`

```bash 
source devel/setup.bash
```
>8. launch 

```bash 
roslaunch servo_angle_publisher deg_dual_arm_publisher.launch
```
