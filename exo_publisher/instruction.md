

# **If you use LINUX**
---

1. Connect USB servo-readers to your PC

2. Check connected ID's
 
```
ls /dev/ttyACM*
```
output example:    
```
user@user:~/.....$ ls /dev/ttyACM*
/dev/ttyACM0
```

3. Пive permission, change ttyACM* to your connected usb ID's

```
sudo chmod 666 /dev/ttyACM*
```
4. Inside dual_arm_angle_publisher.py on lines 12 & 13 you have to change ttyACM* numbers to current. 

as example in code:
```
LEFT_USB_PORT = "/dev/ttyACM0"
RIGHT_USB_PORT = "/dev/ttyACM1"
```
5. Now you can run script to read angle data 

**! Before launch stay still and calm in exo suit, hold your hands down, it's starting position !**
 
```
roslaunch servo_angle_publisher dual_arm_publisher.launch 
```

---
# **If you use WSL, FIRST FOLLOW THIS INSTRUCTION**
---
1. **Turn of vpn if it's working** and connect to USB servo-readers

2. In __Powershell__ check connected com ports
```
usbipd list  
```
output example: 
```
Connected:                                                                                                              BUSID  VID:PID    DEVICE                                                        STATE                                   1-1    1a86:55d3  USB-Enhanced-SERIAL CH343 (COM5)                              Not shared                              1-2    3151:3020  USB Input Device                                              Not shared                              1-14   8087:0aaa  Intel(R) Wireless Bluetooth(R)                                Not shared                                                                                                                                                      Persisted:                                                                                                              GUID                                  DEVICE                                                                            adc65a2d-75ee-4ac0-93d8-b22ad95ef470  USB-Enhanced-SERIAL CH343 (COM4)                                
```

3. In __Powershell__ bind ports for wsl

```
usbipd bind --busid 1-1
```
4. In __Powershell__ attach usb id's to wsl

```
usbipd attach --busid 1-1 --wsl Ubuntu-20.04
```
output example:    

```
usbipd: info: Selecting a specific distribution is no longer required. Please file an issue if you believe that the default selection mechanism is not working for you.                                                                         
usbipd: info: Using WSL distribution 'Ubuntu-20.04' to attach; the device will be available in all WSL 2 distributions. usbipd: info: Detected networking mode 'nat'.                                                                           usbipd: info: Using IP address 172.21.112.1 to reach the host.
```


5. __INSIDE WSL__ to check connected ID's
 
```
ls /dev/ttyACM*
```

output example:    
```
user@user:~/.....$ ls /dev/ttyACM*
/dev/ttyACM0
```
6. Inside dual_arm_angle_publisher.py on lines 12 & 13 you have to change ttyACM* numbers to current. 

as example in code:
```
LEFT_USB_PORT = "/dev/ttyACM0"
RIGHT_USB_PORT = "/dev/ttyACM1"
```

7. __INSIDE WSL__ give permission, change ttyACM* to your connected usb ID's

```
sudo chmod 666 /dev/ttyACM*
```

8. Now you can run script to read angle data 

**! Before launch stay still and calm in exo suit, hold your hands down, it's starting position !**
 
```
roslaunch servo_angle_publisher dual_arm_publisher.launch 
```