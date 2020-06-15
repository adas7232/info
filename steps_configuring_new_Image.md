### Step 1 ###
**Taking ownership of the home directory/folder**
When using someone else's image, most likely the ownership is not transferred. Transfer the ownership of the home directory or a particular folder by using the command below 
```
sudo chown -R pi:pi /home/pi/
sudo chmod -R 750 /home/pi
```
### Step 2 ###
**Activating Wireless**
wireless can be added by using ```raspi_config``` command if using raspbian

### Step 3 ###
**Configuring ROS - Creating new workspace**
```mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash```

### Step 4 ###

**Configuring git - generate ssh key pair from server and add it to git**
Note that, the .ssh folder may be in /root. If so, create a .ssh folder in home directory (```/home/pi/.ssh```).
Generate ssh key pair in the newly created directory using folder command as shown below - 
```ssh-keygen -t rsa -f home/pi/.ssh/id_rsa```
Now open the pub key with vi editor and copy the public key
```vi id_rsa.pub``` and quit the editor using ```:q!``` 
Now add this new key to github. 
In the server, add the git user 
```git config --global user.email "you@example.com"
git config --global user.name "Your Name"```
Test the ssh connection by 
```ssh -T git@github.com```

**Creating new package**
```catkin_create_pkg rpi_ros_car std_msgs geometry_msgs rospy roscpp```
