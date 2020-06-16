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
source devel/setup.bash
```
To make it permanent, open ```nano ~/.bashrc```  and add a line at the end like below    
```source /home/pi/catkin_ws/devel/setup.bash```   
Also, if you would like your terminal to automatically direct to a specific folder, you may add another line as below 
```cd [project folder path]```
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
git config --global user.name "Your Name"
```
Test the ssh connection by 
```ssh -T git@github.com```
If successful, do the git clone from the existing git repository 
```/home/pi/catkin_ws/src/```

**Creating new package - same name of existing git repo**  
Assuming the package is already backed-up in git, this step not be needed. Just try to do ```catkin_make``` from ```/home/pi/catkin_ws/```. If not successful, then try the following command 
```catkin_create_pkg rpi_ros_car std_msgs geometry_msgs rospy roscpp```

### Step 5 ###

**Configuring vscode - ssh to raspberry pi**   
If ssh key pair already exists for windows, then copy the public key content into clipboard. If not, generate one either with *openssh* or *puttygen*. On the server side, go to ```/home/pi/.ssh/``` and open a empty file using *nano* or *vi*. Copy the content of the clipboard (public ssh key) into the file and save it as *autorized_keys* 
Check the connection by going into the Windows command window and type 
```ssh pi@ipaddress```
The connection should happen without any issues. Then, go to *vscode* and use *F1* or *ctrl + shift + P* to open the command pallete and search for ssh. click *Open Configuration File* and edit it as below 
```
Host anyname
  HostName ip-address
  User pi
  ForwardAgent yes
  IdentityFile C:\Users\user\.ssh\id_rsa
  Port 22
  ```
Please note that if you are using ssh for the first time in vscode, then you may need the following extensions installed for successful ssh connection. 
1. Remote Development
2. Remote SSH (Nightly)   

Now ssh connection from vscode (Windows) to raspberry pi is established. Lastly, dont forget to change the EOL sequence to *LF* to avoid any issues on raspberry pi. To change it permanently for any new code, go to *File -> Preferences -> Setting* and serch for EOL. Then set it up for ```\n```. Thats shoud be it. 
