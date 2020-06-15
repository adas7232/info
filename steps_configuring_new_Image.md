### Step 1 ###
**Take ownership of the home directory/folder**
When using someone else's image, most likely the ownership is not transferred. Transfer the ownership of the home directory or a particular folder by using the command below 
```
sudo chown pi:pi /home/pi/
sudo chmod 750 /home/pi
```

