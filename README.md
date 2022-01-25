<h1> Sinusoidal Magistral Series BLDC Motor Driver for ROS</h1>
<h2>Requirements</h2>
<strong><p>1- Install hidapi</p></strong>
</p>pip install hidapi==0.10.1</p>

<h2>Create udev Rules :</h2>

1 - At the root folder

2 - sudo nano /etc/udev/rules.d/99-motordriver.rules 

3 - copy and paste udev rules : 
     
	   SUBSYSTEM=="input", GROUP="input", MODE="0666"

     SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="574d", MODE="666", GROUP="plugdev"

     KERNEL=="hidraw*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="574d", MODE:="0666", GROUP="plugdev"

4- ctrl+x and Y for save the udev rules.

5- sudo udevadm control --reload-rules && sudo udevadm trigger <br/>
   or </br>
   Reboot.

<h2>Installation</h2>
<p>1-mkdir catkin_ws1</p>
<p>2-cd catkin_ws1</p>
<p>3-mkdir src</p>
<p>4-cd src</p>
<p>5-git clone for the repo</p>
<p>6-catkin_init_workspace</p>
<p>7-cd ..</p>
<p>8-catkin_make</p>
<p>9-source devel/setup.bash</p>

<h2>Run</h2>
<p>source devel/setup.bash</p>
<p>roslaunch sinusoidal motordriver.launch</p>
