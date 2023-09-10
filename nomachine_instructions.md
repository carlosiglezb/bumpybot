
# Bumpybot Remote Desktop via NoMachine
For using GUI programs like RViz remotely on the Bumpybot computer, potentially avoiding the future neccesesity of the local (User) machine requiring ROS or any ROS connections.

### Installation & Setup
Download and Install the appropriate version of [NoMachine](https://www.nomachine.com/) for your machine/phone.

Ensure you are connected either to `bumpybot` AP when in Hotspot mode (default) or to the same wireless network as the Bumpybot computer if in Wifi mode.

### Connection

When opening NoMachine, you should see all the available NoMachine servers on your network as well as any past connections you have saved. Choose the connection labeled: "bumpybot, Ubuntu 18.04.4 LTS" and enter the user `hcrl-bumpybot` and password as prompted.


<img src="nomachine_instructions.png " width="650">


Upon connection, you may be prompted with: “Cannot detect any display running. Do you want NoMachine to create a new display and proceed to connect to the desktop?” 

Confirm the new display creation and check the box that sets that as default behaivor.

You should be prompted with various display, audio, and connection settings, set per your preferences, and you should now be presented with the a remote desktop enviorment
