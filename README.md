# Mobile Robotic V2

This is the student repository for the mobile robot course of Polytech Angers


## Setup the programming environment

- Go to the gitlab : gitlab.u-angers.fr
- Search for the **Mobile Robotic v2 Student** repository
- Copy the url in `Code > Cloner avec HTTPS`
- On your PC, go into the T drive (`ThawSpace T:`)
- Open a PowerShell into that drive (`Ctr + L` to open a PowerShell in the current directory)
- In that powerShell, do a git clone command with the previously copied url: 
`PS T:\> git clone https://gitlab.u-angers.fr/cours/mobile_robotic_v2_student.git`
- You should now have a `mobile_robotic_v2_student` directory into your `T` drive
- Get the compressed docker image `ros2_user.tar.gz` and place it into the `mobile_robotic_v2_student/docker directory` (next to the `run_windows_novnc.ps1` script)
- Start the ps1 script into a powershell:
`PS T:\mobile_robotic_v2_student\docker> .\run_windows_novnc.ps1`
- This should:
  - start docker engine
  - load the `ros2_user.tar.gz` image
  - open a noVNC tab in Mozilla Firefox (`127.0.0.1:8080/vnc_auto.html`)
  - this can take few minutes...
- Then open VSCode
- Attach to the running container named `ros2`
  - This should open a new VSCode window inside the container, in the `wdir` directory
  - You should have a `hello.txt` file available in that directory
- Change the content of the `hello.txt` file using VSCode
- Check that the content of the file in your PC has been updated (`T/mobile_robotic_v2_student/wdir/hello.txt`)
- You are all set up! From now you should do all your work using VSCode **in the `wdir` directory**

## Docker directory

Contains all the docker files and scripts needed to set up the working environment

## Course organization

The documents are in the `documents` directory.
### TP1

- [1_1_ros2_introduction](documents/1_1_ros2_introduction.md)
- [1_2_ros2_first_node](documents/1_2_ros2_first_node.md)
- [1_3_ros2_tp1_teleop_turtle](documents/1_3_ros2_tp1_teleop_turtle.md)


### TP2

- [2_1_ros2_launch_file](documents/2_1_ros2_launch_file.md)
- [2_2_ros2_my_action](documents/2_2_ros2_my_action.md)
- [2_3_ros2_tp2_follow_path](documents/2_3_ros2_tp2_follow_path.md)


### TP3

- [3_1_simulating_a_robot](documents/3_1_simulating_a_robot.md)
- [3_2_ros2_gazebo](documents/3_2_ros2_gazebo.md)
- [3_3_gazebo_lidar](documents/3_3_gazebo_lidar.md)
- [3_4_tp3_out_of_box](documents/3_4_tp3_out_of_box.md)

 
