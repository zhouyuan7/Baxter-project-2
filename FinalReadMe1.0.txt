Step1:
enable Baxter

Step2:
Both right and left limbs move to neutral configurations.

Step3:
In a new terminal
cd code/ros_ws
./baxter.sh
rosrun baxter_examples TargetCoordinate1.py
rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
rosrun baxter_examples LeftMainLimb.py

Step4:
Move to next workstation
In a new terminal
cd code/ros_ws
./baxter.sh
rosrun baxter_examples RightLimb.py

Step5:
Move to next workstation
In a new terminal
cd code/ros_ws
./baxter.sh
rosrun baxter_examples TargetCoordinate.py

Go back to the last two workstations to see the back projection profermances (the difference between the red and left point)

If Both good
In a new terminal
cd code/ros_ws
./baxter.sh
rostopic echo /coordinate_real
You can see the estimation of the real ball position

Move to next workstation
open Motion.py file in the baxter_examples folder
paste the estimation position to the data variable in the main function

Step6:
cd code/ros_ws
./baxter.sh
rosrun baxter_examples Motion.py

If the performance is good then
rosrun baxter_examples TargetCoordinate1.py

