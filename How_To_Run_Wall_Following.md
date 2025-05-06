Instruction:

	1. Open a new terminal, run "cd ~/catkin_ws/"
	2. Run "source devel/setup.bash"
	3. Run "roslaunch stingray_sim wall_following.launch"
	4. Open a new terminal again, run "cd ~/catkin_ws/"
	5. Run "source devel/setup.bash"
        6. Run "cd src/stingray_sim/scripts/"
	7. To train the robot, run "rosrun stingray_sim q_td.py train"
	8. To test after training, run "rosrun stingray_sim q_td.py test"
