# CMP3103M Braitenberg Maze Solver

## -Description-

A ROS based maze solver for use with the Turtlebot, using combinations of Braitenberg vehicle behaviours and
OpenCV based computer vision. 
This Turtlebot controller will navigate a maze, sticking to the left wall, avoiding red obstacles and racing towards
blue and green goals.

## -To Run-

Make sure rospy is installed on the PC. Open the appropriate Gazebo simulation featuring a Turtlebot platform (in my case, maze1.launch), and run the 'braitenbergSolver.py' file. 
In my case, this is run in a venv using the PyCharm IDE, however using "python braitenbergSolver.py" from the terminal will also work.

## -Known Issues-

If the program does not run on the first launch, try again even if there are errors. Due to the way rospy handles some publish/subscribe events, the software can occasionally fail to start when first run. 
