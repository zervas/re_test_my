#  R**Y Robotics - Software Engineering Test Task 

This is a basic code that included the basic components that are described with the accompanied document.

An attempt to test all the code with two robots visualized on RViz was made, but it was decided that its complexity should be out of the scope of this test.

## Test it!

After you cloned the repo you should be able to build it from your catkin workspace:

```bash
cd ~/catkin_ws
catkin build remy
```

To run the test you will need three terminals. To the first one:

```bash
roslaunch remy test.launch
```

The above command will run the "Cook" and the "ObjectDetection" node.

On the second terminal, you can listen to any topic from the actions: For example:

```bash
rostopic echo /joint_space_planner/feedback
```

And finally on the third terminal you can run the Chef which is a simple call to the Pick action:

```bash
rosrun remy chef
```