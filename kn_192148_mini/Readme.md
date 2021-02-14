**AMR MINI PROJECT**

My repository contains -

1. kn_192148_mini package

2. mini_project_master

**PREREQUISITES**

1. Launch gazebo turtebot3 in empty world.

2. Rosrun the spawn model.

3. Launch goal_publisher package.

**GENERAL DESCRIPTION**

1. My robot is set into a world full of obstacles.

2. The task is to create a package that, when launched, will navigate the turtlebot to as many different goal positions as possible that are published on a topic called /goals.

3. I am subscribing to /gazebo/model_states for getting current position of robot and /scan for getting laser scanner data.

**ALGORITHM DESCRIPTION**

1.  The concept of master and slave is being used in this algorithm.

2. There is a master program and there are 2 other states which programmed to make robot move to goal points and avoid obstacle on the go.

3. The master controls the 2 states(moving to goal,avoiding obstacles) via Service.

**IMPLEMENTATION**

1. For this specific algorithm 3 classes were created Master_control(Main Prog), S1_reaching_goals(state1), S2_avoiding_obstacles(state2).

2. State 2 programmed to make the turtle bot go around the wall, which in turn has 3 states, 1st state to find wall,2nd state to turn left and then the final state is to follow the wall.

3. State 1 programmed to make the turtle bot to move towards the goal points. Here in State 1 the linear velocity, the angle of rotation and also angular velocity are calculated and defined.

4. Main program or the Master_control, programmed to switch between state 1 and state 2 based on conditions such goals points and how far the bot is away from its original path. 

5. After reaching the goal point the turtebot stops and then goes to the next goal point.

6. This algorithm follows recursively till all 20 goal points are reached and then the turtlebot stops.

**PROBLEMS AND SOLUTIONS**

1. The major problem faced was syncing the goal points from master to other two states, which was solved by creating a own service and publishing the goal point into it.

**Author**
Kannan Neelamegam 

 




