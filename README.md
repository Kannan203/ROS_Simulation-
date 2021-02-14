# ROS_Simulation-
Object Oriented Programming


Autonomous Mobile Robot MINI PROJECT

My repository contains -

    kn_192148_mini package

    mini_project_master

PREREQUISITES

    Launch gazebo turtebot3 in empty world.

    Rosrun the spawn model.

    Launch goal_publisher package.

GENERAL DESCRIPTION

    My robot is set into a world full of obstacles.

    The task is to create a package that, when launched, will navigate the turtlebot to as many different goal positions as possible that are published on a topic called /goals.

    I am subscribing to /gazebo/model_states for getting current position of robot and /scan for getting laser scanner data.

ALGORITHM DESCRIPTION

    The concept of master and slave is being used in this algorithm.

    There is a master program and there are 2 other states which programmed to make robot move to goal points and avoid obstacle on the go.

    The master controls the 2 states(moving to goal,avoiding obstacles) via Service.

IMPLEMENTATION

    For this specific algorithm 3 classes were created Master_control(Main Prog), S1_reaching_goals(state1), S2_avoiding_obstacles(state2).

    State 2 programmed to make the turtle bot go around the wall, which in turn has 3 states, 1st state to find wall,2nd state to turn left and then the final state is to follow the wall.

    State 1 programmed to make the turtle bot to move towards the goal points. Here in State 1 the linear velocity, the angle of rotation and also angular velocity are calculated and defined.

    Main program or the Master_control, programmed to switch between state 1 and state 2 based on conditions such goals points and how far the bot is away from its original path.

    After reaching the goal point the turtebot stops and then goes to the next goal point.

    This algorithm follows recursively till all 20 goal points are reached and then the turtlebot stops.

PROBLEMS AND SOLUTIONS

    The major problem faced was syncing the goal points from master to other two states, which was solved by creating a own service and publishing the goal point into it.

Author Kannan Neelamegam
