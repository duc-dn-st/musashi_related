# nitrabot
Main launch files are here. Pure-pursuit controller is also found here.

## To run Pure-puresuit Controller
rosrun nitrabot controller_node

# To run SIMULATION
change controller_node2.cpp -> controller_node.cpp
In pure_pursuit.cpp line247 & line257
    // rpm_convert_publish(input);    <----uncomment
    getVfhControlCmd(input);      <-----comment out

For Experiment change vise versa.

