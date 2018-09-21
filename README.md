# trav_nav_indigo_ws
3D Autonomous Navigation 

## How to Install, Compile and Run

### Install V-REP

1. Download version 3.2.2 (tested) from http://coppeliarobotics.com/files/V-REP_PRO_V3_2_2_64_Linux.tar.gz <br />
    You don't need to compile anything. Just extract the files in your V-REP installation folder and you are ready to execute the main launcher (vrep.sh) from there. 

2. Set the environment variable VREP_ROOT_DIR: <br />
    add in your .bashrc the following line <br />
    _export VREP_ROOT_DIR=<here you put the absolute path of your V-REP installation folder (which contains the launcher vrep.sh)>_

### Compile

* Open a terminal
* Then execute
<pre><code class="c">
$ cd ~/trav_nav_indigo_ws
</code></pre>
* Then compile
<pre><code class="c">
$ catkin_make -j8
</code></pre>

### Install the vrep_ugv_plugin Package

Once you have compiled the tradr-simulation stack, you have to copy the lib _~/trav_nav_indigo_ws/devel/lib/libv_repExtRos.so_ in the installation folder VREP_ROOT_DIR (NOTE: this lib enables V-REP to get and parse track velocity command messages)

### Testing the vrep_ugv_simulation Package

* Open a terminal, source the workspace and execute
<pre><code class="c">
$ roslaunch vrep_ugv_simulation vrep_ugv_simulation.launch
</code></pre>

* Press the play button on V-REP

Once you completed one of the above procedures, you should have your simulator running and a small window with title "UGV TeleOp" should appear. Keep the focus on that window (click on it) and you will be able to move the UGV with the arrow keys. By using the keys 'A','S','D','W' you are also able to change the configuration of its flippers in order to climb stairs.

### V-REP with the Path Planner

1. Lauch the UGV simulation on V-REP: open a new terminal, source the workspace and run
<pre><code class="c">
$ roslaunch vrep_ugv_simulation vrep_ugv_simulation.launch
</code></pre>

2. Press the play button on V-REP

3. Run the path planner with its RVIZ interface
   - If you want to run the single-waypoint path planner, open a new terminal and run 
   <pre><code class="c">
   $ roslaunch path_planner sim_main_path_planner_ugv1.launch
   </code></pre>

   On RVIZ, you can set a goal for the planner by using the dedicated interactive marker. The interactive marker (a sphere) will appear over the robot when you launch the path_planner node.

   Move the marker at your desired position (hold left click on it and move it w.r.t. image plane; if you also hold SHIFT button you will change the depth of the marker)
   Then right-click on it and select from the menu the action "Select Goal". If you want to abort the goal once is selected, select the action "Abort Goal" from the same menu.
   Text messages will appear over the marker explaining you what is happening. The marker color will change accordingly:
   - Grey, path planner is waiting for a goal selection
   - Yellow, the path planner is planning
   - Green, a path has been found
   - Red, the path planner could not find a path

   N.B.: a path to the designated goal can be actually computed if the shown traversability map actually "connect" the goal and the robot positions. 
   You may be required to wait few seconds till the traversability node actually complete the traversability map construction from the mapper inputs.

   Once a path has been found, the path planner will publish it towards the trajectory controller which will make the robot automatically follow the path.

   - otherwise, if you want to run the multi-waypoint path planner, open a new terminal and run 
   <pre><code class="c">
   $ roslaunch path_planner sim_main_queue_path_planner_ugv1.launch
   </code></pre>
