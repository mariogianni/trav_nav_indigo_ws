# trav_nav_indigo_ws
3D Autonomous Navigation 

## How to Install, Compile and Run

### Install V-REP

1. Download version 3.2.2 (tested) from http://coppeliarobotics.com/files/V-REP_PRO_V3_2_2_64_Linux.tar.gz <br />
    You don't need to compile anything. Just extract the files in your V-REP installation folder and you are ready to execute the main launcher (vrep.sh) from there. 

2. Set the environment variable VREP_ROOT_DIR: <br />
    add in your .bashrc the following line <br />
    _export VREP_ROOT_DIR=<here you put the absolute path of your V-REP installation folder (which contains the launcher vrep.sh)>_

### Install the vrep_ugv_plugin Package

Once you have compiled the tradr-simulation stack, you have to copy the lib _trav_nav_indigo_ws/devel/lib/libv_repExtRos.so_ in the installation folder VREP_ROOT_DIR (NOTE: this lib enables V-REP to get and parse track velocity command messages)

### Testing the vrep_ugv_simulation Package

* Open a terminal, source the workspace and execute
<pre><code class="c">
$ roslaunch vrep_ugv_simulation vrep_ugv_simulation.launch
</code></pre>

* Press the play button on V-REP

Once you completed one of the above procedures, you should have your simulator running and a small window with title "UGV TeleOp" should appear. Keep the focus on that window (click on it) and you will be able to move the UGV with the arrow keys. By using the keys 'A','S','D','W' you are also able to change the configuration of its flippers in order to climb stairs.
