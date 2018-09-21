#!/bin/bash


#enable startup.sh logging
exec 2> /home/robot/log/startup.sh.log
exec 1>&2
set -x

#screen -d -m -S UGV bash -i -c 'sleep 20 ;roslaunch nifti_drivers_launchers ugv_new_with_arm.launch'
