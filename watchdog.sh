# ! /bin/bash
# gnome-terminal -e ' bash -c "/home/nuc12/sh/watchdog.sh" '
# gnome-terminal -- bash -c "sh /home/nuc12/sh/watchdog.sh; exec bash"

echo "nuc12" | sudo -S chmod 777 /dev/bus/usb/003/*

gnome-terminal -e 'bash -c "source /opt/intel/openvino_2023.0.2/setupvars.sh;source /opt/ros/humble/setup.bash;source /home/nuc12/Desktop/sentry_autoaim/install/setup.bash;cd /home/nuc12/Desktop/sentry_autoaim;ros2 launch rmos_bringup left_aim.launch.py;" '
gnome-terminal -e 'bash -c "source /opt/intel/openvino_2023.0.2/setupvars.sh;source /opt/ros/humble/setup.bash;source /home/nuc12/Desktop/sentry_autoaim/install/setup.bash;cd /home/nuc12/Desktop/sentry_autoaim;ros2 launch rmos_bringup right_aim.launch.py " '

sleep 3

while true
do
    target_l_hz=$(timeout 10 ros2 topic hz /target_l 2>&1 | grep -oP 'average rate: \K[0-9.]+' || echo "0")
    target_r_hz=$(timeout 10 ros2 topic hz /target_r 2>&1 | grep -oP 'average rate: \K[0-9.]+' || echo "0")

    cam_pid_l=$(pgrep -f "daheng_camera --ros-args -r __node:=daheng_camera_l")
    cam_pid_r=$(pgrep -f "daheng_camera --ros-args -r __node:=daheng_camera_r")
    detector_pid_l=$(pgrep -f "basic_detector --ros-args -r __node:=basic_detector_l")
    detector_pid_r=$(pgrep -f "basic_detector --ros-args -r __node:=basic_detector_r")
    processer_pid_l=$(pgrep -f "processer --ros-args -r __node:=processer_l")
    processer_pid_r=$(pgrep -f "processer --ros-args -r __node:=processer_r")
    comm_pid_l=$(pgrep -f "usb_comm --ros-args -r __node:=usb_comm_l")
    comm_pid_r=$(pgrep -f "usb_comm --ros-args -r __node:=usb_comm_r")
    bringup_pid_l=$(pgrep -f "rmos_bringup left_aim.launch.py")
    bringup_pid_r=$(pgrep -f "rmos_bringup right_aim.launch.py")

    # 检查左头进程和话题
    if [ -n "$cam_pid_l" ] && [ -n "$detector_pid_l" ] && [ -n "$processer_pid_l" ] && [ -n "$comm_pid_l" ] && awk -v hz="$target_l_hz" 'BEGIN{ exit !(hz > 0) }'
    then
        echo `date`
        echo "left_pid is exitable and topic is publishing!"
    else
        echo `date` >> ~/sh/111.txt
        echo "Left side issue detected - restarting nodes"
        
        echo "nuc12" | sudo -S kill -9 $cam_pid_l
        echo "nuc12" | sudo -S kill -9 $detector_pid_l
        echo "nuc12" | sudo -S kill -9 $processer_pid_l
        echo "nuc12" | sudo -S kill -9 $comm_pid_l
        echo "nuc12" | sudo -S kill -9 $bringup_pid_l

        echo "nuc12" | sudo -S chmod 777 /dev/bus/usb/003/*
        gnome-terminal -e 'bash -c "source /opt/intel/openvino_2023.0.2/setupvars.sh;source /opt/ros/humble/setup.bash;source /home/nuc12/Desktop/sentry_autoaim/install/setup.bash;cd /home/nuc12/Desktop/sentry_autoaim;ros2 launch rmos_bringup left_aim.launch.py;" '
    fi

    # 检查右头进程和话题
    if [ -n "$cam_pid_r" ] && [ -n "$detector_pid_r" ] && [ -n "$processer_pid_r" ] && [ -n "$comm_pid_r" ] && awk -v hz="$target_r_hz" 'BEGIN{ exit !(hz > 0) }'
    then
        echo `date`
        echo "right_pid is exitable and topic is publishing!"
    else
        echo `date` >> ~/sh/111.txt
        echo "Right side issue detected - restarting nodes"

        echo "nuc12" | sudo -S kill -9 $cam_pid_r
        echo "nuc12" | sudo -S kill -9 $detector_pid_r
        echo "nuc12" | sudo -S kill -9 $processer_pid_r
        echo "nuc12" | sudo -S kill -9 $comm_pid_r
        echo "nuc12" | sudo -S kill -9 $bringup_pid_r

        echo "nuc12" | sudo -S chmod 777 /dev/bus/usb/003/*
        gnome-terminal -e 'bash -c "source /opt/intel/openvino_2023.0.2/setupvars.sh;source /opt/ros/humble/setup.bash;source /home/nuc12/Desktop/sentry_autoaim/install/setup.bash;cd /home/nuc12/Desktop/sentry_autoaim;ros2 launch rmos_bringup right_aim.launch.py " '
    fi

    sleep 3
done

exit 0
