#!/bin/bash

# 创建一个新的 tmux 会话，命名为 "ros_nodes"，并在其中启动第一个窗口运行第一个节点
tmux new-session -d -s ros_nodes -n contact_node "

echo 'please run "sudo chmod 777 /dev/ttyACM0 " first'

echo 'Changing directory...';
cd /home/orangepi/GM/nuc/GMaster_project/src/;

echo 'Sourcing ROS 2...';
source /opt/ros/humble/setup.bash;

echo 'Sourcing sentry_msg...';
source sentry_msgs/install/setup.bash;

echo 'Sourcing contact node...';
source contact/install/setup.bash;

echo 'Running contact node...';
ros2 run contact contact;
exec bash
"

echo "Contact node has been started."

# 在同一个 tmux 会话中创建第二个窗口，运行第二个节点
tmux split-window -h -t ros_nodes "
echo 'Changing directory...';
cd /home/orangepi/GM/nuc/GMaster_project/src/;

echo 'Sourcing ROS 2...';
source /opt/ros/humble/setup.bash;

echo 'Sourcing sentry_msg...';
source sentry_msgs/install/setup.bash;

echo 'Sourcing pubtx node...';
source pubtx/install/setup.bash;

echo 'Running pubtx node...';
ros2 run pubtx pubtx;
exec bash
"

echo "Pubtx node has been started."

# 设置 tmux 窗口布局为左右分屏
tmux select-layout -t ros_nodes even-horizontal

# 启动 tmux 会话以便用户可以查看两个节点的输出
tmux attach-session -t ros_nodes
