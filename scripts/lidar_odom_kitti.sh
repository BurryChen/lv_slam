#!/bin/bash

# 1.arg
echo "path_arg : $1"

durations=(471 115 484 83 29 288 115 115 423 165 125)

# 2.SLAM, write odom file

#for seq in 00 01 02 03 04 05 06 07 08 09 10  #using s2k
for seq in 01
do

file=${1}/data/KITTI_${seq}_odom.txt
echo $seq $file
gnome-terminal -x bash -c "echo $seq;roslaunch LVO ndt_odom_kitti.launch odom_file:=$file &sleep 10s;rosbag play --clock /media/whu/HD_CHEN_2T/02data/KITTI_odometry/velobag/velo_${seq}.bag -r 1;echo $seq over&&sleep 20s;exit"
i=10#$seq
time=`expr 60 + ${durations[i+1]} \* 10 / 10`
echo $time s
sleep $time
#wait

file_gt=/home/whu/data/data_source_KITTI/gt/${seq}.txt
file_pdf=${1}/data/KITTI_${seq}_odom.pdf
evo_traj kitti $file      --plot_mode=xz  --ref=$file_gt  --save_plot $file_pdf

# 3.eva
cd '/home/whu/data/data_source_KITTI/devkit_old/cpp' 
./evaluate_odometry_seq ${1} $seq
cd ~

done 

# 4.error png
#python '/home/whu/slam_ws/src/LVO/scripts/error_odom_png.py' ${1}
