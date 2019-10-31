#!/bin/bash

# 1.arg
echo "path_arg : $1"

durations=(471 115 484 83 29 288 115 115 423 165 125     96 110 342 66 198 180 52 187 517 87 282)

# 2.SLAM, write odom file

#for seq in 11 12 13 14 15 16 17 18 19 20 21 
#for seq in 03 04 05 06 07 08 09 10 02 00
for seq in 04  #use_angle_calibration=true
do

logfile=${1}/data/${seq}.log
echo $logfile
touch $logfile

file_odom=${1}/data/KITTI_${seq}_odom.txt
echo $1 $seq $file_odom
#gnome-terminal -x bash -c "echo $seq;roslaunch lvo ndt_odom_kitti.launch odom_file:=$file_odom &sleep 10s;rosbag play --clock /media/whu/HD_CHEN_2T/02data/KITTI_odometry/velobag/velo_${seq}.bag -r 1.0;echo $seq over&&sleep 20s;exit"
gnome-terminal -x zsh -c "echo $seq;roslaunch lv_slam odom_kitti.launch res_dir:=$1 seq:=${seq} >$logfile &sleep 5s;rosbag play --clock /home/whu/data/data_source_KITTI/velostereobag/velo_img_${seq}.bag -r 1.0;echo $seq over&&sleep 25s;exit"
i=10#$seq
time=`expr 60 + ${durations[i]} \* 10 / 09`
echo $time s
sleep $time 
wait

file_gt=/home/whu/data/data_source_KITTI/gt/${seq}.txt
#file_gt=/media/whu/Research/04Dissertation/03LiDAROdometry/NDT_LO/${seq}.txt
file_pdf=${1}/data/KITTI_${seq}_odom.pdf
evo_traj kitti $file_odom      --plot_mode=xz  --ref=$file_gt  --save_plot $file_pdf


file_scan_error=${1}/errors/KITTI_${seq}_scan_error.txt
file_pdf2=${1}/errors/KITTI_${seq}_scan_error.pdf
evo_traj kitti $file_scan_error   --plot_mode=yx --save_plot $file_pdf2

# 3.eva
cd '/home/whu/data/data_source_KITTI/devkit_old/cpp' 
./evaluate_odometry_seq ${1} $seq
cd ~

done 

# 4.error png
#python '/home/whu/slam_ws/src/LVO/scripts/error_odom_png.py' ${1}
