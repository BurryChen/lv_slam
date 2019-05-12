#!/usr/bin/python
import tf
import numpy as np

import matplotlib.pyplot as plt
from pylab import *

import imp
transformations=imp.load_source('transformations','/home/whu/slam_ws/src/test_slam/scripts/transformations.py')
import transformations

PI=3.14159262728
odom_dir=sys.argv[1]
#seq=sys.argv[2]
seqs=['00','01','02','03','04','05','06','07','08','09','10']
#seqs=['00']
for seq in seqs:
 file_odom=odom_dir+'/data/KITTI_'+seq+'_odom.txt'
 file_gt='/home/whu/data/data_source_KITTI/gt/'+seq+'.txt'
 odom = np.loadtxt(file_odom)   
 gt = np.loadtxt(file_gt)   
 error=odom-gt

 dx = [x[3] for x in error]  
 dy = [x[7] for x in error] 
 dz = [x[11] for x in error] 
 error_odom=np.eye(len(odom),6)
 error_s2s=np.eye(len(odom),6)
 for i in range(1,len(odom)):
     RT_odom=np.mat(((odom[i][0], odom[i][1], odom[i][2], odom[i][3]),
		       (odom[i][4], odom[i][5], odom[i][6], odom[i][7]),
		       (odom[i][8], odom[i][9], odom[i][10], odom[i][11]),
		       (        0,          0,          0,          1)), dtype=np.float64)
     x_odom,y_odom,z_odom=transformations.translation_from_matrix(RT_odom)
     al_odom, be_odom, ga_odom = transformations.euler_from_matrix(RT_odom, 'szyx')    
 
     RT_gt=np.mat(((gt[i][0], gt[i][1], gt[i][2], gt[i][3]),
		     (gt[i][4], gt[i][5], gt[i][6], gt[i][7]),
		     (gt[i][8], gt[i][9], gt[i][10], gt[i][11]),
		     (       0,          0,          0,          1)), dtype=np.float64)
     
     x_gt,y_gt,z_gt=transformations.translation_from_matrix(RT_gt) 
     al_gt, be_gt, ga_gt = transformations.euler_from_matrix(RT_gt, 'szyx')     
     error_odom[i,0:6]=[x_odom-x_gt,y_odom-y_gt,z_odom-z_gt,al_odom-al_gt,be_odom-be_gt,ga_odom-ga_gt]
     #print error_odom[i,0:6]

     RT_pre_odom=np.mat(((odom[i-1][0], odom[i-1][1], odom[i-1][2], odom[i-1][3]),
		       (odom[i-1][4], odom[i-1][5], odom[i-1][6], odom[i-1][7]),
		       (odom[i-1][8], odom[i-1][9], odom[i-1][10], odom[i-1][11]),
		       (        0,          0,          0,          1)), dtype=np.float64)
     RT_pre_gt=np.mat(((gt[i-1][0], gt[i-1][1], gt[i-1][2], gt[i-1][3]),
		     (gt[i-1][4], gt[i-1][5], gt[i-1][6], gt[i-1][7]),
		     (gt[i-1][8], gt[i-1][9], gt[i-1][10], gt[i-1][11]),
		     (         0,          0,          0,          1)), dtype=np.float64) 
     RT_s2s_odom=RT_pre_odom.I*RT_odom
     x_s2s_odom,y_s2s_odom,z_s2s_odom=transformations.translation_from_matrix(RT_s2s_odom)
     al_s2s_odom, be_s2s_odom, ga_s2s_odom = transformations.euler_from_matrix(RT_s2s_odom, 'szyx') 
     
     RT_s2s_gt=RT_pre_gt.I*RT_gt
     x_s2s_gt,y_s2s_gt,z_s2s_gt=transformations.translation_from_matrix(RT_s2s_gt) 
     al_s2s_gt, be_s2s_gt, ga_s2s_gt = transformations.euler_from_matrix(RT_s2s_gt, 'szyx')  
     
     error_s2s[i,0:6]=[x_s2s_odom-x_s2s_gt,y_s2s_odom-y_s2s_gt,z_s2s_odom-z_s2s_gt,
		al_s2s_odom-al_s2s_gt,be_s2s_odom-be_s2s_gt,ga_s2s_odom-ga_s2s_gt]
     #print error_s2s[i,0:6]
     
     for j in range(3,6):
        if (error_odom[i,j]>PI): error_odom[i,j]-=PI
        if (error_odom[i,j]<-PI): error_odom[i,j]+=PI
        if (error_s2s[i,j]>PI): error_s2s[i,j]-=PI
        if (error_s2s[i,j]<-PI): error_s2s[i,j]+=PI
        
 plt.figure(int(seq)*4+0)
 plt.plot(error_odom[:,0],"r.")
 plt.plot(error_odom[:,1],"g.")
 plt.plot(error_odom[:,2],"b.")
 plt.title('seq '+seq+' odom_t:r-x-right,g-y-down,b-z-front')
 fig=odom_dir+'/errors/'+'error_'+seq+'_odom_t.png'
 savefig(fig)
 print fig
 
 plt.figure(int(seq)*4+1)
 plt.plot(error_odom[:,3],"r.")
 plt.plot(error_odom[:,4],"g.")
 plt.plot(error_odom[:,5],"b.")
 plt.title('seq '+seq+' odom_r:r-al,g-be,b-ga')
 fig=odom_dir+'/errors/'+'error_'+seq+'_odom_r.png'
 savefig(fig)
 print fig 
  
 #plt.figure(int(seq)*4+2)
 #plt.plot(error_s2s[:,0],"r.")
 #plt.plot(error_s2s[:,1],"g.")
 #plt.plot(error_s2s[:,2],"b.")
 #plt.title('seq '+seq+' s2s_t:r-x-right,g-y-down,b-z-front')
 #plt.ylim(-0.2,0.2)
 #fig=odom_dir+'/errors/'+'error_'+seq+'_s2s_t.png'
 #savefig(fig)
 #print fig 
 
 #plt.figure(int(seq)*4+3)
 #plt.plot(error_s2s[:,3],"r.")
 #plt.plot(error_s2s[:,4],"g.")
 #plt.plot(error_s2s[:,5],"b.")
 #plt.title('seq '+seq+' s2s_r:r-al,g-be,b-ga')
 #fig=odom_dir+'/errors/'+'error_'+seq+'_s2s_r.png'
 #savefig(fig)
 #print fig 
#plt.show()

