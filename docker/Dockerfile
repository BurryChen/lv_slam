FROM ros:melodic

#ENV http_proxy http://127.0.0.1:8001
#ENV https_proxy http://127.0.0.1:8001
 
COPY installers /tmp/installers
#RUN bash /tmp/installers/install_minimal_environment.sh
RUN bash /tmp/installers/install_lv_slam.sh

#RUN sed -i "6i source \"/home/chenshoubin/slam_ws/devel/setup.bash\"" /ros_entrypoint.sh

#WORKDIR /

#ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
