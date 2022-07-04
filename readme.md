Please follow the steps here:

1.Put the folder of source code into {your_catkin_workspace}/src
2.cd {your_catkin_workspace}
3.catkin_make
4.source {your_catkin_workspace}/devel/setup.bash
5.roslaunch bm_scan bm661.launch

Additional:
1.rosrun rviz rviz

default frame_id: laser
defalut ros topic: /scan