<launch>
<!-- ============================================================
     FASTLIO2 LiDAR-IMU 外参标定 — Agras MID360
     用法: ros2 launch fastlio2 lio_agras_calib_launch.py
     ============================================================ -->

	<arg name="rviz" default="true" />

	<node pkg="fastlio2" exec="lio_node" name="laserMapping" output="screen">
		<param name="config_file" value="$(find-pkg-share fastlio2)/config/agras_calib.yaml"/>
	</node>

	<group if="$(var rviz)">
	<node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share fastlio2)/rviz/fastlio2.rviz" />
	</group>

</launch>
