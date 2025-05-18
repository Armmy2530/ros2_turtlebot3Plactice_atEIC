# gz ros2 control Installation
sudo apt install ros-humble-gz-ros2-control

# Add sensor in gazebo

## imu

### add this in gazebo world file
```
<plugin filename="libignition-gazebo-imu-system.so"
    name="ignition::gazebo::systems::Imu">
</plugin>
```

### add this in robot urdf
```
 <gazebo reference="${prefix}_imu_link">
			<sensor name="imu_sensor" type="imu">
				<always_on>1</always_on>
				<update_rate>40</update_rate>
				<visualize>false</visualize>
				<topic>imu</topic>
			</sensor>
</gazebo>
```

### brige imu topic
- see detail <https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos>