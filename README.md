### Build & Run

1. Connect the IMU

- Plug the IMU device via USB.

2. Build the workspace

       git clone https://github.com/ichsanyudika/razor_imu_rviz2.git
       cd ~/razor_imu_rviz2
       colcon build
       source install/setup.bash

3. Launch

       ros2 launch imu_serial serial_imu_node.py
       rviz2

### Simulation

[Demo Video](https://youtu.be/xpN31UG7EPU?si=GbQ2OC2Z0xJJ8L8F)
