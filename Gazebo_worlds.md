## Documentation links:

All gazebo worlds are available under this link: 
https://docs.px4.io/main/en/sim_gazebo_gz/worlds \
All px4 vehicles are available under this link:
https://docs.px4.io/main/en/sim_gazebo_gz/vehicles

---
## Make commands:
#### drones:
- `make px4_sitl gz_x500`
- `make px4_sitl gz_x500_vision`
- `make px4_sitl gz_x500_depth`
- `make px4_sitl gz_x500_mono_cam`
- `make px4_sitl gz_x500_mono_cam_down`
- `make px4_sitl gz_x500_lidar_down`
- `make px4_sitl gz_x500_lidar_front`
- `make px4_sitl gz_x500_lidar_2d`
- `make px4_sitl gz_x500_gimbal`
---
### Gazebo worlds:

To generate a gazebo world different than the default one use:\
`make px4_sitl gz_vehiclename_worldname`\
The make commands below will spawn a X500 Quadrotor with Visual Odometry as an example.

- `make px4_sitl gz_x500_vision_default`
- `make px4_sitl gz_x500_vision_baylands`
- `make px4_sitl gz_x500_vision_walls`

Other worlds can be found in the Gazebo documentation.
### planes:
- `make px4_sitl gz_rc_cessna`
- `make px4_sitl gz_advanced_plane`
- `make px4_sitl gz_standard_vtol`
- `make px4_sitl gz_quadtailsitter`
- `make px4_sitl gz_tiltrotor`
---
### land vehicles:
- `make px4_sitl gz_rover_differential`
- `make px4_sitl gz_rover_ackermann`
- `make px4_sitl gz_rover_mecanum`
