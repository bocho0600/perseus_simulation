cmake_minimum_required(VERSION 3.10)
project(perseus_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf ros2_control rviz meshes launch data
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/bucket_ram_follower.py scripts/joint_states_deg.py
                  scripts/livox_pattern_sampler.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
