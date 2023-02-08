FROM husarion/rviz2:humble-nightly as urdf_builder
RUN apt-get update -y && apt-get install -y ros-$ROS_DISTRO-xacro && \
    source install/setup.bash && \
    xacro /ros2_ws/src/rosbot_ros/rosbot_description/urdf/rosbot.urdf.xacro > /rosbot.urdf && \
    sed -i 's/file:/http:/g' /rosbot.urdf && \
    sed -i 's/\/ros2_ws/localhost:8080\/ros2_ws/g' /rosbot.urdf && \
    xacro /ros2_ws/src/rosbot_xl_ros/rosbot_xl_description/urdf/rosbot_xl.urdf.xacro > /rosbot_xl.urdf && \
    sed -i 's/file:/http:/g' /rosbot_xl.urdf && \
    sed -i 's/\/ros2_ws/localhost:8080\/ros2_ws/g' /rosbot_xl.urdf && \
    # Changing rotation is cause by .stl files in rosbot_ros/rosbot_description. We will change it to the .dae files.
    sed -i 's/rpy=\"1.5707963267948966 0.0 1.5707963267948966\"/rpy=\"0.0 0.0 1.5707963267948966\"/g' /rosbot.urdf

FROM ghcr.io/foxglove/studio:latest
COPY --from=husarion/rviz2:humble-nightly /ros2_ws /src/ros2_ws
COPY --from=urdf_builder /rosbot.urdf /src/rosbot.urdf
COPY --from=urdf_builder /rosbot_xl.urdf /src/rosbot_xl.urdf