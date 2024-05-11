# import sys
# sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

# import numpy as np
# from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt

# """
# Problem :
#     Determine H `camera_link` to `tool0`

# Solution :
#     H `camera_link` to `tool0` = H `camera_color_optical_frame` to `tool0` @ H `camera_link` to `camera_color_optical_frame`

# Note :
#     If we have H `child` to `parent`, use <ros2 run tf2_ros tf2_echo `parent` `child`>
# """


# # Step 1 : From eye-in-hand calibration, we get H `camera_color_optical_frame` to `tool0`.
# HccoTtool0 = [-0.03192296, -0.09189364,  0.02288404, 0.01883875, 0.00332055, 0.01325577, 0.99972914] #xyz qxqyqzqw
# HccoTtool0 = rbt.conv_t_and_quat_to_h(HccoTtool0[0:3], HccoTtool0[3:7])

# # Step 2 : From extrinsic information, we get H `camera_link` to `camera_color_optical_frame`. Terminal : <ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_link>
# HclTcco = [0.015, 0.000, 0.000, 0.500, -0.498, 0.500, 0.503]
# HclTcco = rbt.conv_t_and_quat_to_h(HclTcco[0:3], HclTcco[3:7])

# # Step 3 : Perform multiplication
# HclTtool0 = HccoTtool0 @ HclTcco
# print(f"> HclTtool0: {HclTtool0}")
# HclTtool0 = rbt.conv_h_to_t_and_quat(HclTtool0)
# print(f"> HclTtool0: {HclTtool0}")