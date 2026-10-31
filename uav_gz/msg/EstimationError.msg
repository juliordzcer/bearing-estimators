# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# bebop_gz/msg/EstimationError.msg
#
# Represents estimation error metrics for a specific observer-target pair
#

std_msgs/Header header

int32 observer_id          # ID of the observer robot
int32 target_id           # ID of the target robot

# Position errors
float64 position_error     # Euclidean distance error (m)
float64 position_error_x   # X-axis error (m)
float64 position_error_y   # Y-axis error (m)
float64 position_error_z   # Z-axis error (m)

# Bearing vector errors
float64 bearing_error      # Angular error (radians)
float64 bearing_error_cos  # Cosine similarity (1 - dot_product)

# Covariance metrics
float64[36] position_covariance  # Position covariance matrix (6x6 row-major)
float64 uncertainty_radius        # 3D uncertainty radius (sqrt(trace(covariance[0:3,0:3])))

# Statistical information
float64 innovation_norm   # Innovation vector norm
float64 mahalanobis_distance  # Normalized innovation distance
bool filter_initialized   # Whether the filter is initialized
int32 lost_frames_count   # Number of frames without detection

builtin_interfaces/Time timestamp