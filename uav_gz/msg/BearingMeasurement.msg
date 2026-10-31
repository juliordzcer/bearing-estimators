# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# bebop_gz/msg/BearingMeasurement.msg
#
# Represents a single bearing measurement from an observer to a target.
#

int32 observer_id   # ID of the robot observing (e.g., its index in a list)
int32 target_id     # ID of the robot being observed (e.g., its index in a list)

geometry_msgs/Vector3 bearing_vector      # The unit bearing vector g_ij (normalized e_ij)
geometry_msgs/Vector3 bearing_vector_dot  # The projected bearing derivative g_ij_star (P_g * e_dot / ||e_ij||)

float64 distance                          # solvePnP estimated distance from observer to target [m]
                                          # (0.0 when published from a prediction step, no fresh measurement)

builtin_interfaces/Time timestamp         # Timestamp when this specific bearing measurement was made
                                          # (optional but highly recommended for time-critical applications)