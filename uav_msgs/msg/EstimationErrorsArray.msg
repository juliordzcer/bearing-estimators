# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# uav_msgs/msg/EstimationErrorsArray.msg
#
# An array of EstimationError messages.
#

std_msgs/Header header # Timestamp for the entire array

EstimationError[] errors # A list of individual estimation errors