# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# bebop_gz/msg/BearingMeasurementsArray.msg
#
# An array of BearingMeasurement messages.
#

std_msgs/Header header # Timestamp for the entire array (e.g., when it was published)
                       # and optional frame_id if all bearings refer to a common frame.

BearingMeasurement[] measurements # A list of individual bearing measurements