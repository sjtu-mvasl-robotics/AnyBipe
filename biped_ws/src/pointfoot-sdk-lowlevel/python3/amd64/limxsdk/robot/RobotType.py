"""
@brief This file contains the declarations of types of robots.

@file RobotType.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

from enum import Enum

class RobotType(Enum):
    """
    An enumeration of different types of robots.
    """
    PointFoot   = 1  # Robot with point foot design.
    Wheellegged = 2  # Robot with wheel-legged design.