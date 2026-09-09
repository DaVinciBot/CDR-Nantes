#!/usr/bin/env python3
"""Robot utilities: simplified imports.

Exposes the robot_context API: mode selection and Com link creation.
"""

from .robot_context import (
    create_com,
    get_com_class,
    get_com_config,
    get_mode,
    init_robot,
)

__all__ = [
    'create_com',
    'get_com_class',
    'get_com_config',
    'get_mode',
    'init_robot'
]
