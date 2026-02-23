#!/usr/bin/env python3
"""
Utilitaires pour le robot - Imports simplifiés

Regroupe robot_context et webots_com pour un accès facile.
switch_mode reste dans le dossier parent pour compatibilité.
"""

from .robot_context import (
    is_simulation,
    create_com,
    get_com_config,
    get_config,
    init_robot,
    RobotContext
)

from .webots_com import (
    WebotsComBridge,
    Com
)

__all__ = [
    'is_simulation',
    'create_com',
    'get_com_config',
    'get_config',
    'init_robot',
    'RobotContext',
    'WebotsComBridge',
    'Com'
]


