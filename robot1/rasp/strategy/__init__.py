"""Strategy: the match state machine and the actions it sequences.

Beware: manager.py still holds the hardcoded CDR 2026 stub sequence. The real
sequence depends on the Eurobot 2027 rules.
"""

from .actions import Action, TypeAction
from .manager import StratManager

__all__ = ["Action", "StratManager", "TypeAction"]
