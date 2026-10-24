"""
Módulo FlightPlan de NavSim.
Estructuras cinemáticas y gestión de planes de vuelo para UAVs.
"""

from .waypoint import Waypoint
from .flight_plan import FlightPlan

__all__ = ["Waypoint", "FlightPlan"]
