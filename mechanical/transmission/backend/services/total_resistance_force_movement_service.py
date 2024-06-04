from dataclasses import dataclass

import numpy


@dataclass
class TotalResistanceForceMovementService:
    """Сервис силы сопротивлению движения"""
    angle_array: numpy.array
    full_mass: float

    @property
    def lifting_force(self):
        liftings = []
        for angle in self.angle_array:
            liftings.append(self.full_mass * angle / 100)
        return liftings
