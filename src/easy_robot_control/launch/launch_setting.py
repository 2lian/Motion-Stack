import numpy as np

std_movement_time = 1.0  # seconds
movement_update_rate = 20.0  # Hz

class LegParameters:
    """
    blank object holding standard robot parameters
    """
    def __init__(self,
                 body_to_coxa_mm,
                 coxa_length_mm,
                 femur_length_mm,
                 tibia_length_mm,
                 coxaMax_degree,
                 coxaMin_degree,
                 femurMax_degree,
                 femurMin_degree,
                 tibiaMax_degree,
                 tibiaMin_degree,
                 ):
        self.bodyToCoxa = float(body_to_coxa_mm)
        self.coxaLength = float(coxa_length_mm)
        self.femurLength = float(femur_length_mm)
        self.tibiaLength = float(tibia_length_mm)

        self.coxaMax = float(np.deg2rad(coxaMax_degree))
        self.coxaMin = float(np.deg2rad(coxaMin_degree))
        self.femurMax = float(np.deg2rad(femurMax_degree))
        self.femurMin = float(np.deg2rad(femurMin_degree))
        self.tibiaMax = float(np.deg2rad(tibiaMax_degree))
        self.tibiaMin = float(np.deg2rad(tibiaMin_degree))

        self.minFemurTargetDist = 0.0
        self.update_minFemurTargetDist()

    def update_minFemurTargetDist(self):
        self.minFemurTargetDist = np.abs(self.femurLength + self.tibiaLength * np.exp(1j * self.tibiaMax))


D1 = 0.181  # Distance between Origin of base and origin of the joint1
L1 = 0.283 - D1  # Length between joint1 (Near the base joint) and joint2
L2 = 0.396 - L1 - D1  # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.490 - (L2 + L1 + D1)  # Length between Joint3 and Tip

D1 = 0.181  # Distance between Origin of base and origin of the joint1
L1 = 0.065 # Length between joint1 (Near the base joint) and joint2
L2 = 0.135 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.16  # Length between Joint3 and Tip

moonbot_leg = LegParameters(
    body_to_coxa_mm=float(D1 * 1000),
    coxa_length_mm=float(L1 * 1000),
    femur_length_mm=float(L2 * 1000),
    tibia_length_mm=float(L3 * 1000),
    coxaMax_degree=60,
    coxaMin_degree=-60,
    femurMax_degree=90,
    femurMin_degree=-90,
    tibiaMax_degree=90,
    tibiaMin_degree=-90
)
