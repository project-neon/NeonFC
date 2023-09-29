from entities.coach.coach import BaseCoach

#from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.rsm2023 import Coach as RSM_2023

from entities.coach.rcx2023 import Coach as RCX_2023

from entities.coach.larc2023_G1 import Coach as LARC_2023_G1
from entities.coach.larc2023_G2 import Coach as LARC_2023_G2

_coach_list = [
    # Tournament coaches
    #GuideCoach,
    RSM_2023,
    RCX_2023,
    TestCoach,
    LARC_2023_G1,
    LARC_2023_G2
]

COACHES = {c.NAME: c for c in _coach_list}
