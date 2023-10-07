from entities.coach.coach import BaseCoach

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.rcx2023 import Coach as RCX_2023

from entities.coach.larc2023_E import Coach as LARC_2023_E

from entities.coach.larc2023_Election import Coach as LARC_2023_CHANGE

_coach_list = [
    # Tournament coaches
    GuideCoach,
    RCX_2023,
    LARC_2023_E,
    TestCoach,
    LARC_2023_CHANGE
]

COACHES = {c.NAME: c for c in _coach_list}
