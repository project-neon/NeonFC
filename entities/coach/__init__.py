from entities.coach.coach import BaseCoach

from entities.coach.iron2022 import Coach as IRON_2022

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.rsm2022 import Coach as RSMCoach

from entities.coach.iron2023 import Coach as IRON_2023

from entities.coach.rsm2023 import Coach as RSM_2023

from entities.coach.rcx2023 import Coach as RCX_2023

_coach_list = [
    # Tournament coaches
    GuideCoach,
    RSMCoach,
    IRON_2022,
    IRON_2023,
    RSM_2023,
    RCX_2023,
    TestCoach
]

COACHES = {c.NAME: c for c in _coach_list}
