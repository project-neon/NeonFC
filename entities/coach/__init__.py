from entities.coach.coach import BaseCoach

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.Backup import Coach as IRON_2025

from entities.coach.rsm2025_attack import Coach as RSM_2025_Attack

from entities.coach.rsm2025_defend import Coach as RSM_2025_Defend

_coach_list = [
    # Tournament coaches
    GuideCoach,
    TestCoach,
    IRON_2025,
    RSM_2025_Attack,
    RSM_2025_Defend
]

COACHES = {c.NAME: c for c in _coach_list}
