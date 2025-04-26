from entities.coach.coach import BaseCoach

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.twoplayers import Coach as TwoPlayers

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.Backup import Coach as IRON_2025

from entities.coach.rsm2025 import Coach as RSM_2025

_coach_list = [
    # Tournament coaches
    GuideCoach,
    TwoPlayers,
    TestCoach,
    IRON_2025,
    RSM_2025
]

COACHES = {c.NAME: c for c in _coach_list}
