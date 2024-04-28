from entities.coach.coach import BaseCoach

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.attack import Coach as Attack

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.iron2024 import Coach as IRON_2024

_coach_list = [
    # Tournament coaches
    GuideCoach,
    Attack,
    TestCoach,
    IRON_2024
]

COACHES = {c.NAME: c for c in _coach_list}
