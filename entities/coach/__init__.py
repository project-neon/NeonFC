from entities.coach.coach import BaseCoach

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.twoplayers import Coach as TwoPlayers

from entities.coach.test_coach import Coach as TestCoach

from entities.coach.rcx2024 import Coach as RCX_2024

_coach_list = [
    # Tournament coaches
    GuideCoach,
    TwoPlayers,
    TestCoach,
    RCX_2024
]

COACHES = {c.NAME: c for c in _coach_list}
