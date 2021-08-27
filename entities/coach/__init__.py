from entities.coach.coach import BaseCoach

from entities.coach.larc2020 import Coach as LarcCoach
from entities.coach.iron2021 import Coach as IronCoach
from entities.coach.experiment_astar import Coach as AstarCoach
from entities.coach.coach_newGK import Coach as NGKCoach
from entities.coach.guideCoach import Coach as GuideCoach

_coach_list = [
    LarcCoach,
    IronCoach,
    AstarCoach,
    NGKCoach,
    GuideCoach
]


COACHES = {c.NAME: c for c in _coach_list}
