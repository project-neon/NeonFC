from entities.coach.coach import BaseCoach

from entities.coach.larc2020 import Coach as LarcCoach20
from entities.coach.larc2021 import Coach as LarcCoach21
from entities.coach.larc2021_5v5 import Coach as LarcCoach21_5v5

from entities.coach.iron2021 import Coach as IronCoach
from entities.coach.experiment_astar import Coach as AstarCoach
from entities.coach.coach_newGK import Coach as NGKCoach
from entities.coach.coach_newATK import Coach as NATCoach

from entities.coach.guideCoach import Coach as GuideCoach
from entities.coach.stratch_alex import Coach as AlexCoach

from entities.coach.rsimCoach import Coach as RLCoach

_coach_list = [
    # Tournament coaches
    LarcCoach20,
    LarcCoach21,
    LarcCoach21_5v5,
    IronCoach,
    AstarCoach,
    NGKCoach,
    NATCoach,
    GuideCoach,
    AlexCoach,
    RLCoach
]

COACHES = {c.NAME: c for c in _coach_list}
