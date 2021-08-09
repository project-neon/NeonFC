from entities.coach.coach import BaseCoach

from entities.coach.larc2020 import Coach as LarcCoach
from entities.coach.iron2021 import Coach as IronCoach
from entities.coach.experiment_astar import Coach as AstarCoach

from entities.coach.stratch_alex import Coach as AlexCoach

_coach_list = [
    # Tournament coaches
    LarcCoach,
    IronCoach,

    # Example/Test/Debug coaches
    AstarCoach,

    # Devtest coaches
    AlexCoach
]

COACHES = {c.NAME: c for c in _coach_list}
