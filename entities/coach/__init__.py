from entities.coach.coach import BaseCoach

from entities.coach.larc2020 import Coach as LarcCoach20
from entities.coach.larc2021 import Coach as LarcCoach21
from entities.coach.larc2021_5v5 import Coach as LarcCoach21_5v5

from entities.coach.iron2021 import Coach as IronCoach
from entities.coach.rcx2022 import Coach as RCX_2022
from entities.coach.cbfrs2022 import Coach as CBFRS_2022
from entities.coach.cbfrs2022_2 import Coach as CBFRS_2022_2

from entities.coach.guideCoach import Coach as GuideCoach


_coach_list = [
    # Tournament coaches
    LarcCoach20,
    LarcCoach21,
    LarcCoach21_5v5,
    IronCoach,
    GuideCoach,
    RCX_2022,
    CBFRS_2022,
    CBFRS_2022_2
]

COACHES = {c.NAME: c for c in _coach_list}
