from entities.coach.coach import BaseCoach

from entities.coach.larc2020 import Coach as LarcCoach20
from entities.coach.larc2021 import Coach as LarcCoach21
from entities.coach.larc2021_5v5 import Coach as LarcCoach21_5v5
from entities.coach.iron2022_3v3 import Coach as IronCoach22_3v3
from entities.coach.larc2022_5v5 import Coach as LarcCoach22_5v5
from entities.coach.iron2023_3v3 import Coach as IronCoach23_3v3

from entities.coach.iron2021 import Coach as IronCoach
from entities.coach.rcx2022 import Coach as RCX_2022
from entities.coach.cbfrs2022 import Coach as CBFRS_2022
from entities.coach.cbfrs2022_2 import Coach as CBFRS_2022_2
from entities.coach.cbfrs2022_5v5 import Coach as CBFRS_2022_5V5

from entities.coach.guideCoach import Coach as GuideCoach

from entities.coach.test_coach import Coach as TestCoach

_coach_list = [
    # Tournament coaches
    LarcCoach20,
    LarcCoach21,
    LarcCoach21_5v5,
    IronCoach22_3v3,
    LarcCoach22_5v5,
    IronCoach23_3v3,
    IronCoach,
    GuideCoach,
    RCX_2022,
    CBFRS_2022,
    CBFRS_2022_2,
    CBFRS_2022_5V5,
    TestCoach
]

COACHES = {c.NAME: c for c in _coach_list}
