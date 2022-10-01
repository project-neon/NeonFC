# Baseline de estrategias
from strategy.tests.pfGoalKeeper import GoalKeeper
from strategy.tests.pfAttacker import Attacker
from strategy.tests.pfMidFielder import MidFielder

# Para proposito de criação de novas estrategias
from strategy.tests.Idle import Idle
from strategy.tests.pfScratch import Scratch
from strategy.tests.dwaScratch import DwaScratch

from strategy.tests.newGoalKeeper import newGoalKeeper
from strategy.tests.Defender import Defender
from strategy.tests.newMidFielder import newMidFielder

from strategy.tests.midfieldercx import MidFielderSupporter
from strategy.tests.goalkeeper_rcx import GoalKeeperRCX
from strategy.tests.attacker_rcx import SpinnerAttacker

from strategy.tests.astarAttacker import AstarAttacker
from strategy.tests.dijkSeeker import DijkstraSeeker
from strategy.tests.asScratch import Scratch as astarVoronoi
from strategy.tests.asScratch2 import AsScratch2
from strategy.tests.guideAttacker import My_Attacker
from strategy.tests.newAttacker import newAttacker
from strategy.tests.uvf_attacker import Attacker as UVFAttacker

from strategy.tests.PID_attacker import Attacker as PID_Test
from strategy.tests.thales_atacante import Attacker as thales_atacante