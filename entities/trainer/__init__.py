from entities.trainer.circuit_run import CircuitRunTrainer

_trainer_list = [
    # Tournament coaches
    CircuitRunTrainer
]

TRAINERS = {c.NAME: c for c in _trainer_list}

