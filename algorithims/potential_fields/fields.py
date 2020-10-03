import random
import numpy as np

import commons

MIN_WEIGHT_ACTIVE = 0.0

def call_or_return(func, context):
    if callable(func):
        return func(context)
    return func

def apply_decay(decay_fn, value):
    if decay_fn is None:
        return value
    
    out = decay_fn(abs(value))

    return out if out >= 0 else -out


class PotentialField(object):
    def __init__(self, match, **kwargs):
        self.match = match
        self.name = kwargs.get('name', '{}|{}'.format(self.__class__, random.random() * 10000))
        self.weight = kwargs.get('weight', 1)
        self.output = None
        self.field_childrens = []


    def add_field(self, field):
        self.field_childrens.append(field)


    def compute (self, input):
        output_sum = [0, 0] # velocity x, velocity y

        output_sum_weight = 0

        for field in self.field_childrens:
            weight = field.weight
            output = field.compute(input)
            self.output = output
            
            output_sum_weight += weight

            output_sum[0] += output[0] * min(1, max(0, weight))
            output_sum[1] += output[1] * min(1, max(0, weight))

        if output_sum_weight < MIN_WEIGHT_ACTIVE:
            output_sum = (0, 0)

        self.output = output_sum

        return output_sum

class PointField(PotentialField):
    def __init__(self, match, **kwargs):
        super().__init__(match, **kwargs)
        self.target = kwargs['target']
        self.decay = kwargs['decay']
        self.radius = kwargs.get('radius', kwargs.get('radius_max'))
        self.radius_max = kwargs.get('radius_max')
        self.multiplier = kwargs.get('multiplier', 1)

    def compute(self, input):
        target_go_to = call_or_return(self.target, self.match)
        radius_max = call_or_return(self.radius_max, self.match)
        multiplier = call_or_return(self.multiplier, self.match)

        to_target = np.subtract(target_go_to, input)
        to_taget_scalar = np.linalg.norm(to_target)


        if radius_max and to_taget_scalar > radius_max:
            return (0, 0)

        to_target_norm = commons.math.unit_vector(to_target)

        to_target_scalar_norm = max(0, min(1, to_taget_scalar/self.radius))

        force = apply_decay(self.decay, to_target_scalar_norm)

        return (
            to_target_norm[0] * force * multiplier,
            to_target_norm[1] * force * multiplier
        )

