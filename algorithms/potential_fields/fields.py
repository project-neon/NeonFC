import random
import json
import math
import numpy as np
import commons

MIN_WEIGHT_ACTIVE = 0.0

def call_or_return(func, match_context, robot_id=-1):
    if callable(func):
        return func(match_context)
    return func

def apply_decay(decay_fn, value):
    if decay_fn is None:
        return value

    out = decay_fn(abs(value))

    return out if value >= 0 else -out

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

        self.field_limits = kwargs.get('field_limits', None)

    def compute(self, input):
        target_go_to = call_or_return(self.target, self.match)
        radius_max = call_or_return(self.radius_max, self.match)
        multiplier = call_or_return(self.multiplier, self.match)

        to_target = np.subtract(target_go_to, input)
        to_taget_scalar = np.linalg.norm(to_target)

        if self.field_limits and not(0 <= input[0] <= self.field_limits[0]):
            return (0, 0)
        
        if self.field_limits and not(0 <= input[1] <= self.field_limits[1]):
            return (0, 0)

        if radius_max and to_taget_scalar > radius_max:
            return (0, 0)

        to_target_norm = commons.math.unit_vector(to_target)

        to_target_scalar_norm = max(0, min(1, to_taget_scalar/self.radius))

        force = apply_decay(self.decay, to_target_scalar_norm)

        return (
            to_target_norm[0] * force * multiplier,
            to_target_norm[1] * force * multiplier
        )

class LineField(PotentialField):
    def __init__(self, match, **kwargs):
        super().__init__(match, **kwargs)
        self.target = kwargs['target']
        self.decay = kwargs['decay']

        self.multiplier = kwargs.get('multiplier', 1)
        
        # line definition
        self.theta = kwargs['theta']
        self.line_size = kwargs['line_size']
        self.line_dist = kwargs['line_dist']
        self.line_dist_max = kwargs.get('line_dist_max')

        self.line_size_single_side = kwargs.get('line_size_single_side', False)
        self.line_dist_single_side = kwargs.get('line_dist_single_side', False)

        self.inverse = kwargs.get('inverse', False)

        self.field_limits = kwargs.get('field_limits', None)

    def compute(self, input):
        target_line = call_or_return(self.target, self.match)
        target_theta = call_or_return(self.theta, self.match)

        multiplier = call_or_return(self.multiplier, self.match)
        line_dist_max = call_or_return(self.line_dist_max, self.match)

        to_line = np.subtract(target_line, input)
        to_line_with_theta = commons.math.rotate_via_numpy(to_line, -target_theta)

        if self.field_limits and not(0 <= input[0] <= self.field_limits[0]):
            return (0, 0)
        
        if self.field_limits and not(0 <= input[1] <= self.field_limits[1]):
            return (0, 0)

        if self.line_size and abs(to_line_with_theta[0]) > self.line_size:
            return (0, 0)

        if self.line_size_single_side and to_line_with_theta[0] < 0:
            return (0, 0)

        if self.line_dist_max and abs(to_line_with_theta[1]) > line_dist_max:
            return (0, 0)

        if self.line_dist_single_side and to_line_with_theta[1] < 0 and not self.inverse:
            return(0, 0)
        
        if self.line_dist_single_side and to_line_with_theta[1] > 0 and self.inverse:
            return(0, 0)

        to_line_norm = commons.math.unit_vector(
            commons.math.rotate_via_numpy(
                [0, to_line_with_theta[1]],
                target_theta
            )
        )

        to_line_scalar_norm = max(0, min(1, abs(to_line_with_theta[1]/self.line_dist)))

        force = apply_decay(self.decay, to_line_scalar_norm)

        return (
            to_line_norm[0] * force * multiplier,
            to_line_norm[1] * force * multiplier
        )

class TangentialField(PotentialField):
    def __init__(self, match, **kwargs):
        super().__init__(match, **kwargs)
        self.target = kwargs['target']
        self.clockwise = kwargs.get('clockwise', False)
        self.decay = kwargs['decay']
        self.radius = kwargs.get('radius', kwargs.get('radius_max'))
        self.radius_max = kwargs.get('radius_max')
        self.multiplier = kwargs.get('multiplier', 1)
        self.orbitation_speed = kwargs.get('orbitation_speed', self.multiplier)

        self.K = kwargs.get('K', 1/25000)

        self.field_limits = kwargs.get('field_limits', None)
    
    def compute(self, input, robot_id=-1):
        target_go_to = call_or_return(self.target, self.match, robot_id)
        radius_max = call_or_return(self.radius_max, self.match, robot_id)
        multiplier = call_or_return(self.multiplier, self.match, robot_id)

        cwo = 1 if call_or_return(self.clockwise, self.match, robot_id) else -1 # clockwise ou counterclockwise

        to_target = np.subtract(target_go_to, input)
        to_taget_scalar = np.linalg.norm(to_target)
        
        angle_to_target = math.atan2(target_go_to[1] - input[1], target_go_to[0] - input[0] )

        if self.field_limits and not(0 <= input[0] <= self.field_limits[0]):
            return (0, 0)
        
        if self.field_limits and not(0 <= input[1] <= self.field_limits[1]):
            return (0, 0)

        if radius_max and to_taget_scalar > radius_max:
            return (0, 0)

        to_target_scalar_norm = max(0, min(1, abs((self.radius - to_taget_scalar)/radius_max)))
        end_angle = 0
        if to_taget_scalar > self.radius:
            end_angle = angle_to_target + cwo * (math.pi/2) * (2 - ( (self.radius + self.K)/(to_taget_scalar + self.K) ))
        else:
            end_angle = angle_to_target + cwo * (math.pi/2) * math.sqrt(to_taget_scalar/self.radius)

        to_target_norm = -commons.math.unit_vector( (math.cos(end_angle), math.sin(end_angle)) )

        force = apply_decay(self.decay, to_target_scalar_norm)

        return (
            to_target_norm[0] * force * multiplier,
            to_target_norm[1] * force * multiplier
        )
