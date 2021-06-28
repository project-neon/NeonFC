import os
import math
import numpy as np
import tensorflow as tf
from tensorflow import keras

from collections import deque
import random

from strategy.BaseStrategy import Strategy
from algorithms.potential_fields.fields import PointField, LineField, TangentialField

STEPS_TO_TRAIN = 60 * 60

MIN_REPLAY_SIZE = 1000

def point_in_rect(point,rect):
    x1, y1, w, h = rect
    x2, y2 = x1+w, y1+h
    x, y = point
    if (x1 < x and x < x2):
        if (y1 < y and y < y2):
            return True
    return False

class DeepPlay(Strategy):
    def __init__(self, match):
        self.match = match
        self.game = self.match.game

        self.actions = [None for _ in range(11)]

        self.action_shape = len(self.actions)
        self.learning_rate = 0.001

        self.train_episodes = 300
        self.train_count = 0
        self.test_episodes = 100
        self.test_count = 0

        self.frames_ran = 0
        self.already_trained = False

        self.epsilon = 1
        self.max_epsilon = 1
        self.min_epsilon = 0.01
        self.decay = 0.005

        self.last_action = None
        self.last_observation = None

        self.replay_memory = deque(maxlen=50000)

        self.target_update_counter = 0
        self.steps_to_update_target_model = 0
        self.total_training_rewards = 0

        self.target_model = None
        self.model = None

        self.model_folder = os.environ.get('DEEP_NEON_MODEL_FOLDER_PREFIX')

        super().__init__(match, 'DeepPlayer')

    def calculate_by_action(self, action):
        return self.actions[action].compute([self.robot.x, self.robot.y])

    def generate_possible_actions(self):
        self.actions = []
        ball = self.match.ball
        robot = self.robot
        field_limits = self.match.game.field.get_dimensions()
        mid_field = [ax/2 for ax in field_limits]
        dist_to_ball = ( (ball.x - robot.x)**2 +  (ball.y - robot.y)**2 )**.5

        def choose_target(s):
            def f(m):
                _sign = s
                return (
                    m.ball.x + _sign * math.cos(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball,
                    m.ball.y + _sign * math.sin(math.atan2((mid_field[1] - m.ball.y), (mid_field[0]*2 - m.ball.x)) + math.pi/2) * 0.4 * dist_to_ball
                )
            return f
        
        # BALL POINT
        self.actions.append(
            PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = 0.75
            )
        )
        self.actions.append(
            PointField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                field_limits = [0.75* 2 , 0.65*2],
                multiplier = -0.75
            )
        )
        # BALL X LINE
        self.actions.append(
            LineField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                theta = 0,
                line_size = 1,
                line_dist = 1,
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )
        # BALL Y LINE
        self.actions.append(
            LineField(
                self.match,
                target = lambda m: (m.ball.x, m.ball.y),
                theta = math.pi,
                line_size = 1,
                line_dist = 1,
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )
        # GOAL POINT
        self.actions.append(
            PointField(
                self.match,
                target = lambda m: (m.game.field.get_dimensions()[0], m.game.field.get_dimensions()[1]/2), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                multiplier = 0.75
            )
        )
        # OUR GOAL POINT
        self.actions.append(
            PointField(
                self.match,
                target = lambda m: (0.05, m.game.field.get_dimensions()[1]/2), # centro do campo
                radius = 0.1, # 30cm
                decay = lambda x: x**2,
                multiplier = 0.75
            )
        )
        # TANGENTIAL LEFT TO BALL
        self.actions.append(
            TangentialField(
                self.match,
                target=choose_target(1),                                                                                                                                                                                                                                                                                                                                          
                radius = dist_to_ball * 0.2,
                radius_max = dist_to_ball * 10,
                clockwise = 1,
                decay=lambda _: 1,
                multiplier = 0.75
            )
        )
        # TANGENTIAL RIGHT TO BALL
        self.actions.append(
            TangentialField(
                self.match,
                target=choose_target(-1),                                                                                                                                                                                                                                                                                                                                          
                radius = dist_to_ball * 0.2,
                radius_max = dist_to_ball * 10,
                clockwise = -1,
                decay=lambda _: 1,
                multiplier = 0.75
            )
        )
        # FULL LEFT LINE
        self.actions.append(
            LineField(
                self.match,
                target = field_limits,
                theta = 0,
                line_size = field_limits[0],
                line_dist = field_limits[1],
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )
        self.actions.append(
            LineField(
                self.match,
                target = field_limits,
                theta = math.pi,
                line_size = field_limits[0],
                line_dist = field_limits[1],
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )
        self.actions.append(
            LineField(
                self.match,
                target = field_limits,
                theta = math.pi/2,
                line_size = field_limits[1],
                line_dist = field_limits[0],
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )
        self.actions.append(
            LineField(
                self.match,
                target = field_limits,
                theta = 3*math.pi/2,
                line_size = field_limits[1],
                line_dist = field_limits[0],
                decay = lambda x: 1,
                multiplier = 0.75 # 75 cm/s
            )
        )

    def create_model(self):
        init = tf.keras.initializers.HeUniform()
        _model = keras.Sequential()
        # Input Layer
        _model.add(keras.layers.Dense(44, input_shape=self.state_shape, activation='relu', kernel_initializer=init))
        # Hidden Layer
        _model.add(keras.layers.Dense(22, activation='relu', kernel_initializer=init))
        # Output Layer
        _model.add(keras.layers.Dense(self.action_shape, activation='linear', kernel_initializer=init))

        _model.compile(loss=tf.keras.losses.Huber(), optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate), metrics=['accuracy'])

        return _model

    def start(self, robot=None):
        super().start(robot=robot)
        self.state_shape = (len(self.get_observation()[0]),)
        if not self.target_model or not self.model:
            self.generate_possible_actions()
            self.target_model = self.create_model()
            self.model = self.create_model()

            checkpoint_path = self.model_folder + "/training_" + self.robot.get_name() +"_{epoch:04d}/cp.ckpt"

            self.cp_callback = tf.keras.callbacks.ModelCheckpoint(
                filepath=checkpoint_path,
                save_weights_only=True,
                verbose=1
            )
    
    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)

    def get_random_action(self):
        return random.randint(0, self.action_shape - 1)

    def update_memory(self):
        observation, reward, done = self.get_observation()
        if self.last_action and self.last_observation:
            if not self.already_trained:
                self.replay_memory.append([self.last_observation, self.last_action, reward, observation, done])
        return done
    
    def update_main_network(self, done):
        if (self.steps_to_update_target_model % STEPS_TO_TRAIN == 0 or done) and len(self.replay_memory) > 64 * 2 * 10:
            self.train(done)

    def train(self, done):
        learning_rate = 0.7
        discount_factor = 0.618


        if len(self.replay_memory) < MIN_REPLAY_SIZE:
            return

        batch_size = 64 * 2 * 10
        mini_batch = random.sample(self.replay_memory, batch_size)

        current_states = np.array([transition[0] for transition in mini_batch])
        current_qs_list = self.model.predict(current_states)
        new_current_states = np.array([transition[3] for transition in mini_batch])
        future_qs_list = self.target_model.predict(new_current_states)

        X = []
        Y = []

        for index, (observation, action, reward, _, done) in enumerate(mini_batch):
            if not done:
                max_future_q = reward + discount_factor * np.max(future_qs_list[index])
            else:
                max_future_q = reward
            
            current_qs = current_qs_list[index]
            current_qs[action] = (1 - learning_rate) * current_qs[action] + learning_rate * max_future_q
            X.append(observation)
            Y.append(current_qs)

        self.model.fit(np.array(X), np.array(Y), batch_size=batch_size, verbose=0, shuffle=True, callbacks=[self.cp_callback])

    def _get_reward(self):
        field = self.match.game.field
        ball = self.match.ball
        field_size = field.get_dimensions()
        if point_in_rect([ball.x, ball.y], field.get_small_area("offensive")):
            return +10
        elif point_in_rect([ball.x, ball.y], field.get_small_area("defensive")):
            return -10
        else:
            field_depth = field_size[0]/2
            sign = 1 if ball.x > field_depth else -1
            reward = (1 - (field_depth - abs((ball.x - field_depth)))/field_depth) ** 12
            return sign * reward + ball.vx/5

    def _is_done(self):
        return not self.match.game.referee.can_play

    def get_observation(self):
        ball = self.match.ball
        robot = self.robot if self.robot else None

        dist_to_ball = ( (ball.x - robot.x)**2 +  (ball.y - robot.y)**2 )**.5 if robot else 0
        min_angle_ball_to_goal = -math.atan2((ball.y - 0.55), (ball.x - 0.75*2)) if robot else 0
        angle_robot_to_ball = -math.atan2((robot.y - ball.y), (robot.x - ball.x )) if robot else 0
        
        min_angle_to_goal = abs(min_angle_ball_to_goal - angle_robot_to_ball)

        return [
            self.match.ball.x, self.match.ball.y, self.match.ball.vx, self.match.ball.vy, 
            dist_to_ball, min_angle_ball_to_goal, angle_robot_to_ball, min_angle_to_goal,
            self.match.robots[0].x, self.match.robots[0].y, self.match.robots[0].vx, self.match.robots[0].vy, self.match.robots[0].theta,
            self.match.robots[1].x, self.match.robots[1].y, self.match.robots[1].vx, self.match.robots[1].vy, self.match.robots[1].theta,
            self.match.robots[2].x, self.match.robots[2].y, self.match.robots[2].vx, self.match.robots[2].vy, self.match.robots[2].theta,
            self.match.opposites[0].x, self.match.opposites[0].y, self.match.opposites[0].vx, self.match.opposites[0].vy, self.match.opposites[0].theta,
            self.match.opposites[1].x, self.match.opposites[1].y, self.match.opposites[1].vx, self.match.opposites[1].vy, self.match.opposites[1].theta,
            self.match.opposites[2].x, self.match.opposites[2].y, self.match.opposites[2].vx, self.match.opposites[2].vy, self.match.opposites[2].theta
        ], self._get_reward(), self._is_done()

    def decide(self):
        self.frames_ran +=1
        done = self.update_memory()
        if self.already_trained and not done:
            self.already_trained = False
        self.update_main_network(done)

        observation, reward, done = self.get_observation()

        self.steps_to_update_target_model += 1
            
        random_number = np.random.rand()

        if random_number <= self.epsilon:
            action = self.get_random_action()
        else:
            encoded = np.array(observation)
            encoded_reshaped = encoded.reshape([1, encoded.shape[0]])
            predicted = self.model.predict(encoded_reshaped).flatten()
            action = np.argmax(predicted)

        self.last_action = action
        self.last_observation = observation
        self.total_training_rewards += reward

        if done and not self.already_trained:
            print('Total training rewards: {} after n steps = {} with final reward = {}'.format(self.total_training_rewards, self.frames_ran, reward))
            self.total_training_rewards += 1 # good value reward, do only if goal was made
            self.frames_ran = 0

            if self.steps_to_update_target_model >= 60 * 60: # 1 minute
                print('Copying main network weights to the target network weights')
                self.target_model.set_weights(self.model.get_weights())
                self.steps_to_update_target_model = 0
            
            self.total_training_rewards = 0
            self.train_count += 1

            self.already_trained = True

        self.epsilon = self.min_epsilon + (self.max_epsilon - self.min_epsilon) * np.exp(-self.decay * self.train_count)

        return self.calculate_by_action(action)
        
        
