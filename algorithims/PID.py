class Pid:
	def __init__(self, _kp, _ki, _kd, _ilimit=5000.0):
		self.kp = _kp
		self.ki = _ki
		self.kd = _kd

		self.ilimit = _ilimit

		self.integ = 0
		self.prev_error = 0

	def set_target(self, target):
		self.desired = target

	def update(self, _input, dt):
		error = self.desired - _input

		self.integ += error * dt * self.ki

		if (self.integ > self.ilimit):
			self.integ = self.ilimit
		elif (self.integ < -self.ilimit):
			self.integ = -self.ilimit

		deriv = (error - self.prev_error)/ dt

		output = self.kp * error + self.kd * deriv + self.integ

		self.prev_error = error

		return output

	def reset(self):
		self.prev_error = self.integ = 0

class RobotPid:
	DEFAULT_PID_INTEGRATION_LIMIT = 0

	def __init__(self, robot_id):
		self.dt = 1
		self.pid_l = {'kp': 1,'ki': 0,'kd': 0}
		self.pid_a = {'kp': 1.25,'ki': 0.0015,'kd': 0}

		self.robot_id = robot_id

		self.pid_lin = Pid(*self.pid_l.values())
		self.pid_ang = Pid(*self.pid_a.values())

		self.power_left = 0
		self.power_right = 0

	def set_target(self, target_linear, target_theta):
		self.pid_lin.set_target(target_linear)
		self.pid_ang.set_target(target_theta)

	def speed_to_power(self, actual_y, actual_theta):
		speed_y = self.pid_lin.update(actual_y, self.dt)
		speed_theta = self.pid_ang.update(actual_theta, self.dt)
		acc_left = speed_y + speed_theta
		acc_right = speed_y - speed_theta

		self.power_left = acc_left * self.dt
		self.power_right = acc_right * self.dt

		self.power_left = min(100, max(-100, self.power_left))
		self.power_right = min(100, max(-100, self.power_right))

		return (self.power_left, self.power_right)

