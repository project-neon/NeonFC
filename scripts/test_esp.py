from comm import rl_comm

c = rl_comm.RLComm()
c.start()

def begin_test():
	while True:
		inp = input()
		if inp == 'L':
			print('lft')
			c.send([{
				'robot_id': 7,
				'color': 'yellow',
				'wheel_left': 5.0,
				'wheel_right': 0.0
			}])
		if inp == 'R':
			print('rgh')
		print(inp)