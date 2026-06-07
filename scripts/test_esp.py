from comm import rl_comm

c = rl_comm.RLComm()
c.start()

id = int(input("Qual o ID"))

def begin_test():
	while True:
		inp = input()
		if inp == 'L':
			print('lft')
			c.send([{
				'robot_id': id,
				'color': 'yellow',
				'wheel_left': 20,
				'wheel_right': -20
			}])
		if inp == 'R':
			print('rgh')
		print(inp)