import curses
from comm import rl_comm

c = rl_comm.RLComm()
c.start()

id = int(input("Qual o ID?\n"))
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

def begin_test():

	while True:
		left, right = 0,0
		
		stdscr.refresh()
		key = stdscr.getkey()

		if key == 'KEY_LEFT':
			left += 1
		if key == 'KEY_RIGHT':
			left += 1
		

		c.send([{
			'robot_id': id,
			'color': 'yellow',
			'wheel_left': left,
			'wheel_right': right
		}])
		print(inp)