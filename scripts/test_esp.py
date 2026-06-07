import curses
from comm import rl_comm

c = rl_comm.RLComm()
c.start()

id = int(input("Qual o ID?\n"))

ratio = 2

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

def begin_test():

	while True:
		left, right = 0,0
		
		key = stdscr.getkey()

		if key == 'A':
			stdscr.addstr('LEFT')
			left += .3
			right -= .3
		if key == 'D':
			stdscr.addstr('RIGHT')
			right += .3
			left -= .3
		if key == 'W':
			stdscr.addstr('FRONT')
			left += .3
			right += .3
		if key == 'S':
			stdscr.addstr('BACK')
			left -= .3
			right -= .3
		
		stdscr.clear()
		stdscr.addstr(f'ch = {key}, [{left:.2f}, {right:.2f}]\n')
		stdscr.refresh()

		right *= ratio

		c.send([{
			'robot_id': id,
			'color': 'yellow',
			'wheel_left': left,
			'wheel_right': right
		}])