import curses
from comm import rl_comm

c = rl_comm.RLComm()
c.start()

id = int(input("Qual o ID?\n"))


stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

def begin_test():
	ratio = 1
	while True:
		left, right = 0,0
		
		key = stdscr.getkey()

		if key == 'A':
			stdscr.addstr('LEFT')
			left += .3
			right += 3.42
		if key == 'D':
			stdscr.addstr('RIGHT')
			left += .3
			right += - 3.42
		if key == 'W':
			stdscr.addstr('FRONT')
			left += .3
			right += .3
		if key == 'S':
			stdscr.addstr('BACK')
			left -= .3
			right -= .3
		if key == 'F':
			stdscr.addstr('RAT')
			ratio -= .2
		if key == 'G':
			stdscr.addstr('RAT')
			ratio += .2
		
		stdscr.clear()
		right *= ratio
		stdscr.addstr(f'ch = {key}, [{left:.2f}, {right:.2f}] ratio={ratio:.3f}\n')
		stdscr.refresh()
		c.send([{
			'robot_id': id,
			'color': 'yellow',
			'wheel_left': left,
			'wheel_right': -right
		}])