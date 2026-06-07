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
		
		key = stdscr.getkey()

		if key == 'A':
			stdscr.addstr('LEFT')
			left += 1
		if key == 'D':
			stdscr.addstr('RIGHT')
			right += 1
		if key == 'W':
			stdscr.addstr('FRONT')
			left += 1
			right += 1
		if key == 'S':
			stdscr.addstr('BACK')
			left += 1
			right += 1
		
		stdscr.addstr(f'ch = {key}\n')
		stdscr.refresh()

		c.send([{
			'robot_id': id,
			'color': 'yellow',
			'wheel_left': left,
			'wheel_right': right
		}])