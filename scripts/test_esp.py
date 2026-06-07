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
			stdscr.addch('LEFT')
			left += 5
		if key == curses.KEY_RIGHT:
			'D'
			right += 5
		
		stdscr.addstr(f'ch = {key}\n')
		stdscr.refresh()

		c.send([{
			'robot_id': id,
			'color': 'yellow',
			'wheel_left': left,
			'wheel_right': right
		}])