from live_plot.logger import Writer

kp = -100
ki = 0
kd = -7.5

pid_tunner = Writer('pid_tuner',
                    {'kp': 'FLOAT',
                     'ki': 'FLOAT',
                     'kd': 'FLOAT'})

pid_tunner.write([kp, ki, kd])

while True:

    gain = input(f"change in kp({kp}), ki({ki}) or kd({kd}): ")
    if gain == 'kp':
        kp = float(input('kp: '))
    if gain == 'ki':
        ki = float(input('ki: '))
    if gain == 'kd':
        kd = float(input('kd: '))

    pid_tunner.write([kp, ki, kd])
