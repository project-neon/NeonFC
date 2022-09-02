from strategy.BaseStrategy import Strategy
import math
from controller.uni_controller import UniController

class UVF_Agent(Strategy):
    def __init__(self, match, name="UVF_Test"):
        super().__init__(match, name=name, controller=UniController)

    def N_Posture(self, x, y, bx, by, bq):
        _D = 2
        _N = 8

        theta_d = 0
        rx, ry = 0, 0
        r1, r2, r3, phi, dist = 0, 0, 0, 0, 0

        rx = bx + _D*math.cos(bq)
        ry = by + _D*math.sin(bq)

        r1 = math.atan2(by-y, bx-x)
        r2 = math.atan2(ry-y, rx-x)
        r3 = math.atan2(ry-by, rx-bx)

        dist = math.sqrt((bx-x)**2 + (by-y)**2)

        phi = r2-r1

        while(phi > math.pi): phi -= 2*math.pi
        while(phi < -math.pi): phi += 2*math.pi

        theta_d = r1 - _N*phi

        while(theta_d > math.pi): theta_d -= 2*math.pi
        while(theta_d < -math.pi): theta_d += 2*math.pi

        return theta_d

    def N_Obstacle(self, x, y, ox, oy, theta_d):
        _Ro = 10
        _M = 4
        dist, length, angle, diff_angle = 0, 0, 0, 0
        tmp_x, tmp_y = 0, 0

        dist = math.sqrt((ox-x)**2 + (y-oy)**2)
        length = abs((ox-x)*math.sin(theta_d) + (y-oy)*math.cos(theta_d))
        angle = math.atan2(oy-y, ox-x)
        diff_angle = theta_d - angle

        while(diff_angle > math.pi): diff_angle -= 2*math.pi
        while(diff_angle < -math.pi): diff_angle += 2*math.pi

        if(length < (_Ro+_M) and abs(diff_angle) < math.pi/2):
            if(dist <= _Ro): theta_d = angle-math.pi
            elif(dist <= _Ro+_M):
                if(diff_angle > 0):
                    tmp_x = ((dist-_Ro)*math.cos(angle - 1.5*math.pi)+(_Ro+_M-dist)*math.cos(angle - math.pi))/_M
                    tmp_y = ((dist-_Ro)*math.sin(angle - 1.5*math.pi)+(_Ro+_M-dist)*math.sin(angle - math.pi))/_M
                    theta_d = math.atan2(tmp_y, tmp_x)
                else:
                    tmp_x = ((dist-_Ro)*math.cos(angle - 0.5*math.pi)+(_Ro+_M-dist)*math.cos(angle - math.pi))/_M
                    tmp_y = ((dist-_Ro)*math.sin(angle - 0.5*math.pi)+(_Ro+_M-dist)*math.sin(angle - math.pi))/_M
                    theta_d = math.atan2(tmp_y, tmp_x)
            else:
                if(diff_angle > 0):
                    theta_d = abs(math.atan((_Ro+_M)/math.sqrt(dist**2 - (_Ro+_M)**2))) + angle
                else:
                    theta_d = -abs(math.atan((_Ro+_M)/math.sqrt(dist**2 - (_Ro+_M)**2))) + angle
        
        return theta_d

    def start(self, robot=None):
        super().start(robot=robot)

    def decide(self):
        print(f"{self.robot.x=}\n{self.robot.y=}")
        print(math.sqrt((self.robot.x-self.match.ball.x)**2 + (self.robot.y-self.match.ball.y)**2))
        
        dl = 0.000001
        bq = math.atan2(0.65-self.match.ball.y, 1.5-self.match.ball.x)
        Uvect = self.N_Posture(self.robot.x, self.robot.y, self.match.ball.x, self.match.ball.y, bq)
        Uvect_dl = self.N_Posture(self.robot.x + dl*math.cos(self.robot.theta), self.robot.y + dl*math.sin(self.robot.theta), self.match.ball.x, self.match.ball.y, bq)
        
        # for r in self.match.robots:
        #     if not r.robot_id == self.robot.robot_id:
        #         Uvect = self.N_Obstacle(x, y, r.x, r.y, Uvect)
        #         Uvect_dl = self.N_Obstacle(x, y, r.x, r.y, Uvect_dl)
        
        return Uvect, Uvect_dl

    def reset(self, robot=None):
        super().reset()
        if robot:
            self.start(robot)