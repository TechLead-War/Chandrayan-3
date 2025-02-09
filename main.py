import pygame, sys, math, random, numpy as np

WIDTH, HEIGHT = 1500, 1000
LEFT_BOUND, RIGHT_BOUND = 100, WIDTH - 100
GRAVITY = 1.625       # m/s²
MAX_TILT = 30.0       # degrees
MAX_DECEL = 200.0     # m/s²
START_ALTITUDE = 100000.0
START_VELOCITY = -1500.0
START_POS_X = 50.0
START_HVEL = 100.0
THRUST_MULTIPLIER = 1.2  # main force = thrusters["main"]*THRUST_MULTIPLIER

BLACK  = (0,0,0)
WHITE  = (255,255,255)
GRAY   = (160,160,160)
GREEN  = (50,255,50)
RED    = (255,0,0)
ORANGE = (255,165,0)
YELLOW = (255,255,50)
DARK   = (40,40,40)
BROWN  = (128,70,50)
BLUE   = (0,128,255)


class Terrain:
    def __init__(self):
        self.craters = []
        for _ in range(6):
            cx = random.randint(LEFT_BOUND+50, RIGHT_BOUND-50)
            r = random.randint(30,80)
            self.craters.append((cx, r))
    def draw(self, screen):
        pygame.draw.rect(screen, DARK, (0, HEIGHT-50, WIDTH, 50))
        ground_y = HEIGHT-50
        for (cx, r) in self.craters:
            pygame.draw.circle(screen, BROWN, (cx, ground_y), r)


class Lander:
    # todo: retargeting landing area.
    def __init__(self):
        self.altitude = START_ALTITUDE
        self.vertical_velocity = START_VELOCITY
        self.horizontal_velocity = START_HVEL
        self.x = START_POS_X
        self.angle = 0.0
        self.angular_vel = 0.0
        self.phase = "Rough Braking"
        self.fuel = 100.0
        self.temperature = 100.0
        self.thrusters = {"main":0.0, "A":0.0, "B":0.0, "C":0.0, "D":0.0}
        self.hovering = False
        self.hover_timer = 0.0
        self.target_x = None
        self.landed = False
        self.crashed = False
        self.flag_raised = False
        self.wait_flag_timer = 0.0
        self.path_points = []
        self.projected_path = []

    def altitude_km(self):
        return self.altitude/1000.0

    def time_to_land(self):
        if self.vertical_velocity >= 0:
            return "N/A"
        return f"{abs(self.altitude/self.vertical_velocity):.1f}s"

    def update_phase(self):
        if self.altitude < 30000 and self.phase=="Rough Braking":
            self.phase = "Fine Braking"
        if self.altitude < 3000 and self.phase=="Fine Braking":
            self.phase = "Final Landing"

    def check_landed_or_crashed(self):
        if self.altitude<=0 and not self.landed and not self.crashed:
            if abs(self.vertical_velocity)<3 and abs(self.angle)<=MAX_TILT:
                self.landed=True; self.phase="Landed"
                self.vertical_velocity = 0; self.horizontal_velocity = 0
                self.wait_flag_timer = 3.0
            else:
                self.crashed=True; self.phase="Crashed"

    def check_crater(self, terrain, dt):
        if self.altitude < 200 and not self.hovering and not self.landed and not self.crashed:
            for (cx, r) in terrain.craters:
                if abs(self.x-cx) < r+30:
                    self.hovering = True
                    self.hover_timer = 3.0
                    self.target_x = self.x - (r+120) if self.x<cx else self.x+(r+120)
                    self.thrusters["main"] = 100.0
                    break
        if self.hovering:
            self.hover_timer -= dt
            self.vertical_velocity = 0
            if self.target_x is not None:
                dx = self.target_x - self.x
                if abs(dx)>3:
                    self.horizontal_velocity = math.copysign(min(math.sqrt(abs(dx))*2,80), dx)
                else:
                    self.horizontal_velocity = 0
            if self.hover_timer<=0:
                self.hovering = False; self.target_x = None

    def decide_x_thrust(self):
        margin = 150
        self.thrusters["A"] = self.thrusters["B"] = self.thrusters["C"] = self.thrusters["D"] = 0.0
        if self.x < LEFT_BOUND+margin:
            self.thrusters["B"] = self.thrusters["C"] = 40.0
        elif self.x > RIGHT_BOUND-margin:
            self.thrusters["A"] = self.thrusters["D"] = 40.0
        else:
            if self.horizontal_velocity>5:
                val = min(self.horizontal_velocity,40)
                self.thrusters["A"] = self.thrusters["D"] = val
            elif self.horizontal_velocity<-5:
                val = min(abs(self.horizontal_velocity),40)
                self.thrusters["B"] = self.thrusters["C"] = val

    # --- Use a braking law that drives vertical speed toward a target of –3 m/s.
    def decide_y_thrust(self):
        if self.fuel<=0 or self.hovering:
            self.thrusters["main"] = 0.0; return
        target_v = -3.0
        if self.altitude>0:
            a_req = (self.vertical_velocity**2 - target_v**2)/(2*self.altitude)
        else:
            a_req = 0
        desired_force = a_req + GRAVITY
        thr = desired_force / THRUST_MULTIPLIER
        # When nearly zero, do not over‐thrust (unless crater mode)
        if not self.hovering and self.vertical_velocity > -1:
            max_thr = GRAVITY/THRUST_MULTIPLIER
            if thr>max_thr:
                thr = max_thr
        self.thrusters["main"] = max(min(thr,100.0),0.0)

    def apply_thrust_forces(self, dt):
        main_force = self.thrusters["main"]*THRUST_MULTIPLIER if self.fuel>0 else 0.0
        net_acc = main_force - GRAVITY
        net_acc = max(min(net_acc, MAX_DECEL), -MAX_DECEL)
        new_vy = self.vertical_velocity + net_acc*dt
        # Prevent upward (positive) velocity unless in crater-hover mode.
        if not self.hovering and self.vertical_velocity<0 and new_vy>0:
            new_vy = 0
        self.vertical_velocity = new_vy
        side = (self.thrusters["B"]+self.thrusters["C"]-self.thrusters["A"]-self.thrusters["D"])*0.05
        self.horizontal_velocity = max(min(self.horizontal_velocity+side*dt,200),-200)
        total = sum(self.thrusters.values())
        self.fuel = max(self.fuel - total*0.0004,0)
        self.temperature += 0.02*total*dt

    def apply_physics(self, dt):
        self.x += self.horizontal_velocity*dt
        if self.x < LEFT_BOUND:
            self.x = LEFT_BOUND
            if self.horizontal_velocity<0:
                self.horizontal_velocity += abs(self.horizontal_velocity)*0.5
        elif self.x>RIGHT_BOUND:
            self.x = RIGHT_BOUND
            if self.horizontal_velocity>0:
                self.horizontal_velocity -= abs(self.horizontal_velocity)*0.5
        self.altitude = max(self.altitude + self.vertical_velocity*dt, 0)
        self.angle += self.angular_vel*dt
        self.angular_vel *= 0.98

    def compute_predicted_trajectory(self, steps=120):
        if self.landed or self.crashed: return []
        alt = self.altitude; vy = self.vertical_velocity; vx = self.horizontal_velocity; px = self.x
        main_force = self.thrusters["main"]*THRUST_MULTIPLIER if self.fuel>0 else 0
        path = []; dt_sim = 1.0/20.0
        for _ in range(steps):
            px += vx*dt_sim
            if px<LEFT_BOUND:
                px = LEFT_BOUND; 
                if vx<0: vx += abs(vx)*0.5
            elif px>RIGHT_BOUND:
                px = RIGHT_BOUND;
                if vx>0: vx -= abs(vx)*0.5
            alt_next = alt + vy*dt_sim
            net = max(min(main_force - GRAVITY, MAX_DECEL), -MAX_DECEL)
            vy = vy + net*dt_sim
            if alt_next<=0:
                alt_next = 0; path.append((px,alt_next)); break
            alt = alt_next; path.append((px,alt))
        return path

    def update(self, dt, terrain):
        if self.landed:
            if self.wait_flag_timer>0:
                self.wait_flag_timer -= dt
                if self.wait_flag_timer<=0: self.flag_raised = True
            return
        if self.crashed: return
        self.update_phase()
        self.check_crater(terrain, dt)
        self.decide_x_thrust()
        self.decide_y_thrust()
        self.apply_thrust_forces(dt)
        self.apply_physics(dt)
        self.check_landed_or_crashed()
        self.path_points.append((self.x, self.altitude))
        self.projected_path = self.compute_predicted_trajectory()

def draw_scale(screen, lander):
    bx = WIDTH-60; by = 50; bh = HEIGHT-150
    pygame.draw.rect(screen, GRAY, (bx,by,20,bh),2)
    ratio = min(lander.altitude/100000.0,1)
    fill_h = int(bh*ratio); fy = by+(bh-fill_h)
    pygame.draw.rect(screen, GREEN, (bx+2,fy,16,fill_h))
    step = 100000//10
    font = pygame.font.Font(None,24)
    for i in range(11):
        alt_mark = i*step; y_mark = by+bh-int((alt_mark/100000.0)*bh)
        lab = f"{alt_mark//1000} km"
        screen.blit(font.render(lab, True, WHITE), (bx-50,y_mark-8))

def draw_hud(screen, lander):
    font = pygame.font.Font(None,28)
    lines = [f"Phase: {lander.phase}",
             f"Alt: {lander.altitude_km():.2f} km",
             f"VelY: {lander.vertical_velocity:.1f} m/s",
             f"VelX: {lander.horizontal_velocity:.1f} m/s",
             f"Tilt: {lander.angle:.1f}°",
             f"Fuel: {lander.fuel:.1f}%",
             f"Temp: {lander.temperature:.1f}°C",
             f"Time2Land: {lander.time_to_land()}"]
    y=20
    for l in lines:
        screen.blit(font.render(l, True, WHITE),(20,y)); y+=30
    thr = lander.thrusters
    s = f"A:{thr['A']:.1f} B:{thr['B']:.1f} C:{thr['C']:.1f} D:{thr['D']:.1f} Main:{thr['main']:.1f}"
    screen.blit(font.render(s, True, ORANGE),(20,y))

def draw_trajectory(screen, lander):
    if len(lander.path_points)>=2:
        pts=[]
        scale = HEIGHT-200
        for (px, alt) in lander.path_points:
            r = min(alt/100000.0,1)
            py = (HEIGHT-150) - r*scale
            pts.append((int(px), int(py)))
        if len(pts)>=2:
            pygame.draw.lines(screen, GREEN, False, pts, 2)
    if len(lander.projected_path)>=2:
        pts=[]
        scale = HEIGHT-200
        for (px, alt) in lander.projected_path:
            r = min(alt/100000.0,1)
            py = (HEIGHT-150)-r*scale
            pts.append((int(px), int(py)))
        for i in range(0, len(pts)-1, 8):
            pygame.draw.line(screen, BLUE, pts[i], pts[min(i+4, len(pts)-1)], 2)

def draw_lander(screen, lander):
    scale = HEIGHT-200
    r = min(lander.altitude/100000.0,1)
    py = (HEIGHT-150)-r*scale
    bw, bh = 60,60
    rect = pygame.Rect(lander.x, py, bw, bh)
    center = rect.center
    surf = pygame.Surface((bw,bh), pygame.SRCALPHA)
    pygame.draw.rect(surf, GRAY, (0,0,bw,bh))
    pygame.draw.rect(surf, RED, (bw//2-5,5,10,10))
    pygame.draw.polygon(surf, WHITE, [(0,bh), (-15,bh+30), (bw+15, bh+30), (bw,bh)])
    rsurf = pygame.transform.rotate(surf, lander.angle)
    rrect = rsurf.get_rect(center=center)
    screen.blit(rsurf, rrect)
    draw_thrusters(screen, lander, rrect)
    if lander.flag_raised:
        pole = pygame.Rect(rrect.centerx+20, rrect.top-40, 5, 40)
        pygame.draw.rect(screen, WHITE, pole)
        fw, fh = 30,20
        flag = pygame.Rect(pole.right, pole.top, fw, fh)
        pygame.draw.rect(screen, (255,153,51), (flag.x, flag.y, fw, fh//3))
        pygame.draw.rect(screen, (255,255,255), (flag.x, flag.y+fh//3, fw, fh//3))
        pygame.draw.rect(screen, (19,136,8), (flag.x, flag.y+2*(fh//3), fw, fh//3))

def draw_thrusters(screen, lander, rect):
    cx, cy = rect.center
    m = lander.thrusters["main"]
    fl = int(m*0.6)
    if fl>0:
        pygame.draw.rect(screen, ORANGE, (cx-5, rect.bottom, 10, fl))
    for key, col, offs in [("A", YELLOW, (-1,0)), ("B", YELLOW, (1,0)),
                           ("C", YELLOW, (1, -5)), ("D", YELLOW, (-1, -5))]:
        val = int(lander.thrusters[key])
        if val>0:
            if key=="A":
                pos = rect.topleft; pos = (pos[0]-val, pos[1])
            elif key=="B":
                pos = rect.topright; pos = (pos[0], pos[1])
            elif key=="C":
                pos = rect.bottomright; pos = (pos[0], pos[1]-5)
            else:
                pos = rect.bottomleft; pos = (pos[0]-val, pos[1]-5)
            pygame.draw.rect(screen, col, (pos[0], pos[1], val, 5))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("Chandrayaan Landing")
    clock = pygame.time.Clock()
    lander = Lander()
    terrain = Terrain()
    running = True
    while running:
        dt = clock.tick(20)/1000.0
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running = False
        screen.fill(BLACK)
        lander.update(dt, terrain)
        terrain.draw(screen)
        draw_trajectory(screen, lander)
        draw_lander(screen, lander)
        draw_scale(screen, lander)
        draw_hud(screen, lander)
        pygame.display.flip()
    pygame.quit(); sys.exit()

if __name__=="__main__":
    main()