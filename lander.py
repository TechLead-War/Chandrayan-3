from .constants import *


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
