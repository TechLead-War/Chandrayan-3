from .constants import *
import math


class ThrusterController:
    def __init__(self, lander):
        self.lander = lander

    def decide_x_thrust(self):
        margin = 150
        L = self.lander
        L.thrusters["A"] = L.thrusters["B"] = L.thrusters["C"] = L.thrusters["D"] = 0.0
        if L.x < LEFT_BOUND + margin:
            L.thrusters["B"] = L.thrusters["C"] = 40.0
        elif L.x > RIGHT_BOUND - margin:
            L.thrusters["A"] = L.thrusters["D"] = 40.0
        else:
            if L.horizontal_velocity > 5:
                val = min(L.horizontal_velocity, 40)
                L.thrusters["A"] = L.thrusters["D"] = val
            elif L.horizontal_velocity < -5:
                val = min(abs(L.horizontal_velocity), 40)
                L.thrusters["B"] = L.thrusters["C"] = val

    def decide_y_thrust(self):
        L = self.lander
        if L.fuel <= 0 or L.hovering:
            L.thrusters["main"] = 0.0
            return
        target_v = -3.0
        a_req = ((L.vertical_velocity**2 - target_v**2) / (2 * L.altitude)) if L.altitude > 0 else 0
        thr = (a_req + GRAVITY) / THRUST_MULTIPLIER
        if not L.hovering and L.vertical_velocity > -1:
            max_thr = GRAVITY / THRUST_MULTIPLIER
            thr = min(thr, max_thr)
        L.thrusters["main"] = max(min(thr, 100.0), 0.0)


class PhysicsEngine:
    def __init__(self, lander):
        self.lander = lander

    def apply_thrust_forces(self, dt):
        L = self.lander
        main_force = L.thrusters["main"] * THRUST_MULTIPLIER if L.fuel > 0 else 0.0
        net_acc = max(min(main_force - GRAVITY, MAX_DECEL), -MAX_DECEL)
        new_vy = L.vertical_velocity + net_acc * dt
        if not L.hovering and L.vertical_velocity < 0 and new_vy > 0:
            new_vy = 0
        L.vertical_velocity = new_vy
        side = (L.thrusters["B"] + L.thrusters["C"] - L.thrusters["A"] - L.thrusters["D"]) * 0.05
        L.horizontal_velocity = max(min(L.horizontal_velocity + side * dt, 200), -200)
        total = sum(L.thrusters.values())
        L.fuel = max(L.fuel - total * 0.0004, 0)
        L.temperature += 0.02 * total * dt

    def apply_physics(self, dt):
        L = self.lander
        L.x += L.horizontal_velocity * dt
        if L.x < LEFT_BOUND:
            L.x = LEFT_BOUND
            if L.horizontal_velocity < 0:
                L.horizontal_velocity += abs(L.horizontal_velocity) * 0.5
        elif L.x > RIGHT_BOUND:
            L.x = RIGHT_BOUND
            if L.horizontal_velocity > 0:
                L.horizontal_velocity -= abs(L.horizontal_velocity) * 0.5
        L.altitude = max(L.altitude + L.vertical_velocity * dt, 0)
        L.angle += L.angular_vel * dt
        L.angular_vel *= 0.98

    def compute_predicted_trajectory(self, steps=120):
        L = self.lander
        if L.landed or L.crashed:
            return []
        alt, vy, vx, px = L.altitude, L.vertical_velocity, L.horizontal_velocity, L.x
        main_force = L.thrusters["main"] * THRUST_MULTIPLIER if L.fuel > 0 else 0
        path = []
        dt_sim = 1.0 / 20.0
        for _ in range(steps):
            px += vx * dt_sim
            if px < LEFT_BOUND:
                px = LEFT_BOUND
                if vx < 0:
                    vx += abs(vx) * 0.5
            elif px > RIGHT_BOUND:
                px = RIGHT_BOUND
                if vx > 0:
                    vx -= abs(vx) * 0.5
            alt_next = alt + vy * dt_sim
            net = max(min(main_force - GRAVITY, MAX_DECEL), -MAX_DECEL)
            vy += net * dt_sim
            if alt_next <= 0:
                path.append((px, 0))
                break
            alt = alt_next
            path.append((px, alt))
        return path


class PhaseManager:
    def __init__(self, lander):
        self.lander = lander

    def update_phase(self):
        L = self.lander
        if L.altitude < 30000 and L.phase == "Rough Braking":
            L.phase = "Fine Braking"
        if L.altitude < 3000 and L.phase == "Fine Braking":
            L.phase = "Final Landing"

    def check_landed_or_crashed(self):
        L = self.lander
        if L.altitude <= 0 and not L.landed and not L.crashed:
            if abs(L.vertical_velocity) < 3 and abs(L.angle) <= MAX_TILT:
                L.landed = True
                L.phase = "Landed"
                L.vertical_velocity = L.horizontal_velocity = 0
                L.wait_flag_timer = 3.0
            else:
                L.crashed = True
                L.phase = "Crashed"


class CraterAvoidance:
    def __init__(self, lander):
        self.lander = lander

    def check_crater(self, terrain, dt):
        L = self.lander
        if L.altitude < 200 and not L.hovering and not L.landed and not L.crashed:
            for (cx, r) in terrain.craters:
                if abs(L.x - cx) < r + 30:
                    L.hovering = True
                    L.hover_timer = 3.0
                    L.target_x = L.x - (r + 120) if L.x < cx else L.x + (r + 120)
                    L.thrusters["main"] = 100.0
                    break
        if L.hovering:
            L.hover_timer -= dt
            L.vertical_velocity = 0
            if L.target_x is not None:
                dx = L.target_x - L.x
                L.horizontal_velocity = math.copysign(min(math.sqrt(abs(dx)) * 2, 80), dx) if abs(dx) > 3 else 0
            if L.hover_timer <= 0:
                L.hovering = False
                L.target_x = None


class Lander:
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
        self.thrusters = {"main": 0.0, "A": 0.0, "B": 0.0, "C": 0.0, "D": 0.0}
        self.hovering = False
        self.hover_timer = 0.0
        self.target_x = None
        self.landed = False
        self.crashed = False
        self.flag_raised = False
        self.wait_flag_timer = 0.0
        self.path_points = []
        self.projected_path = []

        self.thruster_controller = ThrusterController(self)
        self.physics_engine = PhysicsEngine(self)
        self.phase_manager = PhaseManager(self)
        self.crater_avoidance = CraterAvoidance(self)

    def altitude_km(self):
        return self.altitude / 1000.0

    def time_to_land(self):
        return "N/A" if self.vertical_velocity >= 0 else f"{abs(self.altitude / self.vertical_velocity):.1f}s"

    def compute_predicted_trajectory(self, steps=120):
        return self.physics_engine.compute_predicted_trajectory(steps)

    def update(self, dt, terrain):
        if self.landed:
            if self.wait_flag_timer > 0:
                self.wait_flag_timer -= dt
                if self.wait_flag_timer <= 0:
                    self.flag_raised = True
            return
        if self.crashed:
            return

        self.phase_manager.update_phase()
        self.crater_avoidance.check_crater(terrain, dt)
        self.thruster_controller.decide_x_thrust()
        self.thruster_controller.decide_y_thrust()
        self.physics_engine.apply_thrust_forces(dt)
        self.physics_engine.apply_physics(dt)
        self.phase_manager.check_landed_or_crashed()
        self.path_points.append((self.x, self.altitude))
        self.projected_path = self.compute_predicted_trajectory()
