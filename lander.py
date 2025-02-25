from .constants import *
import math


class ThrusterController:
    def __init__(self, lander):
        self.lander = lander

    def decide_x_thrust(self):
        boundary_margin = 150
        lander = self.lander
        lander.thruster_outputs["A"] = lander.thruster_outputs["B"] = lander.thruster_outputs["C"] = lander.thruster_outputs["D"] = 0.0
        if lander.position_x < LEFT_BOUND + boundary_margin:
            lander.thruster_outputs["B"] = lander.thruster_outputs["C"] = 40.0
        elif lander.position_x > RIGHT_BOUND - boundary_margin:
            lander.thruster_outputs["A"] = lander.thruster_outputs["D"] = 40.0
        else:
            if lander.velocity_horizontal > 5:
                thrust_value = min(lander.velocity_horizontal, 40)
                lander.thruster_outputs["A"] = lander.thruster_outputs["D"] = thrust_value
            elif lander.velocity_horizontal < -5:
                thrust_value = min(abs(lander.velocity_horizontal), 40)
                lander.thruster_outputs["B"] = lander.thruster_outputs["C"] = thrust_value

    def decide_y_thrust(self):
        lander = self.lander
        if lander.fuel <= 0 or lander.hovering:
            lander.thruster_outputs["main"] = 0.0
            return
        target_velocity = -3.0
        acceleration_required = ((lander.velocity_vertical ** 2 - target_velocity ** 2) /
                                   (2 * lander.altitude)) if lander.altitude > 0 else 0
        thrust_required = (acceleration_required + GRAVITY) / THRUST_MULTIPLIER
        if not lander.hovering and lander.velocity_vertical > -1:
            max_thrust = GRAVITY / THRUST_MULTIPLIER
            thrust_required = min(thrust_required, max_thrust)
        lander.thruster_outputs["main"] = max(min(thrust_required, 100.0), 0.0)


class PhysicsEngine:
    def __init__(self, lander):
        self.lander = lander

    def apply_thrust_forces(self, delta_time):
        lander = self.lander
        main_thruster_force = (lander.thruster_outputs["main"] * THRUST_MULTIPLIER
                               if lander.fuel > 0 else 0.0)
        net_acceleration = max(min(main_thruster_force - GRAVITY, MAX_DECEL), -MAX_DECEL)
        new_velocity_vertical = lander.velocity_vertical + net_acceleration * delta_time
        if not lander.hovering and lander.velocity_vertical < 0 and new_velocity_vertical > 0:
            new_velocity_vertical = 0
        lander.velocity_vertical = new_velocity_vertical
        side_thrust = (lander.thruster_outputs["B"] + lander.thruster_outputs["C"] -
                       lander.thruster_outputs["A"] - lander.thruster_outputs["D"]) * 0.05
        lander.velocity_horizontal = max(min(lander.velocity_horizontal + side_thrust * delta_time, 200), -200)
        total_thrust = sum(lander.thruster_outputs.values())
        lander.fuel = max(lander.fuel - total_thrust * 0.0004, 0)
        lander.temperature += 0.02 * total_thrust * delta_time

    def apply_physics(self, delta_time):
        lander = self.lander
        lander.position_x += lander.velocity_horizontal * delta_time
        if lander.position_x < LEFT_BOUND:
            lander.position_x = LEFT_BOUND
            if lander.velocity_horizontal < 0:
                lander.velocity_horizontal += abs(lander.velocity_horizontal) * 0.5
        elif lander.position_x > RIGHT_BOUND:
            lander.position_x = RIGHT_BOUND
            if lander.velocity_horizontal > 0:
                lander.velocity_horizontal -= abs(lander.velocity_horizontal) * 0.5
        lander.altitude = max(lander.altitude + lander.velocity_vertical * delta_time, 0)
        lander.orientation_angle += lander.angular_velocity * delta_time
        lander.angular_velocity *= 0.98

    def compute_predicted_trajectory(self, steps=120):
        lander = self.lander
        if lander.landed or lander.crashed:
            return []
        altitude = lander.altitude
        velocity_vertical = lander.velocity_vertical
        velocity_horizontal = lander.velocity_horizontal
        position_x = lander.position_x
        main_thruster_force = (lander.thruster_outputs["main"] * THRUST_MULTIPLIER
                               if lander.fuel > 0 else 0)
        predicted_path = []
        simulation_delta_time = 1.0 / 20.0
        for _ in range(steps):
            position_x += velocity_horizontal * simulation_delta_time
            if position_x < LEFT_BOUND:
                position_x = LEFT_BOUND
                if velocity_horizontal < 0:
                    velocity_horizontal += abs(velocity_horizontal) * 0.5
            elif position_x > RIGHT_BOUND:
                position_x = RIGHT_BOUND
                if velocity_horizontal > 0:
                    velocity_horizontal -= abs(velocity_horizontal) * 0.5
            next_altitude = altitude + velocity_vertical * simulation_delta_time
            net_acceleration = max(min(main_thruster_force - GRAVITY, MAX_DECEL), -MAX_DECEL)
            velocity_vertical += net_acceleration * simulation_delta_time
            if next_altitude <= 0:
                predicted_path.append((position_x, 0))
                break
            altitude = next_altitude
            predicted_path.append((position_x, altitude))
        return predicted_path


class PhaseManager:
    def __init__(self, lander):
        self.lander = lander

    def update_phase(self):
        lander = self.lander
        if lander.altitude < 30000 and lander.phase == "Rough Braking":
            lander.phase = "Fine Braking"
        if lander.altitude < 3000 and lander.phase == "Fine Braking":
            lander.phase = "Final Landing"

    def check_landed_or_crashed(self):
        lander = self.lander
        if lander.altitude <= 0 and not lander.landed and not lander.crashed:
            if abs(lander.velocity_vertical) < 3 and abs(lander.orientation_angle) <= MAX_TILT:
                lander.landed = True
                lander.phase = "Landed"
                lander.velocity_vertical = 0
                lander.velocity_horizontal = 0
                lander.flag_raise_timer = 3.0
            else:
                lander.crashed = True
                lander.phase = "Crashed"


class CraterAvoidance:
    def __init__(self, lander):
        self.lander = lander

    def check_crater(self, terrain, delta_time):
        lander = self.lander
        if lander.altitude < 200 and not lander.hovering and not lander.landed and not lander.crashed:
            for crater_center_x, crater_radius in terrain.craters:
                if abs(lander.position_x - crater_center_x) < crater_radius + 30:
                    lander.hovering = True
                    lander.hover_duration_remaining = 3.0
                    lander.target_position_x = (lander.position_x - (crater_radius + 120)
                                                if lander.position_x < crater_center_x
                                                else lander.position_x + (crater_radius + 120))
                    lander.thruster_outputs["main"] = 100.0
                    break
        if lander.hovering:
            lander.hover_duration_remaining -= delta_time
            lander.velocity_vertical = 0
            if lander.target_position_x is not None:
                distance_to_target = lander.target_position_x - lander.position_x
                if abs(distance_to_target) > 3:
                    lander.velocity_horizontal = math.copysign(min(math.sqrt(abs(distance_to_target)) * 2, 80),
                                                               distance_to_target)
                else:
                    lander.velocity_horizontal = 0
            if lander.hover_duration_remaining <= 0:
                lander.hovering = False
                lander.target_position_x = None


class Lander:
    def __init__(self):
        self.altitude = START_ALTITUDE
        self.velocity_vertical = START_VELOCITY
        self.velocity_horizontal = START_HVEL
        self.position_x = START_POS_X
        self.orientation_angle = 0.0
        self.angular_velocity = 0.0
        self.phase = "Rough Braking"
        self.fuel = 100.0
        self.temperature = 100.0
        self.thruster_outputs = {"main": 0.0, "A": 0.0, "B": 0.0, "C": 0.0, "D": 0.0}
        self.hovering = False
        self.hover_duration_remaining = 0.0
        self.target_position_x = None
        self.landed = False
        self.crashed = False
        self.flag_raised = False
        self.flag_raise_timer = 0.0
        self.flight_path_points = []
        self.predicted_path = []

        self.thruster_controller = ThrusterController(self)
        self.physics_engine = PhysicsEngine(self)
        self.phase_manager = PhaseManager(self)
        self.crater_avoidance = CraterAvoidance(self)

    def altitude_in_km(self):
        return self.altitude / 1000.0

    def time_to_land(self):
        return "N/A" if self.velocity_vertical >= 0 else f"{abs(self.altitude / self.velocity_vertical):.1f}s"

    def compute_predicted_trajectory(self, steps=120):
        return self.physics_engine.compute_predicted_trajectory(steps)

    def update(self, delta_time, terrain):
        if self.landed:
            if self.flag_raise_timer > 0:
                self.flag_raise_timer -= delta_time
                if self.flag_raise_timer <= 0:
                    self.flag_raised = True
            return
        if self.crashed:
            return

        self.phase_manager.update_phase()
        self.crater_avoidance.check_crater(terrain, delta_time)
        self.thruster_controller.decide_x_thrust()
        self.thruster_controller.decide_y_thrust()
        self.physics_engine.apply_thrust_forces(delta_time)
        self.physics_engine.apply_physics(delta_time)
        self.phase_manager.check_landed_or_crashed()
        self.flight_path_points.append((self.position_x, self.altitude))
        self.predicted_path = self.compute_predicted_trajectory()