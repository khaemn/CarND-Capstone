
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 vehicle_mass=0.,
                 fuel_capacity=0.,
                 brake_deadband=0.,
                 decel_limit=0.,
                 accel_limit=0.,
                 wheel_radius=0.,
                 wheel_base=0.,
                 steer_ratio=0.,
                 max_lat_accel=0.,
                 max_steer_angle=0.,
                 **kwargs):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        # TODO: Implement
        pass

    def control(self,
                desired_linear_vel=0.,
                desired_angular_vel=0.,
                curr_linear_vel=0.,
                curr_angular_vel=0.,
                dbw_enabled=False,
                **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return .2, 0., 0.7
