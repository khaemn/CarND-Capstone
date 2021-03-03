
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

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
        self.min_speed_ms = 0.1
        
        kp = 0.3; ki = 0.1; kd = 0.;
        self.min_allowed_throttle = 0.
        max_allowed_throttle = 0.4
        self.throttle_controller = PID(kp, ki, kd, self.min_allowed_throttle, max_allowed_throttle)
        
        self.yaw_controller = YawController(self.wheel_base,
                                            self.steer_ratio,
                                            self.min_speed_ms,
                                            self.max_lat_accel,
                                            self.max_steer_angle)
        sample_time_sec = 0.02
        cutoff_freq_hz = 0.5
        self.speed_filter = LowPassFilter(cutoff_freq_hz, sample_time_sec)
        self.last_time = rospy.get_time()

        # TODO: Implement
        pass

    def control(self,
                desired_linear_vel=0.,
                desired_angular_vel=0.,
                curr_linear_vel=0.,
                curr_angular_vel=0.,
                dbw_enabled=False,
                **kwargs):
        time_now = rospy.get_time()
        time_passed = time_now - self.last_time
        self.last_time = time_now
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        filtered_curr_linear_vel = self.speed_filter.filt(curr_linear_vel)
        linear_velocity_error = desired_linear_vel - filtered_curr_linear_vel
        throttle = self.throttle_controller.step(linear_velocity_error, time_passed)

        if throttle <= self.min_allowed_throttle and desired_linear_vel < filtered_curr_linear_vel:
            brake = 700. # TODO: DEBUG!
        else:
            brake = 0. 
            
        steering = self.yaw_controller.get_steering(
                        desired_linear_vel,
                        desired_angular_vel,
                        filtered_curr_linear_vel)

        return throttle, brake, steering
