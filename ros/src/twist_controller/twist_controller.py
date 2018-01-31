
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy
from pid import PID
from lowpass import LowPassFilter

class Controller(object):
    def __init__(self, *args, **kwargs):

        # TODO: Implement
        #pass

        kp = 2.01
        ki = 0.001
        kd = 0.01


        kp = 1.6
        ki = 0.0
        kd = 0.001

        #kp = 3.03
        #kd = 0.012
        mn= -1.0
        mx= 1.0

        self.pid_controller = PID(kp, ki, kd, mn, mx)
        ts = 1.0/ 50.0
        tau = 1.0/1000 #default value
        tau = (7.0/8.0) * ts * 0.632
        self.throttle_filter = LowPassFilter(tau, ts)
        self.steer_filter = LowPassFilter(tau, ts)

    def control(self, *args, **kwargs):

        current_linear_velocity = kwargs['current_vel']
        target_linear_velocity = kwargs['target_vel']
        target_angular_velocity = kwargs['target_angular_vel']
        yaw_controller = kwargs['yaw_control']


        max_brake = (GAS_DENSITY * kwargs['fuel_capacity'] + kwargs['vehicle_mass']) * \
                        kwargs['decel_limit'] * kwargs['wheel_radius']

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        actual_target_vel = target_linear_velocity

        if 0:
            if target_linear_velocity < 0.015: #to protect against small movement
                target_linear_velocity = 0

        #rospy.loginfo("target velocity: %f %f %f", actual_target_vel, target_linear_velocity, current_linear_velocity)

        error = (target_linear_velocity - current_linear_velocity)

        if 0:
            #ease on throttle if current is trying to meet the target_linear_velocity
            #but brake quickly if the current exceeds target
            if abs(error) < 0.01: #error < 0.05  and error > -0.0001 :
                error = 0

        #rospy.loginfo("error:%f", error)
        #rospy.loginfo("kd:%f %f %f", self.pid_controller.kp, self.pid_controller.kd, self.pid_controller.ki)

        noise_throttle = self.pid_controller.step(error, sample_time=1.0/50.0)

        #filter the throttle
        throttle = self.throttle_filter.filt(noise_throttle)
        #throttle = noise_throttle # for time being

        noise_steering = yaw_controller.get_steering(target_linear_velocity,
                                               target_angular_velocity,
                                              current_linear_velocity )

        steering = self.steer_filter.filt(noise_steering)

        brake = 0.0

        actual_th = throttle
        if   throttle < 0:
            brake = throttle * max_brake
            throttle = 0

        #rospy.loginfo("th:%f steer: %f brake:%f", actual_th, steering, brake)

        return throttle, brake, steering


    #    return 1., 0., 0.
