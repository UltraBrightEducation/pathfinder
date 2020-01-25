import logging


class ManualDriveLaneFollower(object):
    def __init__(self, car):
        logging.info('Creating a ManualDriveLaneFollower...')
        self.car = car
        self.curr_steering_angle = 90

    def manual_drive(self, input):
        final_frame = self.steer(input)
        return final_frame

    def steer(self, input):
        if input == 'a':
            self.curr_steering_angle = max(self.curr_steering_angle - 5, 45)
        if input == 'd':
            self.curr_steering_angle = min(self.curr_steering_angle + 5, 135)
        if input == 's':
            self.curr_steering_angle = 90

        self.car.front_wheels.turn(self.curr_steering_angle)
        return self.curr_steering_angle
