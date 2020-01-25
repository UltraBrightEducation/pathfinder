import logging
import time

import picar
import cv2
import datetime
import getch

from argparse import ArgumentParser

from lane_followers.lane_follower_utils import show_image
from lane_followers.manual_drive_follower import ManualDriveLaneFollower
from pathlib import Path

_SHOW_IMAGE = True
logging.basicConfig(level=logging.DEBUG, format='%(levelname)-5s:%(asctime)s: %(message)s')


class DeepPiCar(object):

    __INITIAL_SPEED = 0
    __SCREEN_WIDTH = 320
    __SCREEN_HEIGHT = 240
    _INTERVAL = 500

    def __init__(
            self,
            mode='manual',
            object_detection=False,
            output_dir=Path.home() / 'output_data',
            save_image=False,
            display_image=False
    ):
        """ Init camera and wheels"""
        logging.info('Creating a DeepPiCar...')

        picar.setup()
        logging.debug('Set up camera')
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(3, self.__SCREEN_WIDTH)
        self.camera.set(4, self.__SCREEN_HEIGHT)

        self.pan_servo = picar.Servo.Servo(1)
        self.pan_servo.offset = -30  # calibrate servo to center
        self.pan_servo.write(90)

        self.tilt_servo = picar.Servo.Servo(2)
        self.tilt_servo.offset = 20  # calibrate servo to center
        self.tilt_servo.write(90)

        logging.debug('Set up back wheels')
        self.back_wheels = picar.back_wheels.Back_Wheels()
        self.back_wheels.speed = 0  # Speed Range is 0 (stop) - 100 (fastest)

        logging.debug('Set up front wheels')
        self.front_wheels = picar.front_wheels.Front_Wheels()
        self.front_wheels.turning_offset = 15  # calibrate servo to center
        self.front_wheels.turn(90)  # Steering Range is 45 (left) - 90 (center) - 135 (right)

        self.manual_driver = ManualDriveLaneFollower(self)
        self.save_image = save_image
        self.display_image = display_image
        self._run_id = datetime.datetime.now().strftime("%y%m%d_%H%M%S")

        if mode == 'CNN':
            from lane_followers.ml_lane_follower import MLLaneFollower
            self.lane_follower = MLLaneFollower(self)
            self.image_dir = output_dir / 'images' / 'ml_follower' / self._run_id
        elif mode == 'CV2':
            from lane_followers.rules_based_lane_follower import RulesBasedLaneFollower
            self.lane_follower = RulesBasedLaneFollower(self)
            self.image_dir = output_dir / 'images' / 'rules_follower' / self._run_id
        else:
            self.image_dir = output_dir / 'images' / 'manual' / self._run_id

        self.image_dir.mkdir(parents=True, exist_ok=True)

        if object_detection:
            from objec_detection.objects_on_road_processor import ObjectsOnRoadProcessor
            self.traffic_sign_processor = ObjectsOnRoadProcessor(self)
        else:
            self.traffic_sign_processor = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

        video_dir = output_dir / 'videos'
        video_dir.mkdir(parents=True, exist_ok=True)
        video_path = video_dir / f'{self._run_id}.avi'
        self.video_recorder = self.create_video_recorder(str(video_path))
        logging.info('Created an AutoBot')

    def create_video_recorder(self, path):
        return cv2.VideoWriter(path, self.fourcc, 20.0, (self.__SCREEN_WIDTH, self.__SCREEN_HEIGHT))

    def __enter__(self):
        """ Entering a with statement """
        return self

    def __exit__(self, _type, value, traceback):
        """ Exit a with statement"""
        if traceback is not None:
            logging.error('Exiting with statement with exception %s' % traceback)

        self.cleanup()

    def cleanup(self):
        """ Reset the hardware"""
        logging.info('Stopping the car, resetting hardware.')
        self.back_wheels.speed = 0
        self.front_wheels.turn(90)
        self.camera.release()
        self.video_recorder.release()
        cv2.destroyAllWindows()

    def self_drive(self, speed=__INITIAL_SPEED):
        """ Main entry point for self-driving mode

        Keyword arguments:
        speed -- speed of back wheel, range is 0 (stop) - 100 (fastest)
        """

        logging.info('Starting to drive at speed %s...' % speed)
        self.back_wheels.speed = speed
        i = 0
        start_ms = time.time()
        while self.camera.isOpened():
            _, image_lane = self.camera.read()
            image_objs = image_lane.copy()
            image_lane = self.lane_follower.follow_lane(image_lane)
            if self.display_image:
                show_image('Lane Lines', image_lane)
            if self.save_image:
                if time.time() - start_ms > self._INTERVAL:
                    output_path = self.image_dir / f'{self._run_id}_{i}_{self.manual_driver.curr_steering_angle}.png'
                    cv2.imwrite(str(output_path), image_lane)
                    i += 1
                    start_ms = time.time()

            if self.traffic_sign_processor:
                image_objs = self.traffic_sign_processor.process_objects_on_road(image_objs)
                self.video_recorder.write(image_objs)
                show_image('Detected Objects', image_objs)
            else:
                self.video_recorder.write(image_lane)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
                break

    def manual_drive(self, speed=__INITIAL_SPEED):
        logging.info('Starting to drive at speed %s...' % speed)
        self.back_wheels.speed = speed
        i = 0
        start_ms = time.time()
        while self.camera.isOpened():
            _, image = self.camera.read()
            if self.display_image:
                show_image('Manual Drive', image)

            if cv2.waitKey(1):
                input = getch.getch()
                if input == 'q':
                    self.cleanup()
                    break
                _ = self.manual_driver.manual_drive(input)

            if self.save_image:
                if time.time() - start_ms > self._INTERVAL:
                    output_path = self.image_dir / f'{self._run_id}_{i}_{self.manual_driver.curr_steering_angle}.png'
                    cv2.imwrite(str(output_path), image)
                    i += 1
                    start_ms = time.time()


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        "--mode", default='manual', type=str, help="Driving mode, select from 'manual', 'CNN', 'CV2''"
    ),
    parser.add_argument("--init-speed", default=20, type=int, help="Initial speed of the car")
    parser.add_argument("-o", "--object-detection", action="store_true", help="Perform object detection and tasks")
    parser.add_argument("-s", "--save-image", action="store_true", help="Save camera images")
    parser.add_argument("-d", "--display-image", action="store_true", help="Display camera images")
    args, _ = parser.parse_known_args()

    with DeepPiCar(
            mode=args.mode,
            object_detection=args.object_detection,
            save_image=args.save_image,
            display_image=args.display_image
    ) as car:
        if args.mode == 'manual':
            car.manual_drive(speed=args.init_speed)
        elif args.mode in ['CNN', 'CV2']:
            car.self_drive(speed=args.init_speed)
