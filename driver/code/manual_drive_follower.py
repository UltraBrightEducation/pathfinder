import cv2
import numpy as np
import logging
import math
import datetime
import sys

_SHOW_IMAGE = False


class ManualDriveLaneFollower(object):

    def __init__(self, car=None):
        logging.info('Creating a ManualDriveLaneFollower...')
        self.car = car
        self.curr_steering_angle = 90

    def manual_drive(self, frame, input):
        # Main entry point of the lane follower
        show_image("orig", frame)
        
        #Following line may be deleted later
        #lane_lines, frame = detect_lane(frame)
        final_frame = self.steer(frame, input)

        return final_frame
        
        
    #handles img saving as well now
    def steer(self, frame, input):
        logging.debug('steering...')
        new_steering_angle = self.curr_steering_angle
        #compute new angle
        #new_steering_angle = compute_steering_angle(frame, input)
        if input == 'a':
            new_steering_angle = self.curr_steering_angle - 5
            #steering angle should never go below 0 or above 130
            if new_steering_angle < 45:
                new_steering_angle = 45
        if input == 'd':
            new_steering_angle = self.curr_steering_angle + 5
            #steering angle should never go below 0 or above 130
            if new_steering_angle > 135:
                new_steering_angle = 135
        if input == 's':
            new_steering_angle = 90
            
          
        #getting rid of turn stabilization for now
        #self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle)
        self.curr_steering_angle = new_steering_angle
        
        #actual steering operation
        if self.car is not None:
            #filename =  '~/DeepPiCar/models/lane_navigation/data/images' + datetime.datetime.now().strftime("%y/%m/%d %H:%M:%S.%f")
            
            self.car.front_wheels.turn(self.curr_steering_angle)
            
        #curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        
        #trying to save images with display heading lines, might revert to blank images later, move the following line up into the previous if block and change curr_heading_image to frame 
        #cv2.imwrite("%s__%03d.png" % (datetime.datetime.now().strftime("%y/%m/%d %H:%M:%S.%f"), self.curr_steering_angle), curr_heading_image)
        #show_image("heading", frame)

        return frame



#needs changes to remove lane lines from calculation
#getting rid of stabalization for now
'''def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle'''


############################
# Utility Functions
############################

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image



def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)



############################
# Test Functions
############################
def test_photo(file):
    land_follower = HandCodedLaneFollower()
    frame = cv2.imread(file)
    combo_image = land_follower.follow_lane(frame)
    show_image('final', combo_image, True)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_video(video_file):
    lane_follower = HandCodedLaneFollower()
    cap = cv2.VideoCapture(video_file + '.avi')

    # skip first second of video.
    for i in range(3):
        _, frame = cap.read()

    video_type = cv2.VideoWriter_fourcc(*'XVID')
    video_overlay = cv2.VideoWriter("%s_overlay.avi" % (video_file), video_type, 20.0, (320, 240))
    try:
        i = 0
        while cap.isOpened():
            _, frame = cap.read()
            print('frame %s' % i )
            combo_image= lane_follower.follow_lane(frame)
            
            cv2.imwrite("%s_%03d_%03d.png" % (video_file, i, lane_follower.curr_steering_angle), frame)
            
            cv2.imwrite("%s_overlay_%03d.png" % (video_file, i), combo_image)
            video_overlay.write(combo_image)
            cv2.imshow("Road with Lane line", combo_image)

            i += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        video_overlay.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    test_video('/home/pi/DeepPiCar/driver/data/tmp/video01')
    #test_photo('/home/pi/DeepPiCar/driver/data/video/car_video_190427_110320_073.png')
    #test_photo(sys.argv[1])
    #test_video(sys.argv[1])