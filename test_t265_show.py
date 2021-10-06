# This assumes .so file is found on the same directory
# https://markku.ai/post/realsense-t265/
import pyrealsense2 as rs

# Prettier prints for reverse-engineering
from pprint import pprint
import numpy as np
import cv2
import t265_example2

# Get realsense pipeline handle
pipe2 = rs.pipeline()

# Configure the pipeline
config2 = rs.config()
config2.enable_device('119622110606') # T265 camera 119622110606


# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))

# Enable streams you are interested in
config2.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
config2.enable_stream(rs.stream.fisheye, 1) # Left camera
config2.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe2.start(config2)

t265_example2
try:
    for _ in range(10):
        frames = pipe2.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)
        color_t265 = cv2.applyColorMap(cv2.convertScaleAbs(frames, alpha=0.03), cv2.COLORMAP_JET)
        imshow(color_t265)
        # Positional data frame
        pose = frames.get_pose_frame()
        if pose:
            pose_data = pose.get_pose_data()
            print('\nFrame number: ', pose.frame_number)
            print('Position: ', pose_data.translation)
            print('Velocity: ', pose_data.velocity)
            print('Acceleration: ', pose_data.acceleration)
            print('Rotation: ', pose_data.rotation)
            
finally:
    pipe2.stop()

"""
result of 
pprint(dir(rs.stream))
['__class__',
 '__delattr__',
 '__dir__',
 '__doc__',
 '__entries',
 '__eq__',
 '__format__',
 '__ge__',
 '__getattribute__',
 '__getstate__',
 '__gt__',
 '__hash__',
 '__index__',
 '__init__',
 '__init_subclass__',
 '__int__',
 '__le__',
 '__lt__',
 '__members__',
 '__module__',
 '__ne__',
 '__new__',
 '__reduce__',
 '__reduce_ex__',
 '__repr__',
 '__setattr__',
 '__setstate__',
 '__sizeof__',
 '__str__',
 '__subclasshook__',
 'accel',
 'any',
 'color',
 'confidence',
 'depth',
 'fisheye',
 'gpio',
 'gyro',
 'infrared',
 'name',
 'pose',
 'value']
"""