import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys

file = 'outdoors.bag'
dir =os.path.dirname(os.path.realpath(__file__))
bag_filename = os.path.join(dir, '')
np.set_printoptions(threshold=sys.maxsize)

def main():
    # if not os.path.exists(args.directory):
    #     os.mkdir(args.directory)
    try:
        config = rs.config()
        config.enable_device_from_file(file)
        pipeline = rs.pipeline()
        # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        pipeline.start(config)
        # create opencv window
        cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)
        # create colorizer object
        colorizer = rs.colorizer()
        i = 0
        ### from ros docs
        while True:
            # get frameset of depth
            frames = pipeline.wait_for_frames()
            # get depth frame
            depth_frame = frames.get_depth_frame()
            # Colorize depth frame to jet colormap
            depth_color_frame = colorizer.colorize(depth_frame)
            # Convert depth_frame to numpy array to render image in opencv
            depth_color_image = np.asanyarray(depth_color_frame.get_data())

            # get color frame and convert it to a numpy array
            color_frame = frames.get_color_frame()

            # Render image in opencv window
            # if i==0:
            #     print(depth_color_image)
            #     i+=1
            print(depth_frame.get_distance(150,150))
            cv2.imshow("Depth Stream", depth_color_image)
            key = cv2.waitKey(1)
            # If pressed, escape exit program
            if key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pass

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
    # parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    # args = parser.parse_args()
    main()