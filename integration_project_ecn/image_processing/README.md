# image_processing
This package is designed to use color detection on images to detect the main color of an image and returns the masscenter of the detected contour

## center_circle_detector

a node that uses the image topic /robot/camera1/image_raw and returns a 3D point published on /masscenter with the position of the circumscribed circle of the detected contour on the x and y coordinates, and the timestamp on the z coordinate.

## mass_center_detector

a node that uses the image topic /robot/camera1/image_raw and returns a 3D point published on /masscenter with the position of the mass center of the detected contour on the x and y coordinates, and the timestamp on the z coordinate.
