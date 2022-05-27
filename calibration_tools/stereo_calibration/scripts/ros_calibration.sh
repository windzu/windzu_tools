rosrun camera_calibration cameracalibrator.py --size 5x8 \
--square 0.0214 \
left:=/master_publisher/image_raw \
right:=/slaver_publisher/image_raw 
