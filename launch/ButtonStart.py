from gpiozero import LED, Button
from signal import pause
#from picamera2.encoders import H264Encoder, Quality
#from picamera2 import Picamera2, Preview
import datetimeled = LED(23)
import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["record_data.launch"])


#button = Button(24)
#camera = Picamera2()
#video_config = camera.create_video_configuration(main={"size": (1920, 1080)}, lores={"size": (640, 480)}, display="lores")
#camera.configure(video_config)
#encoder = H264Encoder()
#output = "/home/mino/Desktop/" + datetime.datetime.now().strftime('%Y-%m-%d%H:%M:%S') + ".h264"

def start():
    led.blink(0.25,0.25)
    #camera.start_preview(Preview.OTGL)
    #camera.start_recording(encoder, output, Quality.VERY_HIGH)
	
	# Start ROS
	launch.start()
	rospy.loginfo("Started.")
	
def end():
    led.off()
    #camera.stop_recording()
    #camera.stop_preview()
	button.when_pressed = start
	
	# Stop ROS
	rospy.loginfo("Shutting down.")
	launch.shutdown()
	
	
button.when_released = end
led.on()
pause()
