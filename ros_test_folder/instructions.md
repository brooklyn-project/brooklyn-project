# Setup

(All of this should be done in the Docker container). 

1. Install `video_stream_opencv`. 

```
sudo apt install ros-noetic-video-stream-opencv
```


2. Download [`default_0.bag`](https://drive.google.com/file/d/1hA3eQfZeKkTRLLdNMlE9FyVdY20Zobzh/view?usp=share_link) and [`test_video.mp4`](https://drive.google.com/file/d/18vGyEkqzx62KtMSGWotWfNlYZbbWUAhz/view?usp=share_link) and place them into this folder. 

# Usage

1. Run the following command

```
cd <to this folder>
roslaunch test_data.launch
```

This will start publishing all of the data that we'll be consuming (including image data). In another terminal, you can run `rostopic list` to see all of the topics that are available.

2. From here, open another terminal to test whatever piece(s) of code you want to run. For example:

```
rosrun target_recognition target_recognition_node
```
or
```
rosrun map_generation map_generation_node.py
```
or both. From here you can debug with things like `rostopic list`, `rostopic echo`, `rostopic info`, and `rosnode info`.