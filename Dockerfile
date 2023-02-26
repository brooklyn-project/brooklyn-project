FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get install -y software-properties-common
RUN apt-add-repository ppa:fish-shell/release-3
RUN apt-get update
RUN apt-get install -y git python3-pip fish
COPY requirements.txt /opt/requirements.txt
RUN pip install -r /opt/requirements.txt

# Add SSH keys
RUN mkdir -p /root/.ssh
ADD ./id_rsa /root/.ssh/id_rsa
RUN chmod 600 /root/.ssh/id_rsa
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# Add catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Add brooklyn-project
# WORKDIR /root/catkin_ws/src
# RUN git clone git@github.com:brooklyn-project/brooklyn-project.git

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "DONE!"