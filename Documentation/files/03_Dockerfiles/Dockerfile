# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:noetic-desktop-full

RUN sudo apt-get clean
RUN apt-get update
RUN sudo apt-get upgrade -y
RUN apt-get install -y git && apt-get install -y python3-pip
RUN echo "git and pip Installed"
RUN apt install -y gedit
RUN apt install -y gedit-plugins
RUN echo "gedit Installed"
RUN apt install nautilus -y
RUN apt install gnome-terminal -y
RUN sudo apt install nautilus-actions gnome-terminal -y
RUN echo "Nautilus File manager Installed"


RUN cd /home/

RUN echo "ALL Done"