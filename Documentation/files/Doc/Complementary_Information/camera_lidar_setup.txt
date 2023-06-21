#!/bin/bash

echo -e "\e[0mIniciando configuración de la plataforma GoPiGo.\e[0m"


###################################################################################################
# Instalamos drivers del robot.
###################################################################################################

echo -e "\e[1;33mInstalando los drivers de GoPiGo...\e[0m"

# Instalamos la libreria GoPiGo.

su - $SUDO_USER -c "curl -kL dexterindustries.com/update_gopigo3 | bash -s -- --user-local --bypass-gui-installation"
su - $SUDO_USER -c "curl -kL dexterindustries.com/update_sensors | bash -s -- --user-local --bypass-gui-installation"

echo -e "\e[1;32mDrivers de GoPiGo instalados con éxito.\e[0m"


###################################################################################################
# Configuracion de la camara.
###################################################################################################

echo -e "\e[1;33mInstalando la cámara...\e[0m"

# Habilitamos el acceso a la camara.
echo 'start_x=1' >> /boot/config.txt

# Instalamos el modulo python de la camara.
su - $SUDO_USER -c "pip install picamera"

# Necesario para el package raspicam.
apt-get install -y libyaml-cpp-dev
apt-get install -y libogg-dev libvorbis-dev libtheora-dev

echo -e "\e[1;32mCámara instalada con éxito. Reiniciando Raspberry Pi.\e[0m"


###################################################################################################
# Instalamos ROS Melodic
###################################################################################################

echo -e "\e[1;33mInstalando ROS...\e[0m"

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update
apt-get upgrade -y

apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

rosdep init

su - $SUDO_USER -c "bash" << EOF

rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator robot --rosdistro melodic --deps --wet-only --tar > melodic-robot-wet.rosinstall

wstool init -j2 src melodic-robot-wet.rosinstall

rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster

EOF

cd $(eval echo ~$SUDO_USER)/ros_catkin_ws
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2

su - $SUDO_USER -c "echo \"source /opt/ros/melodic/setup.bash\" >> ~/.bashrc"

echo -e "\e[1;32mROS instalado con éxito.\e[0m"


###################################################################################################
# Instalamos OpenCV
###################################################################################################

su - $SUDO_USER -c "mkdir ~/opencv"

apt-get install -y build-essential cmake unzip pkg-config
apt-get install -y libjpeg-dev libpng-dev libtiff-dev
apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
apt-get install -y libxvidcore-dev libx264-dev
apt-get install -y libgtk-3-dev
apt-get install -y libcanberra-gtk*
apt-get install -y libatlas-base-dev gfortran
apt-get install -y python3-dev

su - $SUDO_USER -c "bash" << EOF
        cd ~/opencv/
        wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.12.zip
        unzip opencv.zip
        wget https://bootstrap.pypa.io/get-pip.py
        cd opencv-3.4.12
        mkdir build
EOF


cd $(eval echo ~$SUDO_USER)/opencv
python get-pip.py
python3 get-pip.py

su - $SUDO_USER -c "bash" << EOF
        cd ~/opencv/opencv-3.4.12/build
        cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
        make -j1
EOF

cd $(eval echo ~$SUDO_USER)/opencv/opencv-3.4.12/build
make install
ldconfig

find /usr/local/lib -type f -name "cv2.*.so" -exec sh -c 'x="{}"; echo mv "$x" "$(dirname ${x})/cv2.so"' \;

echo -e "\e[1;32mOpenCV instalado con éxito. Reiniciando Raspberry Pi.\e[0m"


###################################################################################################
# Salimos y reiniciamos
###################################################################################################

reboot
