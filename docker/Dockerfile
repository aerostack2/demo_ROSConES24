FROM aerostack2/humble:1.1.2 

RUN apt update && apt-get install ros-humble-as2-platform-crazyflie -y

RUN echo "set -g mouse on" > ~/.tmux.conf 

# Clone demo project
WORKDIR /root/
RUN git clone https://github.com/aerostack2/demo_ROSConES24.git

# Crazyflie dependencies 
RUN apt install git python3-pip libxcb-xinerama0 libxcb-cursor0 -y
RUN pip3 install --upgrade pip
RUN pip3 install cfclient
RUN apt install udev usbutils -y

# RUN groupadd plugdev
RUN usermod -aG plugdev root
COPY ./99-bitcraze.rules /etc/udev/rules.d/99-bitcraze.rules

WORKDIR /root/demo_ROSConES24