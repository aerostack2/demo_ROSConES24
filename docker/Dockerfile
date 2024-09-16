FROM aerostack2/humble:1.1.2 

RUN echo "set -g mouse on" > ~/.tmux.conf 

# Clone demo project
WORKDIR /root/
RUN git clone https://github.com/aerostack2/demo_ROSConES24.git

WORKDIR /root/demo_ROSConES24
