FROM ubuntu:22.04

WORKDIR /app

# Move the current folder inside the container
COPY . .

RUN apt-get update && apt-get install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# To avoid strange errors
RUN apt-get update && apt-get install -y tzdata
ENV TZ=Europe/Athens
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y ros-humble-ros-base ros-dev-tools
RUN apt-get install -y libgl1 libglib2.0-0
SHELL ["/bin/bash", "-c"]

# Download python packages
RUN apt-get install -y python3 python3-pip
RUN apt-get clean

# It is crucial to download ultralytics only for CPU; otherwise, the disk requirements are enormous.
RUN pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu && pip cache purge
RUN pip install --no-cache-dir ultralytics && pip cache purge
