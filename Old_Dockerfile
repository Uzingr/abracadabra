FROM osrf/ros:humble-desktop-full-jammy
# Aggiorna il registro dei pacchetti e installa locales
RUN apt-get update && apt-get install -y locales cmake build-essential

# Imposta l'ambiente per evitare input interattivi
ENV DEBIAN_FRONTEND=noninteractive

# Aggiorna i pacchetti e installa le dipendenze necessarie
RUN apt-get update && apt-get install --no-install-recommends -y \
    software-properties-common \
    && add-apt-repository ppa:ubuntugis/ppa \
    && apt-get update \
    && apt-get install --no-install-recommends -y \
        build-essential \
        ca-certificates \
        cmake \
        doxygen \
        g++ \
        git \
        libeigen3-dev \
        libgdal-dev \
        libpython3-dev \
        python3 \
        python3-pip \
        python3-matplotlib \
        python3-tk \
        lcov \
        libgtest-dev \
        libtbb-dev \
        swig \
        libgeos-dev

# Installa gcovr tramite pip
RUN python3 -m pip install gcovr

# Clona il repository Fields2Cover
RUN git clone https://github.com/Marnonel6/Fields2Cover.git

# Imposta la directory di lavoro
WORKDIR /Fields2Cover

# Crea la directory build e cambia la directory di lavoro
RUN mkdir -p build

# Cambia la directory di lavoro nella directory build
WORKDIR /Fields2Cover/build

# Esegui cmake e make
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) \
    && make install

# Genera le localizzazioni
RUN locale-gen en_US en_US.UTF-8
# Imposta le localizzazioni come predefinite
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# Imposta la variabile di ambiente LANG
ENV LANG=en_US.UTF-8
# Installa software-properties-common e aggiunge il repository universe
RUN apt-get install -y software-properties-common && \
    add-apt-repository universe
# Aggiorna il registro dei pacchetti e installa curl
RUN apt-get update && apt-get install -y curl unzip nano inetutils-ping
# Scarica la chiave di autenticazione ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Aggiunge il repository ROS 2 al file di configurazione apt
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Aggiorna il registro dei pacchetti e aggiorna il sistema
#RUN apt-get update && apt-get upgrade -y

# Installare QgroundControl su Lattepanda (companion computer)
#COPY QGroundControl.AppImage  /QGroundControl.AppImage

WORKDIR /

# Clona il github per il drone keyboard mission per il setup del drone control computer
RUN git clone https://github.com/Marnonel6/ROS2_offboard_drone_control.git
# Clonare ROS2 package ???   osprey_code_pull
# Installa i pacchetti e installa le dipendenze necessarie
#RUN apt-get update && apt-get install -y cmake build-essential
# Clona il repositort
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
# Imposta la directory di lavoro
WORKDIR /Micro-XRCE-DDS-Agent
# Crea la directort build
RUN mkdir build
# Cambia la directory di lavoro nella directory build 
WORKDIR /Micro-XRCE-DDS-Agent/build
# Esegui cmake e make
RUN cmake .. \
 && make
#Esegui make install e ldCondif
RUN make install \
	&& ldconfig /usr/local/lib/
# Imposta la directory di lavoro di default per il container
WORKDIR /

# Installa ROS 2 desktop e gli strumenti di sviluppo
#RUN apt-get install -y ros-humble-desktop ros-dev-tools
# Configura l'ambiente ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# Sorgente l'ambiente ROS
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
#Avvia la shell di bash all'interno del container
CMD ["/bin/bash"]
