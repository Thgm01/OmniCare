blue='\e[0;34m'
green='\e[0;32m'
yellow='\e[1;33m'
red='\e[0;31m'
NC='\e[0m' #No Color

echo -e "${blue} STARTING JETSON CONFIGURATION ${NC}"

git submodules init
git submodules update

sudo apt update & sudo apt upgrade -y
sudo apt install python3-pip


if [ -z "$(pip show jetson-stats)" ]; then
    sudo pip3 install -U jetson-stats
fi 

if [ -z "$(pip show ultralytics)" ]; then
    pip install ultralytics[export]
    if [ -n "$(pip show ultralytics)" ]; then
        sudo reboot
    fi
fi 

wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev cuda-toolkit-12-4 cuda-compat-12-4
sudo ln -sfn /usr/local/cuda-12.4 /usr/local/cuda

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y python3-pip libjpeg-dev libpng-dev libtiff-dev

pip install https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

pip install "numpy<2.0" 
git clone https://github.com/pytorch/vision.git torchvision
cd torchvision
git checkout v0.20.0
sudo python3 setup.py install
pip uninstall torchvision==0.21.0

pip install onnx2tf==1.26.3
pip install onnx_graphsurgeon
pip install sng4onnx
pip install tflite_support

#################################################################
#                       _                                       #
#                      / \   _ __  _ __  ___                    #
#                     / _ \ | '_ \| '_ \/ __|                   #
#                    / ___ \| |_) | |_) \__ \                   #
#                   /_/   \_\ .__/| .__/|___/                   #
#                           |_|   |_|                           #
#################################################################

# Chromium Browser
if [ -z "$(which chromium-browser)" ]; then
    echo -e "${blue}Instaling Chromium Browser${NC}"
    sudo apt install chromium-browser -y
    
    if [ -z "$(which chromium-browser)" ]; then
        echo -e "${green}Chromium Browser Installed Successfully${NC}"
    else
        echo -e "${red}ERROR: Chromium Browser Instalation${NC}"
    fi
else 
    echo -e "${green}Chromium Browser Already Installed${NC}"
fi

# VS Code
if [ -z "$(which code)" ]; then
    echo -e "${blue}Instaling VS Code${NC}"
    wget -O code.deb "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64"
    sudo dpkg -i code.deb
    sudo apt --fix-broken install -y
    rm -rf code.deb

    if [ -z "$(which code )" ]; then
        echo -e "${red}ERROR: VS Code Instalation${NC}"
    else
        echo -e "${green}VS Code Installed Successfully${NC}"
    fi
else 
    echo -e "${green}VS Code Already Installed${NC}"
fi


#################################################################
#                                _                              #
#                        _______| |__                           #
#                       |_  / __| '_ \                          #
#                        / /\__ \ | | |                         #
#                       /___|___/_| |_|                         #
#                                                               #
#################################################################
sudo apt update & sudo apt upgrade -y
sudo apt install zsh curl -y


# Instalacao da fonte para usar no terminal
if [ -z "$(ls -a ~/.local/share | grep fonts)" ]; then
    echo -e "${blue}Instaling Nerd Fonts${NC}"
    wget -P ~/.local/share/fonts https://github.com/ryanoasis/nerd-fonts/releases/download/v3.0.2/JetBrainsMono.zip \
    && cd ~/.local/share/fonts \
    && unzip JetBrainsMono.zip \
    && rm JetBrainsMono.zip \
    && fc-cache -fv
fi

# Caso já exista não instala o Oh my zsh
if [ -n "$(ls -a ~/ | grep .oh-my-zsh)" ]; then
    echo "OH-MY-ZSH Já está instalado"
else
    echo -e "${blue}Instaling Oh My Zsh${NC}"
    wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
    # Comentar a linha de inicialização automática para não travar o resto da instalação
    sed -i '/exec zsh -l/s/^/#/' install.sh
    sh install.sh
    rm -f install.sh
fi

# Configuração de plugins do zsh
git clone https://github.com/zsh-users/zsh-autosuggestions.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
sudo apt install zoxide

# Definindo o zsh como padrão
chsh -s $(which zsh)

# Para o Setup padrao, modificar as linhas
# plugins=(git zsh-autosuggestions)
# E inserir a linha abaixo no final do arquivo
# eval "$(zoxide init zsh)"


#################################################################
#                    ____   ___  ____ ____                      #
#                   |  _ \ / _ \/ ___|___ \                     #
#                   | |_) | | | \___ \ __) |                    #
#                   |  _ <| |_| |___) / __/                     #
#                   |_| \_\\___/|____/_____|                    #
#                                                               #
#################################################################
    if [ -z "$(ls -a /opt | grep ros)" ]; then
        echo -e "${blue}Instaling ROS2 Humble${NC}"

        # Instalar o ROS Humble
        sudo apt install software-properties-common -y
        sudo add-apt-repository universe -y
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update && sudo apt upgrade -y
        sudo apt install ros-humble-desktop -y
        sudo apt install ros-dev-tools -y
        sudo apt install python3-rosdep -y

        # Colcon Setup
        sudo apt install python3-colcon-common-extensions -y

        # Intalar Onshape to Robot
        pip install onshape-to-robot

        # Source
        if [[ "$SHELL" == *"zsh"* ]]; then
            source /opt/ros/humble/setup.zsh
        else
            source /opt/ros/humble/setup.bash
        fi
        
        # Instalando dependencias do ROS2
        rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y

        source /software/src/omnicare_navigation/rplidar_ros/scripts/create_udev_rules.sh

        echo -e "${green}ROS Installed Successfully${NC}"
    fi



    # remover as linhas autoload -U +x compinit && compinit
    # e autoload -U +x bashcompinit && bashcompinit dos arquivos abaixo:
    #
    # sudo vim /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    # sudo vim /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.zsh
    # sudo vim /opt/ros/humble/share/ament_index_python/environment/ament_index-argcomplete.zsh
    # sudo vim /opt/ros/humble/share/rosidl_cli/environment/rosidl-argcomplete.zsh

#################################################################
#                               _       _                       #
#                 ___  ___ _ __(_)_ __ | |_ ___                 #
#                / __|/ __| '__| | '_ \| __/ __|                #
#                \__ \ (__| |  | | |_) | |_\__ \                #
#                |___/\___|_|  |_| .__/ \__|___/                #
#                                |_|                            #
#################################################################
    if ! grep -q 'source /home/robot/TCC/scripts.sh' ~/.bashrc; then
        echo 'source /home/robot/TCC/scripts.sh' >> ~/.bashrc
    fi

    if [ -n "$(ls -a ~/ | grep .zshrc)" ]; then
        if ! grep -q 'source /home/robot/TCC/scripts.sh' ~/.zshrc; then
            echo 'source /home/robot/TCC/scripts.sh' >> ~/.zshrc
        fi
    fi
