blue='\e[0;34m'
green='\e[0;32m'
yellow='\e[1;33m'
red='\e[0;31m'
NC='\e[0m' #No Color

echo -e "${blue} STARTING JETSON CONFIGURATION ${NC}"

sudo apt update & sudo apt upgrade -y

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
    rm -rf 

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
if [ -z "$(ls -a ~/.local/share | grep fonts)"]; then
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


if [ -z "$(ls -a /opt | grep ros)"]; then
    echo -e "${blue}Instaling ROS2 Humble${NC}"

    # Instalar o ROS Humble
    sudo apt install python3-pip
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade -y
    sudo apt install ros-humble-desktop -y
    sudo apt install ros-dev-tools -y

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

    echo -e "${green}ROS Installed Successfully${NC}"