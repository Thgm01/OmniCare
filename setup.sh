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
        echo -e "${green}VS Code Installed Successfully${NC}"
    else
        echo -e "${red}ERROR: VS Code Instalation${NC}"
    fi
else 
    echo -e "${green}VS Code Already Installed${NC}"
fi

# #################################################################
# #                                _                              #
# #                        _______| |__                           #
# #                       |_  / __| '_ \                          #
# #                        / /\__ \ | | |                         #
# #                       /___|___/_| |_|                         #
# #                                                               #
# #################################################################
# sudo apt update & sudo apt upgrade -y
# sudo apt install zsh curl -y

# # Caso já exista não instala o Oh my zsh
# if [ -n "$(ls -a ~/ | grep .oh-my-zsh)" ]; then
#     echo "OH-MY-ZSH Já está instalado"
# else
#     # Download e instalação
#     wget -P "$folder_path" https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
#     # Comentar a linha de inicialização automática para não travar o resto da instalação
#     sed -i '/exec zsh -l/s/^/#/' "$folder_path"/install.sh
#     sh "$folder_path"/install.sh
#     rm -f "$folder_path"/install.sh
# fi

# # Configuração de plugins do zsh
# git clone https://github.com/zsh-users/zsh-autosuggestions.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
# sudo apt install zoxide

# # Passando minhas configurações do zsh
# sudo rm ~/.zshrc
# cp "$folder_path"/zshsetup.txt ~/.zshrc

# # Definindo o zsh como padrão
# chsh -s $(which zsh)




# # ROS Humble Instalation
# sudo apt update && sudo apt install locales -y
# sudo locale-gen en_US en_US.UTF-8
# sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8

# sudo apt install software-properties-common -y
# sudo add-apt-repository universe -y
# sudo apt update && sudo apt install curl -y
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# sudo apt update -y
# sudo apt upgrade -y

# sudo apt install ros-humble-desktop -y
# sudo apt install ros-dev-tools -y

# echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc

# # Install rqt
# sudo apt update -y
# sudo apt install '~nros-humble-rqt*' -y

# # Install and setup Colcon
# sudo apt install python3-colcon-common-extensions -y
# echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
# echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# # Install Moveit2
# # fazer depois

