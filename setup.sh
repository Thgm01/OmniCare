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

# TODO: Incluir como instalar a fonte para o tema do zsh