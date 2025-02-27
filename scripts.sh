#!/usr/bin/env bash

#################################################################
#                    ____   ___  ____ ____                      #
#                   |  _ \ / _ \/ ___|___ \                     #
#                   | |_) | | | \___ \ __) |                    #
#                   |  _ <| |_| |___) / __/                     #
#                   |_| \_\\___/|____/_____|                    #
#                                                               #
#################################################################

if [[ "$SHELL" == *"zsh"* ]]; then
	source /opt/ros/humble/setup.zsh
else
	source /opt/ros/humble/setup.bash
fi

alias rr='ros2 run'
alias rl='ros2 launch'

alias rte='ros2 topic echo'
alias rtl='ros2 topic list'

alias cb='colcon build --symlink-install'

if [[ "$SHELL" == *"zsh"* ]]; then
	alias si='source install/setup.zsh'
else
	alias si='source install/setup.bash'
fi

             
#################################################################
#                           ____ _ _                            #
#                          / ___(_) |_                          #
#                         | |  _| | __|                         #
#                         | |_| | | |_                          #
#                          \____|_|\__|                         #
#                                                               #
#################################################################

git_commit() {
  if [ -z "$1" ]; then
    echo "Por favor, forneça uma mensagem de commit."
    return 1
  fi
  git add .
  git commit -m "$1"
  git push origin main
}

alias gc='git_commit'
alias gpl='git pull'
alias gps='git push'