#!/usr/bin/expect -f
# Pm2 run bash scripts
# Dinh Ngoc Duc
# 2023/10/22

sleep 30s

chmod +x myprompt.sh

gnome-terminal -- bash start_ui.sh 
gnome-terminal -e ~/musashi_ws/myprompt.sh