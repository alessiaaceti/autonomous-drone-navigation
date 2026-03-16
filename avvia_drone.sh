#!/bin/bash
echo "Avvio sistema del drone"

# 1. Avvia Gazebo e PX4 (sostituisci col tuo comando/cartella esatta)
echo "Avvio Gazebo..."
gnome-terminal --title="Gazebo" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth; exec bash"

# Aspetta 8 secondi per dare tempo alla simulazione 3D di caricarsi
sleep 8

# 2. Avvia il ponte di comunicazione (MicroXRCEAgent)
echo "Avvio MicroXRCEAgent..."
gnome-terminal --title="Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# Aspetta 3 secondi
sleep 3

# 3. Avvia Nav2
echo "Avvio Nav2 e RViz..."
gnome-terminal --title="Nav2" -- bash -c "cd ~/autonomous-drone-navigation && ros2 launch ./launch/start_nav2.launch.py; exec bash"

echo "Tutto lanciato! Controlla le finestre che si sono aperte."
