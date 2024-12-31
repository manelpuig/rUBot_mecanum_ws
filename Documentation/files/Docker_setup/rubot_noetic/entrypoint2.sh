#!/bin/bash
set -e

# Configura l'entorn ROS
source /opt/ros/noetic/setup.bash
source /root/rUBot_mecanum_ws/devel/setup.bash
cd /root/rUBot_mecanum_ws
chmod -R +x *

# Espera que el servei ROS Master estigui disponible (opcional)

# Executa el comandament final
exec "$@"
