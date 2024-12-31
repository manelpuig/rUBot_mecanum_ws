#!/bin/bash
set -e

# Configura l'entorn ROS
source /opt/ros/noetic/setup.bash
source /root/rUBot_mecanum_ws/devel/setup.bash
cd /root/rUBot_mecanum_ws
chmod -R +x *

# Espera que el servei ROS Master estigui disponible (opcional)
until nc -z ${ROS_MASTER_URI#*//} 11311; do
    echo "Esperant ROS Master..."
    sleep 1
done
# Executa el comandament final
exec "$@"
