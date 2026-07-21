#!/bin/bash
echo "Setup unitree ros2 environment"

# 1. Cargamos Humble (el contenedor usa Humble, no Foxy)
source /opt/ros/humble/setup.bash

# 2. Usamos la ruta /workspace que es donde el contenedor monta el proyecto
source /workspace/cyclonedds_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>  
                            <NetworkInterface name="enp4s0" priority="default" multicast="default" />      
                       </Interfaces></General></Domain></CycloneDDS>'