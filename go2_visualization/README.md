# go2_visualization — ver el Unitree Go2 en RViz

Paquete "casero" (sin compilar) para **ver el perro Go2 real en RViz**: su modelo 3D
articulándose con los joints reales + el point cloud del lidar + la odometría + los
frames TF. Todo corre **dentro del devcontainer** de `unitree_ros2`.

## Qué hay acá

```
go2_visualization/
├── models/go2_description/          modelo del Go2 (URDF + meshes .dae), bajado de unitree_ros
│   └── urdf/
│       ├── go2_description.urdf                      (original, usa package://)
│       └── go2_description_resolved_file_paths.urdf  (meshes resueltos a file:// — este se usa)
├── scripts/
│   └── go2_lowstate_to_joint_states_bridge.py   /lowstate -> /joint_states (+ TF odom->base)
├── launch/
│   └── go2_full_visualization.launch.py         levanta RSP + bridge + RViz
└── rviz/
    ├── go2_full_robot_model_and_sensors.rviz    modelo + lidar + odometría + TF
    └── go2_world_view_lidar_and_odometry.rviz   solo lidar + odometría + TF (sin modelo)
```

## Por qué hace falta esto

`unitree_ros2` publica el estado en `/lowstate` y `/sportmodestate`, que son mensajes
**custom** que RViz no sabe dibujar, y **no** publica un URDF (`/robot_description`).
Para ver el perro se necesita: (1) el modelo `go2_description`, (2) `robot_state_publisher`
con ese URDF, y (3) un puente que convierta `/lowstate` en `/joint_states`. Eso es
justo lo que arma este paquete.

## Cómo se usa

1. Abrí el devcontainer (VSCode: *Reopen in Container*) con el **Go2 conectado** por
   ethernet (PC en `192.168.123.99`, perro en `192.168.123.161`).
2. En una terminal del contenedor:
   ```bash
   source /workspace/setup.sh
   ros2 launch /workspace/go2_visualization/launch/go2_full_visualization.launch.py
   ```
   Se abre RViz mostrando el modelo del Go2 moviéndose con la data real, el point
   cloud del lidar y la flecha de odometría, todo en el frame `odom`.

### Opciones del launch

- `launch_rviz:=false` — levanta RSP + bridge sin abrir RViz (útil si abrís RViz aparte).
- `rviz_config:=/workspace/go2_visualization/rviz/go2_world_view_lidar_and_odometry.rviz`
  — usar el layout sin modelo.
- `publish_base_transform_from_odometry:=false` — no colocar el modelo en el mundo;
  mostrarlo articulándose "en el lugar" (Fixed Frame `base`).

### Ver solo el mundo (sin modelo), rápido

Si solo querés el lidar + odometría sin el modelo, abrí RViz con el layout liviano:
```bash
source /workspace/setup.sh
ros2 run rviz2 rviz2 -d /workspace/go2_visualization/rviz/go2_world_view_lidar_and_odometry.rviz
```

## Detalles / gotchas

- **Fixed Frame = `odom`** (así lo traen los `.rviz`). El modelo se ancla vía TF
  `odom -> base`, que publica el bridge desde `/utlidar/robot_odom`.
- **PointCloud2 en Best Effort:** el lidar publica BEST_EFFORT; los `.rviz` ya tienen
  puesto *Reliability Policy = Best Effort*. Si agregás un cloud a mano y no aparece,
  es casi siempre por dejarlo en Reliable.
- **Mapeo de joints:** el URDF ordena los joints FL, FR, RL, RR, pero `LowState.motor_state`
  viene FR, FL, RR, RL. El bridge mapea por índice explícito (ver el script).
- **Meshes:** el URDF original usa `package://go2_description/...`; como `go2_description`
  no es un paquete ament instalado, se generó una copia con los paths reescritos a
  `file:///workspace/...` (`go2_description_resolved_file_paths.urdf`). Si movés esta
  carpeta, regenerá ese archivo con:
  ```bash
  sed 's#package://go2_description/#file:///workspace/go2_visualization/models/go2_description/#g' \
      models/go2_description/urdf/go2_description.urdf \
      > models/go2_description/urdf/go2_description_resolved_file_paths.urdf
  ```
- Si RViz no dibuja por X11 en Linux nativo, en el host: `xhost +local:root`.
