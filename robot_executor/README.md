# robot_executor — el ejecutor de skills (Fase 2 de AI-VL)

Cierra el loop de voz: recibe el **skill JSON** que produce `/command` de AI-VL y
**hace que el robot lo ejecute**, publicando el comando Unitree por **ROS2**.

```
🎤 → /command (skill JSON) → backend /api/execute → ESTE /execute → ROS2 → 🤖 Go2
```

Corre **dentro del devcontainer** de `unitree_ros2` (ahí están ROS2 + la conexión DDS
al robot). El backend de AI-VL (en el host) le reenvía los comandos por HTTP; como el
contenedor usa `network_mode: host`, el backend lo alcanza en `localhost:8090`.

## Diseño

- **Transporte abstracto** (`RobotTransport`): hoy `Go2Ros2Transport` (rclpy, publica
  `unitree_api/msg/Request` en `/api/sport/request`). Mañana se puede sumar un
  transporte SDK / para el G1 sin tocar la capa HTTP.
- **Seguridad:**
  - `SAFE_MODE=true` (default) **bloquea acrobacias** (flips, handstand, walk_upright).
  - `DRY_RUN=true` arma y loguea el comando **sin publicarlo** — para probar sin mover
    el robot.
  - Los `walk`/`turn` acotados re-publican `Move` a `MOVE_RATE_HZ` y mandan `StopMove`
    al terminar (`DEFAULT_STEP_S`, tope `MAX_STEP_S`); el `stop` corta cualquier
    movimiento al instante.

## Cómo se usa

1. Copiá la config: `cp robot_executor/.env.example robot_executor/.env` y ajustá.
2. En una terminal del devcontainer:
   ```bash
   source /workspace/setup.sh
   python3 /workspace/robot_executor/robot_executor_service.py
   ```
3. Desde AI-VL (front → backend `/api/execute`) o probando a mano:
   ```bash
   curl -s localhost:8090/health
   curl -s localhost:8090/execute -H 'Content-Type: application/json' \
        -d '{"robot":"go2","skill":"hello","params":{}}'
   ```

### Probar sin mover el robot
Arrancá con `DRY_RUN=true` (en `.env` o inline):
```bash
DRY_RUN=true python3 /workspace/robot_executor/robot_executor_service.py
```
Loguea el `api_id` + parámetro que publicaría, sin tocar el robot.

## Endpoints

- `GET /health` → `{ok, default_robot, safe_mode, dry_run, robot_ip}`
- `POST /execute {robot, skill, params}` → `{ok, robot, skill, detail, api_id, ...}`
  - `403 {blocked:true}` si `SAFE_MODE` y el skill es acrobático.
  - `422` si el skill no está mapeado para ese robot.

## Mapeo de skills (Go2)

Locomoción/postura/gestos/trucos del `SportClient` → `api_id` de `/api/sport/request`
(ver `go2_commands.py`): `walk`/`turn` → `Move(1008)` con `{x,y,z}`; `stop`→1003;
`sit`→1009; `hello`→1016; `stand_up`→1004; `dance1`→1022; `back_flip`→2043; etc.
`handstand`/`walk_upright`/`pose` llevan `{"data": on}`. El **G1** todavía no tiene
transporte (devuelve "unsupported").
