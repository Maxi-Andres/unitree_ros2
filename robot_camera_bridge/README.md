# robot_camera_bridge — la cámara del robot como fuente de "Live"

Hace que **la cámara del robot sea la fuente de video** de AI-VL (en vez de que
alguien abra Live con el teléfono). Lee la cámara por ROS2, la decodifica a JPEG y
la manda al backend por WebSocket; el backend la reenvía **directo a los monitores
sin pasar por YOLO** → **mínima latencia** para ver lo que ve el robot.

```
🤖 cámara --ROS2--> bridge (decode->JPEG) --WS--> backend /ws/robot-cam --> Monitor
```

Corre **dentro del devcontainer** (ROS2 + DDS del robot). Host-networked → llega al
backend en `wss://localhost:8443`.

## Fuentes de cámara (`camera_sources.py`)
- **`go2`** — cámara frontal vía la **video API** (`GetImageSample`, api_id 1001 en
  `/api/videohub`): el robot devuelve un **JPEG listo** que reenviamos tal cual (sin
  decodificar) → mínima latencia y confiable. `GO2_VIDEO_FPS` regula el poll.
  *(No usamos `/frontvideostream`: sus secuencias H.264 grandes se deserializan
  corruptas sobre el bridge ROS2/cyclonedds del Go2.)*
- **`g1`** — un topic `sensor_msgs/Image` configurable (`G1_IMAGE_TOPIC`). Cuando el
  G1 publique su cámara, ponés el topic y listo (mismo bridge).
- **`test`** — frame sintético en movimiento: **verifica todo el pipeline sin robot**.

- **`stream`** — el video que **ya salió del robot**, sin DDS y sin importar en qué red
  está. `STREAM_URL` elige el lector, y la elección está medida (ver `.env.example`):
  **`https://127.0.0.1:8889/robot/whep` es la buena** — el H.264 de mediamtx por WebRTC,
  ~200 ms, y no le cuesta nada al robot. `rtsp://` es el mismo stream 2455 ms tarde;
  `http://<robot>:8093/stream` es una segunda copia de la imagen cruzando el enlace de campo.

  > This bullet is the one thing in this file kept in English on purpose, because it is the
  > setting that has been wrong twice: **the drive view, YOLO and the VLM all read whatever
  > `STREAM_URL` points at.** WHEP needs `aiortc` in the container (`requirements.txt`;
  > `run_camera_bridge.sh` installs it if missing) and mediamtx serving over **HTTPS** — its
  > `webrtcEncryption: yes` makes a plain `http://` POST answer `400 Bad Request` with no
  > explanation. TLS verification stays ON for remote hosts (pin the CA with
  > `STREAM_TLS_CA`); it is skipped only for loopback, where there is no network to
  > intercept.

## Cómo se usa
1. `cp robot_camera_bridge/.env.example robot_camera_bridge/.env` y ajustá (`CAMERA_ROBOT`, etc.).
2. En el devcontainer:
   ```bash
   source /workspace/setup.sh
   python3 /workspace/robot_camera_bridge/robot_camera_bridge.py
   ```
   (o `bash /workspace/robot_camera_bridge/run_camera_bridge.sh`).
3. Desde la página **Monitor**: botón **"Usar cámara del robot"** (start/stop), o a mano:
   ```bash
   curl -s localhost:8091/health
   curl -sX POST localhost:8091/start
   curl -sX POST localhost:8091/stop
   ```

### Probar sin robot
```bash
CAMERA_ROBOT=test START_STREAMING=true bash /workspace/robot_camera_bridge/run_camera_bridge.sh
```
Activá el Monitor y vas a ver el patrón de prueba moviéndose (valida bridge → backend → monitor).

## Latencia
- Bajá `GO2_RESOLUTION` (180p) y `JPEG_QUALITY` para menos latencia/ancho de banda.
- El path NO pasa por YOLO (fanout directo). La detección se activa aparte cuando la
  necesites (el pipeline normal de Live con el teléfono sigue existiendo).

## Endpoints (control)
- `GET /health` · `GET /status` · `POST /start` · `POST /stop`

## Dependencias (en el contenedor)
`websocket-client` (productor WS), `opencv` (`cv2`) + `numpy` (solo para las fuentes
g1/test). El Go2 no necesita decodificar nada (recibe JPEG). Instalado con
`pip install websocket-client` (ya está).
