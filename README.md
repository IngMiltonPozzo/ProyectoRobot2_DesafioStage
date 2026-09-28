# Desafío Stage ROS2 — Proyecto de Robots II (PRIA)

Solución al Desafío Stage ROS2 para la materia Proyecto de Robots II del
posgrado PRIA (UTEC / FURG / UNRaf). Un robot Pioneer2dx simulado en Stage
navega de forma autónoma hasta un objetivo indicado interactivamente en
RViz ("2D Goal Pose"), evitando obstáculos con un láser 2D simulado, y
detecta una baliza fiducial de checkpoint en el camino.

**Autor:** Milton Pozzo

## Arquitectura ROS 2

### Dependencias externas (no incluidas en este repo, ver Instalación)
- **Stage** (tuw-robotics, rama `jazzy`) — simulador físico 2D
- **stage_ros2** (tuw-robotics, rama `jazzy`) — bridge Stage ↔ ROS 2
- **marker_msgs** (tuw-robotics, rama `ros2`) — mensaje `MarkerDetection`
  para la baliza fiducial

### Paquete propio: `stage_challenge`

| Nodo | Función |
|---|---|
| `navigator` | Nodo principal: percibe (láser), decide (máquina de estados) y actúa (`/cmd_vel`). |
| `checkpoint_detector` | Suscribe `/fiducials` y `/goal_pose`; marca el checkpoint cumplido al detectar la baliza esperada a corta distancia. |
| `map_markers` | Publica la geometría del mapa como `MarkerArray` para visualizarla en RViz. |

**Tópicos usados por `navigator`:**

| Tópico | Tipo | Uso |
|---|---|---|
| `/base_scan` | `sensor_msgs/LaserScan` | Percepción (270°, 5 m de alcance) |
| `/ground_truth` | `nav_msgs/Odometry` | Posición real del robot (frame `world`), usada para navegar |
| `/odom` | `nav_msgs/Odometry` | Solo para calcular la corrección TF `world→odom` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Objetivo, recibido desde RViz |
| `/cmd_vel` | `geometry_msgs/Twist` | Comando de velocidad al robot |
| `/goal_marker` | `visualization_msgs/Marker` | Marca visual del objetivo en RViz |

## Mapa

Arena cerrada de 12×12 m con una pared divisoria y un obstáculo suelto —
ver `worlds/desafio_stage.world`.

## Diseño del navigator

*(el detalle completo de cada decisión, con los datos reales que la
motivaron, está en [`DEVLOG.md`](DEVLOG.md); acá un resumen de la
versión final)*

Máquina de 4 estados: **ADVANCE / TURN / HOLD / SEARCH**.

- **ADVANCE** — avanza en línea recta hacia el objetivo mientras el
  camino está despejado.
- **TURN** — al detectar un obstáculo, decide una sola vez hacia qué
  lado girar (prioriza el lado del objetivo; si el lado de escape
  también está crítico, va directo a `SEARCH` en vez de comprometerse)
  y gira hasta alinearse con el obstáculo.
- **HOLD** — avanza bordeando el obstáculo, regulando la distancia
  lateral. En cada esquina real, reconsidera con datos frescos si el
  objetivo ya quedó accesible directo, y en ese caso lo suelta sin
  esperar a que el sensor lo libere del todo.
- **SEARCH** — último recurso: gira en el lugar buscando una dirección
  despejada.

### Decisiones de diseño clave

1. **`ground_truth` en vez de `/odom` para navegar.** Stage simula
   deriva de odometría; al quedar empujando una pared, `/odom` puede
   desviarse varios metros. La corrección TF `world→odom` se recalcula
   dinámicamente en cada ciclo (no una sola vez al inicio), comparando
   ambos, para que RViz siga mostrando la posición real incluso cuando
   el robot empuja algo.

2. **`use_sim_time` fijado vía `parameter_overrides`**, no
   `declare_parameter` (rclpy ya lo declara internamente). Sin esto,
   los timestamps del nodo usan reloj de pared mientras Stage/RViz usan
   reloj de simulación, y RViz descarta todos los mensajes del láser.

3. **Dos umbrales, roles distintos:** `clearance` (0.6 m, umbral
   cómodo, dispara reaccionar) y `min_body_clearance` (0.3 m, piso
   físico real — radio del Pioneer2dx de su centro a su propia esquina,
   calculado geométricamente a partir de su chasis 0.44×0.38 m).

4. **Láser dividido en 7 sectores** (frente + 3 subsectores por lado:
   `near`/`mid`/`far`), no solo 3. El subsector lejano detecta cuándo el
   *cuerpo* del robot — no solo el punto donde está el sensor — está por
   rozar algo.

5. **Ningún mecanismo retrocede.** Toda maniobra de escape combina giro
   y avance hacia adelante; nunca velocidad lineal negativa. Un pivote
   puro barre las esquinas del chasis en un círculo fijo; trasladarse
   mueve el cuerpo por el espacio, no solo lo rota.

6. **El giro tiene dirección físicamente verificada; el avance, no.**
   Durante una corrección lateral activa (algún lado ya cruzó el piso
   físico), el avance se apaga del todo — un empuje hacia adelante
   podría, según la geometría del contacto, acercar el robot al mismo
   obstáculo en vez de alejarlo.

7. **Watchdog único** (posición Y orientación juntas), evaluado
   siempre, antes de cualquier lógica de estado — si ninguna de las dos
   cambia, hay un atasco real, sin importar la rama de código activa.

## Cómo ejecutar

### Instalación (entorno completo, desde cero)

1. Clonar y levantar el Dev Container base:
   ```bash
   git clone https://github.com/sbarcelona11/ros2_jazzy_docker.git
   cd ros2_jazzy_docker && code .
   ```
   Abrir en el Dev Container desde VS Code (Docker Desktop debe estar
   corriendo).

2. Ajustes necesarios al Dev Container (aplicar en
   `.devcontainer/` antes de reconstruir, si no vienen ya incluidos):
   - `devcontainer.json`: `"overrideCommand": true`
   - `Dockerfile`: `chown -R ros:ros /home/ros/.devcontainer` + paquetes
     de Stage
   - `post-create.sh`: agregar el `source` del workspace a `.bashrc`
     sin duplicar
   - `vnc-start.sh`: `unset WAYLAND_DISPLAY` + `setsid`/`disown` en los
     procesos de fondo

3. Clonar este repositorio dentro de `/ros2_ws/src/`:
   ```bash
   cd /ros2_ws/src
   git clone https://github.com/IngMiltonPozzo/ProyectoRobot2_DesafioStage.git stage_challenge
   ```

4. Traer las dependencias externas con vcstool:
   ```bash
   cd /ros2_ws/src
   vcs import . < stage_challenge/dependencies.repos
   ```

5. Compilar:
   ```bash
   cd /ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

### Ejecución

```bash
# Terminal 1
ros2 launch stage_challenge desafio_stage.launch.py

# Terminal 2
ros2 run stage_challenge navigator

# Terminal 3
ros2 run stage_challenge checkpoint_detector
```

En RViz: botón **"2D Goal Pose"**, click en el mapa para dar un
objetivo. Agregar displays `MarkerArray` (`/map_markers`) y `Marker`
(`/goal_marker`) si no aparecen, y `Ctrl+S` para persistirlos.

## Limitaciones conocidas

- **No enhebra pasos angostos-pero-transitables:** al detectar el lado
  del objetivo bloqueado (por el umbral cómodo, no el físico), siempre
  rodea por el lado de escape sin evaluar si el original, aunque
  ajustado, seguía siendo pasable. Decisión de diseño (preferir rodear a
  enhebrar), no un bug.
- Un atasco físico genuino con cero grados de libertad (encajado sin
  ningún margen de giro) requiere reiniciar la simulación — ningún
  comando de velocidad, sin retroceso, puede resolverlo por diseño.

## Estructura del repositorio

```
stage_challenge/
├── README.md                — este archivo
├── DEVLOG.md                 — bitácora completa de decisiones e iteraciones
├── dependencies.repos        — dependencias externas (vcstool)
├── package.xml / setup.py    — definición del paquete ROS 2
├── stage_challenge/
│   ├── navigator.py          — nodo principal de navegación
│   ├── checkpoint_detector.py
│   └── map_markers.py
├── launch/desafio_stage.launch.py
├── worlds/desafio_stage.world
└── config/rviz/desafio_stage.rviz
```
