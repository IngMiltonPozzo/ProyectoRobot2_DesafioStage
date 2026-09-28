# Bitácora — Desafío Stage ROS2 (PRIA, Proyecto de Robots II)

## Entorno (infraestructura)
- WSL2 (Ubuntu-24.04) + Docker Desktop, backend WSL2, ambos con su disco reubicado a E: por espacio.
- Proyecto base: repo del profesor `sbarcelona11/ros2_jazzy_docker`, clonado en `~/ros2_jazzy_docker`.
- Arreglos hechos al Dev Container para que arranque solo, sin pasos manuales:
  - `devcontainer.json`: `overrideCommand: true` (el `/bin/bash` suelto hacía que el contenedor se autoapagara).
  - `Dockerfile`: `chown` de `/home/ros/.devcontainer` (permisos, para que los hooks de arranque puedan escribir su marcador).
  - `post-create.sh`: resiliente a fallos de red, agrega el `source` del entorno ROS al `.bashrc` sin duplicar.
  - `vnc-start.sh`: `unset WAYLAND_DISPLAY` (WSLg confundía a x11vnc) + `setsid` en cada proceso de fondo (Docker los mataba con SIGHUP al cerrar la sesión del arranque automático).
  - `Dockerfile`: agregado `ros-jazzy-ackermann-msgs`, `libfltk1.3-dev`, `libtool`, `libtool-bin` (dependencias de Stage).
- Stage instalado desde código fuente (no hay binario apt para Jazzy): clonados en `ros2_ws/src/`: `tuw-robotics/Stage`, `tuw-robotics/stage_ros2` (rama `jazzy`), `tuw-robotics/marker_msgs` (rama `ros2`).

## Entendimiento del desafío (a partir del PDF)
- El PDF describe una estructura general (idea, objetivo, dinámica, criterios, entregables), no entrega un mapa ni una misión concretos — cada equipo arma su propio escenario y misión.
- Decisión de misión: navegación punto a punto — el robot recibe un destino por ROS2 (`/goal_pose`, publicado desde RViz con "2D Goal Pose"), evita obstáculos usando el láser, y detecta un checkpoint (baliza fiducial) en el camino.

## Diseño técnico

### Mapa
Arena 12x12m, perímetro cerrado, pared divisoria (con los dos extremos acortados 0.5m cada uno para evitar rincones cóncavos sin salida — se detectó en la práctica que el robot quedaba atrapado contra el perímetro en ambos extremos), un obstáculo suelto, una baliza fiducial (ID 2), robot `pioneer2dx_with_laser_and_fiducial` arrancando en (-5,-5) a 45°.

### Arquitectura de nodos
- **`navigator`**: nodo principal de decisión y control. Suscribe `/base_scan`, `/ground_truth`, `/odom`, `/goal_pose`; publica `/cmd_vel` y `/goal_marker`.
- **`checkpoint_detector`**: nodo separado, una sola responsabilidad — suscribe `/fiducials` y `/goal_pose`, reporta cuando el robot pasa a menos de 0.5m reales de la baliza (id=2). Se reinicia con cada objetivo nuevo, para que sea verificable en cada corrida, no solo la primera vez.
- **`map_markers`**: publica la geometría estática del mapa (paredes, obstáculo, baliza) como `MarkerArray` en el frame `world`, para poder visualizarla en RViz — Stage no expone su mapa por ningún tópico de ROS2.

### Decisión: navegar con `/ground_truth`, no con `/odom`
Stage simula deriva de odometría (parámetro `odom_error` del robot) — al quedar empujando una pared, `/odom` puede desviarse varios metros de la posición real. Un robot real no tendría acceso a esta "verdad" del simulador, pero se usa a propósito para que la misión sea confiable y medible dentro del tiempo disponible. Beneficio adicional: `/ground_truth` viene en el frame `world`, el mismo que usa RViz para "2D Goal Pose" — ya no hace falta ninguna conversión manual de coordenadas entre el click y la navegación.

### Máquina de estados: SEEK / BORDER
Reemplaza un primer diseño (SEEK / WALL_FOLLOW con disparo por tiempo de bloqueo continuo) que fallaba en esquinas cóncavas: el láser podía alternar bloqueado/libre muy rápido ahí, y el contador de tiempo nunca llegaba a dispararse — el robot quedaba oscilando sin avanzar.
- **`SEEK`**: gira hacia el objetivo (girando en el lugar si el error de rumbo es grande, para no barrer un arco ancho que puede chocar objetos chicos al costado) y avanza. Vigila también los costados: si algo se acerca demasiado a un lado aunque el frente esté libre, corrige alejándose de ese lado sin dejar de avanzar hacia el objetivo (necesario porque el robot llegó a chocar la baliza, un objeto chico, mientras iba en línea recta con el frente despejado).
- **`BORDER`**: al toparse con un obstáculo, bordea la pared regulando por la distancia **lateral** (no la frontal) — el frente solo se usa para detectar una esquina real (pared/cuerpo justo enfrente), donde gira en el lugar sin retroceder nunca (retroceder aumenta la lectura frontal sin resolver la esquina, lo que en una versión anterior generaba un bucle: retrocedía, el frente se "liberaba", volvía a SEEK, y SEEK lo mandaba de nuevo contra la misma pared).
- El lado para bordear prioriza el que acerca al objetivo, pero solo si no implica resignar mucho más espacio libre que el otro lado (evita meterse por un hueco angosto habiendo uno bien más amplio cerca). Se reevalúa en cada ciclo, no solo al entrar a `BORDER`: si el lado hacia el objetivo se abre mientras bordea, cambia de lado ahí mismo, en vez de esperar a que la pared actual "se termine" del todo.
- Sale de `BORDER` cuando el camino directo al objetivo se despeja (chequeo puntual con el láser en esa dirección exacta) o cuando la pared que sigue termina — lo que ocurra primero.

### Resolución del láser
El sensor cubre 270°, no 360° — hay un cono ciego real de ~90° directo atrás del robot, limitación física del sensor. Dentro de esos 270°, cada lado se divide en 3 sub-sectores (cerca/medio/lejos del frente) en vez de un solo número por lado, para detectar huecos angostos y objetos chicos con más precisión. El sub-sector "lejano" de cada lado se usa además, junto con el frente, para detectar cuándo el **cuerpo** del robot (no solo el sensor) está por rozar algo mientras bordea — el frente solo no alcanzaba para ese caso.

### Watchdog de atascado
Corre en cualquier estado (no solo `BORDER` — un objeto chico al costado, como la baliza, puede trabar al robot en `SEEK` sin que el frente detecte nada). Escala en duración con los intentos **consecutivos** reales (el contador solo se resetea cuando el robot demuestra progreso de verdad, no por un simple cambio de estado — una versión anterior reseteaba el contador al volver a `SEEK` aunque el robot siguiera clavado en el mismo punto, y cada intento se registraba como "el primero", con la misma duración corta, sin escalar nunca). Tiene un tope de intentos; si se supera, prueba una maniobra distinta (giro completo buscando una dirección despejada) antes de darse por vencido con un único mensaje claro, en vez de reintentar para siempre.
**Límite conocido**: si el robot queda con cero grados de libertad de verdad (encajado sin ningún ángulo de giro libre), ningún comando lo libera — hace falta reiniciar la simulación. Es un caso de borde geométrico, no un bug del watchdog.

### Corrección de posición en RViz (frame `world` → `odom`)
RViz dibuja vía TF, siguiendo la cadena `world → odom → base_link`; el segundo tramo lo calcula Stage a partir de la odometría con deriva. El `navigator` publica una corrección `world → odom` calculada en cada ciclo (comparando `/ground_truth` contra `/odom`), para que RViz muestre la posición real del robot — mismo patrón que la corrección `map → odom` de un stack de localización real.
Encontrado en el camino: el `navigator` nunca declaraba `use_sim_time`, así que sus timestamps usaban el reloj de pared en vez del reloj de simulación que usan Stage y RViz — con la corrección publicada como transformación dinámica normal, RViz no lograba encontrarla a tiempo para cada dato de láser y los descartaba todos ("queue is full"), dejando la vista congelada. Se resolvió declarando `use_sim_time` en `True` por defecto (vía `parameter_overrides` en `super().__init__`, porque `rclpy` ya declara ese parámetro implícitamente para el reloj interno de todo nodo — declararlo de nuevo con `declare_parameter` choca con esa declaración y tira `ParameterAlreadyDeclaredException`).

### Qué se puede ver en cada ventana
- **Stage**: la simulación física en sí. No dibuja nada que le llegue por ROS2 (ni el objetivo de RViz, ni ningún dato externo) — limitación de la herramienta.
- **RViz**: todo lo que el robot percibe/publica — láser, fiducial, el mapa (`map_markers`) y el objetivo (`/goal_marker`), alineado con Stage gracias a la corrección de TF. Vista en **TopDownOrtho** para comparar de igual a igual contra Stage.

## Hecho
- Paquete `stage_challenge` completo: mundo propio, launch propio (silencia el log verboso de `stage_ros2` con `--log-level stage:=error`), config de RViz propia.
- Tres nodos (`navigator`, `checkpoint_detector`, `map_markers`) compilando, corriendo y confirmados con datos reales de log — llegada al objetivo confirmada en múltiples corridas, checkpoint detectándose de forma repetible.
- Watchdog de atascado con escalado real y tope; bordeo con reevaluación de lado y salida por línea de visión directa al objetivo; seguridad lateral activa en `SEEK`; sub-sector lejano conectado a la detección de esquina/cuerpo.

## Pendiente
- **Validación final de robustez**: correr varias misiones seguidas con el código actual (después del último ajuste del sub-sector lejano) y confirmar que no aparecen bloqueos nuevos — es la pata más floja de los criterios de verificación del PDF.
- Pulido opcional del recorrido: el seguimiento de pared todavía serpentea un poco en tramos largos: ajustar ganancia/zona muerta si da tiempo, no es bloqueante.
- **Entregables**: `README.md` con arquitectura y decisiones de diseño (paso 6 del PDF) — se arma a partir de esta bitácora. Estructura Git: `.gitignore`, `.repos` (vcstool, para las dependencias externas Stage/stage_ros2/marker_msgs sin subirlas al repo), y subir a un repositorio propio en GitHub (el PDF acepta también un .txt con el link, en vez del paquete completo).
