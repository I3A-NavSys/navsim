# Guía de Arquitectura, Contexto y Relevos para Agentes de IA (`RafaBranch`)

> **Propósito de este documento**: Servir como memoria técnica exhaustiva y punto de partida para cualquier agente de Inteligencia Artificial (o desarrollador) que continúe la evolución de **NavSim** en esta rama. Lee este documento completo antes de modificar código, extensiones o escenas USD.

---

## 1. Filosofía y Visión de la Nueva Rama

El objetivo central de esta rama (`RafaBranch`) es **migrar NavSim hacia un simulador físico y de comunicaciones completamente autónomo, desacoplado y fiel a escenarios del mundo real**:

1. **Autonomía y Desacoplamiento Total del Simulador**:
   - Anteriormente, el simulador de Isaac Sim estaba fuertemente acoplado a módulos de planificación U-Space, gestores de colas (MQTT) y lógica de negocio interna dentro del propio repositorio/proceso de simulación.
   - En la nueva arquitectura, **Isaac Sim actúa exclusivamente como el "Mundo Real" (Gemelo Físico/Cinemático)** y las aeronaves como **vehículos reales autónomos**.
   - Toda interacción externa (envío de planes de vuelo 4D, control manual por joystick/teclado, monitorización de tráfico aéreo) se realiza exclusivamente por red mediante **MAVLink v2 sobre UDP**, exactamente igual que se interactuaría con una flota de drones reales o SITL (Software-In-The-Loop).

2. **Georreferenciación Real sobre el Elipsoide WGS-84**:
   - El espacio cartesiano local de Isaac Sim $(X, Y, Z)$ deja de ser un sistema arbitrario y pasa a representar un **plano tangente local ENU (East-North-Up)** anclado a coordenadas geodésicas reales $(\text{Latitud}, \text{Longitud}, \text{Altitud HAE})$.
   - Por defecto, el origen $(0, 0, 0)$ está anclado en el **Campus Universitario de Albacete (UCLM - ESII / I3A)**:
     - Latitud: `38.981800° N`
     - Longitud: `-1.857000° W`
     - Altitud Elipsoidal (HAE): `686.00 m`
   - Además, el simulador inspecciona dinámicamente el Stage USD al abrirse para detectar metadatos geoespaciales nativos (`omni:geospatial:wgs84:reference:referencePosition`, `CesiumGeoreference`, o atributos `NavSim:georeference:*`).

3. **Cumplimiento Normativo Internacional (ASTM F3411-22a / EUROCAE ED-282)**:
   - Cada aeronave en escena cumple con la normativa **Open Remote ID (Direct & Network Remote ID)** exigida por la FAA (ASTM F3411) y EASA U-Space (EUROCAE ED-282 / Reg. Delegado UE 2019/945).
   - La telemetría emitida por cada dron al exterior no utiliza tramas propietarias ad-hoc, sino los mensajes oficiales **`OPEN_DRONE_ID_*`** de MAVLink v2, transmitiendo posición geodésica WGS-84 (`lat/lon` en $10^{-7}$ grados), altitudes geodésica y AGL, velocidades polares horizontales/verticales, rumbo verdadero (*track angle* $0..359.99^\circ$ horario desde el Norte) e identidad oficial del UAS y del operador.
   - El simulador incluye un motor de **validación previa al vuelo** que bloquea el despegue si detecta colisiones de identidad (dos drones emitiendo el mismo `UAS ID` en el aire) o metadatos obligatorios corruptos.

4. **Catálogo Declarativo de Flota Heterogénea (`fleet/`)**:
   - Las propiedades físicas (masa, tensor de inercia, posiciones de rotores respecto al Centro de Masas), aerodinámicas ($k_{FT}, k_{MDR}, k_{FD}, k_{MD}, \omega_{\min}, \omega_{\max}, \omega_{\text{hov}}$) y las matrices del controlador lineal en espacio de estados ($H_s, K_x, K_y$) ya no están codificadas en scripts de Python, sino declaradas en ficheros `fleet/<modelo>/dynamics.yaml`.
   - El motor físico apila tensores NumPy heterogéneos $(N, \dots)$ permitiendo simular decenas o cientos de drones de distintos modelos (p. ej. `UAM_aerotaxi`, `UAM_minidrone`, `Jovi S4`) en una única pasada matricial vectorizada por paso de física.

---

## 2. Estructura Modular del Repositorio

El repositorio ha sido limpiado y reorganizado bajo la siguiente jerarquía:

```text
navsim/
├── agents/                         # Documentación técnica y guías de relevo para agentes de IA
│   └── README.md                   # Este documento
├── doc/                            # Documentación general e imágenes del proyecto
├── extensions/                     # Extensiones nativas de NVIDIA Isaac Sim (Omniverse Kit)
│   ├── navsim/                     # Extensión principal: Física vectorizada, ASTM F3411, WGS-84 y MAVLink
│   └── manual_controller/          # Extensión de pilotaje manual (Joystick/Teclado) + ChaseCamera en Viewport 2
├── fleet/                          # Catálogo declarativo de modelos de aeronaves (USD + dynamics.yaml)
│   ├── Jovi S4/                    # Modelo Jovi S4 + dynamics.yaml
│   ├── UAM_aerotaxi/               # Modelo UAM Aerotaxi (1_rb_model.usd + dynamics.yaml)
│   ├── UAM_minidrone/              # Modelo UAM Minidrone (1_rb_model.usd + dynamics.yaml)
│   └── README.md                   # Especificación del formato dynamics.yaml
├── mavlink/                        # Núcleo de comunicaciones MAVLink v2 (sin dependencias externas pesadas)
│   ├── __init__.py
│   ├── navsim.xml                  # Definición XML oficial del dialecto MAVLink v2 de NavSim
│   ├── dialect.py                  # Codec binario MAVLink v2 (CRC-16/MCRF4XX + CRC Extra) y estructuras de mensajes
│   └── udp_manager.py              # Gestor de sockets UDP no bloqueantes por dron (FleetMavlinkManager)
├── scripts/                        # Herramientas externas autónomas (clientes/monitores fuera de Isaac Sim)
│   └── telemetry_monitor/          # Analizador/Dashboard CLI en tiempo real de telemetría ASTM F3411
│       ├── telemetry_monitor.py
│       └── run_telemetry_monitor.bat
├── sims/                           # Escenarios USD de simulación listos para abrir en Isaac Sim
├── vertiports/                     # Modelos USD modulares de vertipuertos y pads de aterrizaje/entrega
└── worlds/                         # Entornos 3D base (Campus, Ciudades, Lezuza, Madrid, San Francisco, Boston...)
```

---

## 3. Arquitectura de Comunicaciones MAVLink v2 (`mavlink/`)

El paquete `mavlink/` implementa un codec MAVLink v2 puro en Python/NumPy (`MAVLinkV2Codec`) con validación de firma `STX = 0xFD` y suma de comprobación `X.25 CRC-16/MCRF4XX` con `CRC_EXTRA`.

### 3.1. Esquema de Puertos UDP por Dron
Cada UAV descubierto en la escena USD (ordenado alfabéticamente por su ruta Prim) recibe un índice `idx = 0, 1, ..., N-1` y un `sysid = (idx % 254) + 1`:

| Canal | Dirección | Puerto UDP | Descripción |
| :--- | :--- | :--- | :--- |
| **Recepción de Comandos (RX)** | Exterior $\rightarrow$ Dron `idx` | `15000 + idx` | Escucha planes de vuelo polinómicos 4D (`12000`, `12001`), comandos de control manual (`12004`) y pings de suscripción. |
| **Emisión de Telemetría (TX)** | Dron `idx` $\rightarrow$ Exterior | `16000 + idx` | Emite a 20 Hz tramas estándar Open Remote ID (`12900`, `12901`, `12903`, `12905`) junto con el estado de seguimiento del plan de vuelo 4D (`12003`) hacia `127.0.0.1:(16000 + idx)` y suscriptores activos. |

> **Lección crítica implementada**: Los destinos de telemetría (`telemetry_destinations`) están **desacoplados** de la dirección remitente del último comando (`last_cmd_addr`). Así, cuando el controlador manual por joystick envía comandos desde un puerto efímero hacia `15000 + idx`, la telemetría continúa emitiéndose ininterrumpidamente hacia el puerto fijo de monitorización `16000 + idx`.

### 3.2. Catálogo de Mensajes MAVLink v2 Implementados (`mavlink/dialect.py`)

1. **Mensajes de Telemetría Estándar ASTM F3411 / EUROCAE ED-282 (OpenDroneID)**:
   - `OPEN_DRONE_ID_BASIC_ID` (**ID 12900**): Identificador único `uas_id` (20 bytes ASCII), `id_type` (Serie ANSI/CTA-2063-A, Registro CAA, UUID UTM, Sesión) y `ua_type` (Multirrotor, VTOL, Ala fija, etc.).
   - `OPEN_DRONE_ID_LOCATION` (**ID 12901**): Estado operativo (`GROUND`, `AIRBORNE`, `EMERGENCY`, `REMOTE_ID_SYSTEM_FAILURE`), rumbo `direction` en centígrados ($0..35999$), velocidades `speed_horizontal` y `speed_vertical` en $\text{cm/s}$, `latitude` y `longitude` en $10^{-7}\text{ deg}$ (WGS-84), `altitude_geodetic`, `height` AGL sobre el suelo/vertipuerto y precisiones estandarizadas.
   - `OPEN_DRONE_ID_SELF_ID` (**ID 12903**): Descripción en texto libre del propósito de la operación/misión.
   - `OPEN_DRONE_ID_OPERATOR_ID` (**ID 12905**): Identificador oficial de registro del operador ante la autoridad aeronáutica (p. ej. AESA/EASA).

2. **Mensajes de Planes de Vuelo Polinómicos 4D (Polinomio de Taylor de orden 5) y Control Manual (Extensión NavSim `12000..12004`)**:
   - `NAVSIM_FLIGHT_PLAN_HEADER` (**ID 12000**): Metadatos de ruta (`plan_id`, `priority`, `radius`, `max_var_lin_vel`, `max_var_ang_vel`, `target_yaw`, `num_waypoints`).
   - `NAVSIM_FLIGHT_PLAN_WAYPOINT` (**ID 12001**): Tramo polinómico de Taylor de orden 5 con tiempo `t` y vectores 3D de posición, velocidad, aceleración, *jerk*, *snap*, *crackle* y *heading*.
   - `NAVSIM_FLIGHT_PLAN_ACK` (**ID 12002**): Confirmación de recepción y ensamblado completo de un plan de vuelo.
   - `NAVSIM_FLIGHT_PLAN_STATUS` (**ID 12003**): Telemetría de retorno de ejecución del FlightPlan 4D y cronorreferenciación (`time_unix_usec` en µs UTC, `sim_time`, `uav_state`, `plan_id`, `current_wp`, `total_wps`, `wp_label`, ventana temporal `[fp_init_time, fp_finish_time]`, instante del tramo `wp_time`, referencia polinómica evaluada `desired_pos`/`desired_vel` y errores instantáneos `pos_error`/`vel_error`).
   - `NAVSIM_MANUAL_CONTROL` (**ID 12004**): Consigna cinemática directa en el marco local del dron (`vel_x`, `vel_y`, `vel_z` en $\text{m/s}$, `rot_z` en $\text{rad/s}$) y modo (`0=DISARM/IDLE`, `1=MANUAL`, `2=AUTO`, `3=HOVER`).

---

## 4. Detalle de las Dos Extensiones Actuales de Isaac Sim (`extensions/`)

### 4.1. Extensión 1: `navsim` (`extensions/navsim/`)
Es el motor central del simulador. Se compone de 6 módulos cohesivos dentro de `extensions/navsim/navsim_python/`:

1. **[`extension.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/extension.py) (`NavSim(omni.ext.IExt)`)**:
   - Gestiona el ciclo de vida en Omniverse Kit (`on_startup`, `on_shutdown`, `on_timeline_play`, `on_timeline_stop`, `on_physics_step`, `on_stage_event`).
   - Al pulsar **PLAY**:
     1. Recarga el catálogo `fleet/` y detecta la georreferenciación del escenario USD.
     2. Ejecuta `validate_current_fleet()` descubriendo automáticamente las aeronaves en el Stage mediante atributos `NavSim:*` (sin ninguna restricción de nombres o rutas en el árbol USD).
     3. Si hay conflictos críticos de identidad (dos drones con el mismo `UAS ID`), detiene el Timeline inmediatamente y lanza una alerta visual y en consola.
     4. Inicializa la vista vectorizada `RigidPrim` de Isaac Sim, aplica masas e inercias desde `dynamics.yaml` y arranca `UAVControl` y `FleetMavlinkManager`.
   - En cada paso de física (`on_physics_step`):
     1. Sondea sin bloqueo nuevos planes de vuelo polinómicos (`poll_new_flight_plans`).
     2. Sondea consignas de control manual por joystick/teclado (`poll_manual_controls`).
     3. Ejecuta el paso de control y fuerzas físicas `uav_control.step(current_time, step_size)`.
     4. Emite telemetría ASTM F3411 si se ha cumplido el periodo de 20 Hz (`maybe_send_telemetry`).

2. **[`uav_control.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/uav_control.py) (`UAVControl`)**:
   - Implementa la lectura de sensores inerciales (`imu()`), convirtiendo cuaterniones a ángulos de Euler (Roll, Pitch, Yaw) y transformando velocidades lineales y angulares del marco global ENU al marco local del cuerpo del dron.
   - Soporta conmutación transparente por cada dron entre:
     - **Modo `AUTO`**: Evalúa los polinomios de Taylor de grado 5 en función del tiempo de simulación $t - t_k$ para obtener la referencia deseada.
     - **Modo `MANUAL` / `HOVER`**: Integra las consignas de velocidad lineal local $(v_x, v_y, v_z)$ y velocidad angular de guiñada $(\dot{\psi})$ enviadas por MAVLink. Incluye un *watchdog* de seguridad de `0.5 s`: si dejan de llegar tramas manuales, el dron pasa automáticamente a estacionario (*hover* con velocidad cero) para evitar que se estrelle o derive indefinidamente.
   - Ejecuta la ley de control lineal en espacio de estados de forma 100% matricial para los $N$ drones:
     $$u = H_s - K_x x - K_y E$$
   - Calcula el empuje de cada uno de los 4 rotores, el arrastre aerodinámico lineal y los momentos de fricción/reacción, aplicando las fuerzas y pares resultantes en el Centro de Masas mediante `rigid_prim_view.apply_forces_and_torques_at_pos(...)`.

3. **[`georeference.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/georeference.py) (`WGS84Georeference`)**:
   - Implementa la transformación exacta entre el plano tangente local **ENU** $(x=\text{Este}, y=\text{Norte}, z=\text{Arriba})$, coordenadas geocéntricas **ECEF** y coordenadas geodésicas **WGS-84** $(\phi, \lambda, h)$ mediante el método analítico de Bowring (vectorizado en NumPy para toda la flota en $< 0.05\text{ ms}$).

4. **[`drone_identity.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/drone_identity.py) (`UAVStandardIdentity`, `validate_fleet_prims`, `find_navsim_drone_prims`)**:
   - Detección automática de aeronaves en el Stage (`find_navsim_drone_prims`) basada exclusivamente en atributos `NavSim:*`, permitiendo nombres libres (sin obligar a la convención `UAV_*`).
   - Inspecciona los atributos USD de cada dron según la lista unificada:
     - `NavSim:id`, `NavSim:id_type`, `NavSim:ua_type`, `NavSim:operator_id` (obligatorios).
     - `NavSim:operator_id_type` (opcional, defecto `0=CAA`).
     - `NavSim:self_id_desc` (opcional, defecto `""`).
     - El modelo físico no requiere atributo: se deduce de la referencia o payload nativo de USD hacia `fleet/`.
   - Valida formato ANSI/CTA-2063-A y unicidad de identificadores en toda la flota.

5. **[`drone_config.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/drone_config.py) (`FleetCatalog`, `DroneModelConfig`, `FleetDynamicsTensors`)**:
   - Escanea automáticamente `fleet/*/dynamics.yaml` y construye los tensores apilados por lote según el modelo de cada UAV presente en la escena.

6. **[`ui_builder.py`](file:///c:/Users/rafae/code/navsim/extensions/navsim/navsim_python/ui_builder.py) (`UIBuilder`)**:
   - Construye la ventana **"NavSim - Consola"** en Isaac Sim con desplegables organizados:
     - **Georreferenciación**: Configuración interactiva del origen geodésico WGS-84 (Latitud, Longitud, Altitud) y botón de restablecimiento al Campus de Albacete.
     - **Flota**: Estado de validación ASTM F3411 / EUROCAE ED-282 en tiempo real y tabla de aeronaves inspeccionadas.

---

### 4.2. Extensión 2: `manual_controller` (`extensions/manual_controller/`)
Permite tomar el control manual en tiempo real de cualquier dron de la escena mediante un **Joystick/Gamepad** o el **Teclado**, enviando tramas MAVLink v2 `NAVSIM_MANUAL_CONTROL` al puerto UDP del dron seleccionado.

Consta de 4 módulos en `extensions/manual_controller/manual_controller_python/`:

1. **[`extension.py`](file:///c:/Users/rafae/code/navsim/extensions/manual_controller/manual_controller_python/extension.py) (`ManualController(omni.ext.IExt)`)**:
   - Construye la interfaz gráfica con:
     - Selector desplegable (`ComboBox`) de todos los drones presentes en la escena (descubiertos por propiedades `NavSim:*`) y botón de refresco.
     - Botón de encendido/apagado del control manual y conmutador Joystick / Teclado.
     - Botones de modo de vuelo (`MANUAL`, `AUTO`, `HOVER`, `DISARM`).
     - Límites configurables de velocidad lineal ($\text{m/s}$) y angular ($\text{rad/s}$).
     - Controles de ajuste en vivo para la cámara de seguimiento (distancia, elevación, ángulo de mirada) y casillas de inversión de ejes.
     - 4 gráficas en tiempo real (`ui.Plot`) para las consignas $V_x, V_y, V_z, \omega_z$.

2. **[`controller_logic.py`](file:///c:/Users/rafae/code/navsim/extensions/manual_controller/manual_controller_python/controller_logic.py) (`ControllerLogic`)**:
   - Ejecuta un bucle asíncrono a ~30 Hz que lee los ejes normalizados del dispositivo de entrada, escala por los límites de velocidad y empaqueta un mensaje `MavManualControlMsg` hacia `127.0.0.1:(15000 + uav_idx)`.
   - **Cámara de Seguimiento en Viewport Secundario (`NavSim — Cámara de Seguimiento`)**:
     - Para no alterar ni bloquear jamás la vista general del escenario en el **Viewport 1**, `ControllerLogic` crea o reutiliza una ventana de Viewport secundaria dedicada (`create_viewport_window("NavSim — Cámara de Seguimiento")`).
     - Crea un Prim `UsdGeom.Camera` hijo del UAV seleccionado (`<uav_path>/NavSimChaseCamera`) posicionado detrás y ligeramente por encima del dron, aplicando una matriz de vista `Gf.Matrix4d().SetLookAt(eye, target, up)` en coordenadas locales del dron.
     - Al cambiar de dron en el selector desplegable, la cámara del Viewport secundario conmuta instantáneamente al nuevo dron sin tocar el Viewport principal.

3. **[`joystick_input.py`](file:///c:/Users/rafae/code/navsim/extensions/manual_controller/manual_controller_python/joystick_input.py) (`JoystickInput`)**:
   - Lee mandos y joysticks directamente a través de `carb.input` / `omni.appwindow` en Isaac Sim, aplicando zona muerta (*deadzone*) y mapeando sticks analógicos y gatillos a traslación 3D, rotación de guiñada (con sentido de giro corregido: izquierda $\rightarrow$ rotación antihoraria $+Z$, derecha $\rightarrow$ rotación horaria $-Z$) y control orbital de cámara.

4. **[`keyboard_input.py`](file:///c:/Users/rafae/code/navsim/extensions/manual_controller/manual_controller_python/keyboard_input.py) (`KeyboardInput`)**:
   - Alternativa de control mediante teclado (`W/A/S/D` para traslación horizontal, `Espacio/Shift` o flechas para altitud y guiñada).

---

## 5. Aplicaciones y Herramientas Externas (`apps/`)

### 5.1. Monitor de Telemetría ASTM F3411 (`apps/telemetry_monitor/`)
- **Archivo principal**: [`apps/telemetry_monitor/telemetry_monitor.py`](file:///c:/Users/rafae/code/navsim/apps/telemetry_monitor/telemetry_monitor.py) (lanzable también con `run_telemetry_monitor.bat`).
- **Funcionamiento**:
  - Es una herramienta de consola interactiva independiente (ejecutable en la misma máquina o en máquinas remotas) que escucha sockets UDP no bloqueantes para los drones indicados (por defecto UAV #0 en `127.0.0.1:16000`).
  - Admite argumentos flexibles: puertos individuales (`16004`), listas (`16000 16005`), rangos (`16000-16009`) o destinos remotos (`192.168.1.50:16000-16003`).
  - Envía datagramas de presencia/suscripción periódica para auto-registrarse en Isaac Sim, permitiendo telemetría remota transparente.
  - Decodifica tramas MAVLink v2 `OPEN_DRONE_ID_*` y `NAVSIM_FLIGHT_PLAN_STATUS`.
  - **Modos de visualización**:
    - **Modo Tabla Interactiva (por defecto)**: Tabla compacta (una fila por aeronave) con posición geodésica WGS-84, alturas AGL/HAE, velocidades polares, rumbo, tasa en Hz y hora UTC exacta (con resolución de milisegundos), acompañada de un panel de detalle inferior para la aeronave seleccionada mediante las teclas `↑`/`↓`.
    - **Modo Stream (`--stream`)**: Emisión continua línea por línea para registro o redirección a ficheros.
    - **Ayuda integrada (`--help` / `-h`)**: Muestra guía completa de uso, formatos y ejemplos.

```powershell
# Ejecución estándar del monitor en una terminal externa:
python apps/telemetry_monitor/telemetry_monitor.py
```

---

## 6. Lecciones Aprendidas y Reglas Críticas para Futuros Agentes (Omniverse Kit / Isaac Sim)

Durante la estabilización de las extensiones en **Isaac Sim (Omniverse Kit)** se resolvieron varios problemas sutiles que cualquier agente futuro **debe respetar estrictamente**:

1. **PROHIBIDO usar `gc.get_objects()` o parches de clases en caliente en `extension.py`**:
   - Nunca escanees `gc.get_objects()` buscando instancias de `NavSim` ni sobrescribas métodos de instancia con `lambda *a, **k: None`. En Omniverse Kit, eso provoca `FutureWarning` en módulos C++/PyTorch cargados en memoria y, peor aún, anula los callbacks de la instancia activa suscrita al stream de eventos C++ de `omni.timeline` y `omni.physx`, dejando la simulación muda al pulsar PLAY.
2. **PROHIBIDO hacer `importlib.reload()` de `extension.py` o `ui_builder.py` dentro de sí mismos**:
   - Recargar el módulo de la propia extensión en mitad de un callback reinicia las variables globales del módulo y desincroniza la instancia viva que mantiene el gestor de extensiones de Kit.
3. **Limpieza explícita en `on_shutdown()`**:
   - Para evitar la advertencia `extension object is still alive, something holds a reference on it` al activar/desactivar extensiones en Isaac Sim:
     - `UIBuilder.cleanup()` debe destruir `self.window` y poner `self.extension = None` para romper la referencia circular `NavSim <-> UIBuilder`.
     - `on_shutdown()` debe llamar a `.unsubscribe()` (si existe) y poner a `None` todas las suscripciones (`on_play_sub`, `on_pause_sub`, `on_stop_sub`, `on_physics_step_sub`, `on_stage_event_sub`).
4. **Compatibilidad de `omni.ui` en ComboBox**:
   - Los sub-ítems de `ui.ComboBox().model` en Omniverse Kit (`omni.ui.AbstractItemModel`) no tienen método `.set_value()`. Para poblar o limpiar un `ComboBox` dinámicamente, utiliza `model.get_item_children()` con `model.remove_item(child)` y `model.append_child_item(None, ui.SimpleStringModel(text))`.
5. **Gestión de archivos pesados en Git**:
   - GitHub rechaza ficheros individuales $> 100\text{ MB}$ y *pushes* masivos de varios gigabytes. Antes de hacer `git add -A` tras editar escenas USD o texturas en `worlds/` o `sims/`, verifica siempre el tamaño de los ficheros nuevos con PowerShell (`Get-ChildItem -Recurse -File | Where-Object { $_.Length -gt 50MB }`).

---

## 7. Estado Actual y Próximos Pasos Sugeridos

### Estado Actual (Verificado y Operativo)
- [x] Reestructuración completa del repositorio (`fleet/`, `mavlink/`, `scripts/`, `sims/`, `vertiports/`, `worlds/`, `extensions/`).
- [x] Activación/desactivación limpia de las extensiones `navsim` y `manual_controller` sin fugas de referencias ni errores en consola.
- [x] Ciclo PLAY / STOP estable en Isaac Sim con inicialización vectorizada `RigidPrim`.
- [x] Pilotaje manual con Joystick en `manual_controller` con cámara de persecución en Viewport secundario independiente y sentido de guiñada (Yaw) corregido.
- [x] Emisión simultánea de telemetría MAVLink v2 ASTM F3411 a 20 Hz por todos los drones (incluyendo el dron pilotado por joystick) y visualización en tiempo real en `scripts/telemetry_monitor/telemetry_monitor.py`.

### Próximos Pasos para Continuar el Desarrollo
1. **Generador / Emisor Externo de Planes de Vuelo Georreferenciados (`scripts/flight_plan_sender/`)**:
   - Crear un script o servicio autónomo fuera de Isaac Sim que permita definir misiones en coordenadas geográficas reales **WGS-84** $(\text{Lat}, \text{Lon}, \text{Alt})$ —o seleccionarlas sobre un mapa / fichero de misión—, las convierta a trayectorias polinómicas 4D y las transmita vía UDP MAVLink (`NAVSIM_FLIGHT_PLAN_HEADER` + `NAVSIM_FLIGHT_PLAN_WAYPOINT`) a los puertos `15000 + idx`.
2. **Integración con Planificadores U-Space Externos**:
   - Conectar servicios externos de gestión de espacio aéreo (U-Space / UTM) consumiendo directamente el flujo de telemetría `OPEN_DRONE_ID_*` en los puertos `16000 + idx` y enviando planes de vuelo dinámicos o resoluciones tácticas de conflictos a los puertos `15000 + idx`.
3. **Sensores Embarcados y Cámaras en Escenarios Reales**:
   - Aprovechar los entornos georreferenciados (`worlds/campus`, `worlds/Lezuza`, `worlds/San_Francisco`, etc.) para pruebas de navegación visual o aterrizaje de precisión en los modelos de `vertiports/`.
