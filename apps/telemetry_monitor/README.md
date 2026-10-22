# Manual de Usuario - Monitor de Telemetría MAVLink v2 (NavSim)

El **Monitor de Telemetría** es una herramienta de consola de alto rendimiento diseñada para inspeccionar en tiempo real el estado de vuelo, posicionamiento geodésico, identificación digital y navegación 4D de las aeronaves simuladas en **NavSim (NVIDIA Isaac Sim)**.

---

## 1. Arquitectura de Red y Esquema de Puertos UDP

La comunicación entre Isaac Sim y las aplicaciones externas (joysticks, estaciones de control GCS y este monitor) opera mediante sockets **UDP** independientes por cada aeronave para evitar cuellos de botella y garantizar escalabilidad para flotas de hasta 1000 drones:

| Canal | Rango de Puertos | Propósito | Rol del Monitor |
| :--- | :--- | :--- | :--- |
| **TX (Telemetría)** | **`16000 + ID`** (16000, 16001, ..., 16999) | Emisión de telemetría desde Isaac Sim hacia el exterior | **Escucha (Receptor pasivo UDP)** |
| **RX (Comandos)** | **`15000 + ID`** (15000, 15001, ..., 15999) | Recepción de mandos y planes de vuelo en Isaac Sim | Informativo (para emisores de control) |

> [!IMPORTANT]
> **¿Por qué el puerto base por defecto es 16000?**  
> El monitor es un receptor de telemetría. Dado que Isaac Sim transmite la telemetría saliente del primer dron (UAV #0) en el puerto UDP **16000**, el monitor escucha de forma predeterminada a partir de dicho puerto.

### Mapeo de puertos por dron
* **UAV #00**: TX `16000` / RX `15000`
* **UAV #01**: TX `16001` / RX `15001`
* **UAV #04**: TX `16004` / RX `15004`
* **UAV #10**: TX `16010` / RX `15010`

---

## 2. Puesta en Marcha Rápida

### Requisitos previos
1. Tener Python 3.10 o superior disponible en el entorno del sistema.
2. Iniciar la simulación en Isaac Sim y pulsar el botón **PLAY** (la física debe estar activa para emitir telemetría).

### Ejecución básica (Drones 0 a 3)
Por defecto, el monitor muestra una ventana de **4 aeronaves consecutivas** para encajar perfectamente en cualquier consola sin scroll vertical:

```bash
# Desde la raíz del repositorio o dentro de scripts/telemetry_monitor:
python scripts/telemetry_monitor/telemetry_monitor.py
```

*En Windows también puedes hacer doble clic sobre el script [`run_telemetry_monitor.bat`](file:///c:/Users/rafae/code/navsim/scripts/telemetry_monitor/run_telemetry_monitor.bat).*

---

## 3. Modos de Ejecución y Parámetros

El monitor acepta argumentos posicionales o parámetros opcionales para seleccionar qué drones monitorizar:

### A. Monitorizar a partir de un puerto de telemetría específico
Si deseas observar los drones a partir del UAV #4, pasa su puerto de telemetría (`16000 + 4 = 16004`):

```bash
python scripts/telemetry_monitor/telemetry_monitor.py 16004
```
*Monitorizará automáticamente 4 drones: UAV #04, UAV #05, UAV #06 y UAV #07.*

### B. Monitorizar indicando directamente el índice del dron
Puedes pasar directamente el número de dron (índice 0-based):

```bash
python scripts/telemetry_monitor/telemetry_monitor.py 4
```
*Equivale exactamente a pasar el puerto `16004`.*

### C. Modificar el número de aeronaves simultáneas (`--count` o `-n`)
Si cuentas con una pantalla de mayor resolución o deseas ver más drones:

```bash
python scripts/telemetry_monitor/telemetry_monitor.py 16000 --count 8
```

### D. Modo Stream (`--stream`)
En lugar de una tabla interactiva en tiempo real, emite cada mensaje recibido línea a línea como un feed continuo (ideal para redirigir a archivos de registro o pipes):

```bash
python scripts/telemetry_monitor/telemetry_monitor.py --stream
```

### Resumen de argumentos disponibles

| Argumento | Tipo | Por defecto | Descripción |
| :--- | :--- | :--- | :--- |
| `port` | Posicional / Entero | `16000` | Puerto TX inicial (o índice de UAV) a monitorizar |
| `-p`, `--port` | Entero | `None` | Alternativa con flag para el puerto o índice inicial |
| `-n`, `--count` | Entero | `4` | Cantidad de aeronaves consecutivas en pantalla |
| `--stream` | Flag | `False` | Salida continua tipo log en vez de cuadro interactivo |
| `--host` | Cadena | `127.0.0.1` | Dirección IP donde se ejecuta Isaac Sim |
| `--timeout` | Flotante | `2.0` | Segundos sin señal antes de marcar timeout |

---

## 4. Estructura de la Pantalla y Datos Mostrados

Cada bloque de dron en la consola presenta información estructurada de acuerdo a los estándares aeronáuticos internacionales:

```text
UAV #00 (UAV_00 | TX:16000 RX:15000) -> [AIRBORNE (En vuelo)]   (Tiempo:  24.50 s sim | Tasa:  1.0 Hz)
  • Identificación UAS: ID: UAV_00               Tipo: Helicóptero / Multirrotor  Tipo ID: Nº Serie
  • Registro Operador:  Operador: ESP-OP-123456  Misión: Plan_UAV_00
  • Posición WGS-84:    Lat: +38.9818000°        Lon:  -1.8570000°        Alt HAE: +695.50 m
  • Alturas ASTM:       Altura (AGL):  9.50 m    Alt Baro: +695.50 m      Ref: Suelo (AGL)
  • Velocidad ASTM:     V. Horiz:  5.20 m/s (18.7 km/h)  V. Vert: +0.20 m/s (Ascenso)
  • Rumbo / Track:      Rumbo/Track:  45.0° (NE )        Exactitud: Horiz <1m | Vert <1m
  • Ruta 4D (Taylor):   Plan: Plan_UAV_00 (AUTO 4D)      WP: 2/5 'WP_CRUISE' (t_k=10.0s)  Ventana: [0.0..35.0s] ( 70.0%)
  • Seguimiento 4D:     Ref Taylor: [+12.5, +30.2, +10.0] m  Error 4D: |e_pos|=0.042 m | |e_vel|=0.015 m/s
------------------------------------------------------------------------------------------------
```

### Descripción de campos
1. **Cabecera**:
   - `UAV #XX`: Índice interno del dron en la simulación.
   - `UAS_ID`: Matrícula o identificador oficial emitido en la trama Remote ID.
   - `TX / RX`: Puertos UDP asignados (TX para telemetría, RX para comandos).
   - `Estado Operativo ASTM F3411`:
     - `GROUND (En tierra)`: Motores en reposo o dron en vertipuerto ($h < 15\text{ cm}$).
     - `AIRBORNE (En vuelo)`: Aeronave sustentada en el aire.
     - `EMERGENCY`: Estado de alerta de vuelo.
     - `DETENIDO / DISARMED`: Motores cortados a 0 RPM.
     - `ESPERANDO CONEXIÓN`: Dron inicializado a la espera del inicio de simulación.
   - `Tasa`: Frecuencia real de recepción de paquetes UDP (configurable en NavSim, por defecto 1.0 Hz según norma ASTM F3411).

2. **Identificación y Operador (ASTM F3411 / ED-282)**:
   - Identificador UAS oficial (`OPEN_DRONE_ID_BASIC_ID`, ID 12900).
   - Tipo de aeronave (Multirrotor, VTOL, etc.).
   - Número de registro oficial CAA del operador (`OPEN_DRONE_ID_OPERATOR_ID`, ID 12905).
   - Misión o propósito dinámico de la operación (`OPEN_DRONE_ID_SELF_ID`, ID 12903).

3. **Geolocalización WGS-84 y Cinemática**:
   - Coordenadas geodésicas calculadas a partir del punto de anclaje georreferenciado de la escena USD.
   - Altura AGL (*Above Ground Level*) calculada con respecto a la superficie del terreno o vertipuerto.
   - Vector de velocidad polar descompuesto en velocidad horizontal ($V_h$) y vertical ($V_z$).
   - Rumbo cinemático (*Track Angle*) y cuadrante cardinal (N, NE, ESE, etc.).

4. **Navegación y Rutas 4D Taylor (`NAVSIM_FLIGHT_PLAN_STATUS`, ID 12003)**:
   - `Plan`: Nombre del plan de vuelo polinómico 4D cargado.
   - `WP`: Waypoint actual, total de waypoints de la misión y etiqueta asignada.
   - `Ventana`: Rango de tiempo de la misión y porcentaje completado.
   - `Ref Taylor`: Posición 3D deseada evaluada por el polinomio $P(t)$ en el instante actual.
   - `Error 4D`: Norma euclídea del error de seguimiento en posición ($|e_{pos}|$) y velocidad ($|e_{vel}|$).

---

## 5. Solución de Problemas (Troubleshooting)

### El monitor muestra "0/4 activos" o "Esperando datagramas UDP"
* **Causa más habitual**: Isaac Sim está en pausa o detenido.
* **Solución**: Pulsa el botón **PLAY** en la barra lateral de Isaac Sim para arrancar la simulación de física. El monitor comenzará a recibir datos instantáneamente.

### Los caracteres de color se ven extraños en consolas antiguas de Windows
* El script habilita automáticamente el procesamiento de terminal virtual de Windows (`ENABLE_VIRTUAL_TERMINAL_PROCESSING`).
* Se recomienda utilizar **Windows Terminal** o PowerShell moderno para una mejor experiencia visual.

### Conflicto de identificación detectado
* Si dos drones comparten la misma matrícula oficial en USD (`uas_id`), el monitor emitirá una alerta visual roja en la parte superior:
  ```text
  🚨 CONFLICTO CRÍTICO EN EL AIRE: Identificador 'UAV_01' duplicado en UAVs [1, 2]! (ASTM F3411 prohíbe IDs duplicados)
  ```
  Para resolverlo, asigna identificadores únicos a cada dron en el archivo USD o en la extensión de NavSim.
