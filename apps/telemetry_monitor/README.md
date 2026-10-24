# Manual de Usuario - Monitor de Telemetría MAVLink v2 (NavSim)

El **Monitor de Telemetría** es una herramienta de consola de alto rendimiento diseñada para inspeccionar en tiempo real el estado de vuelo, posicionamiento geodésico WGS-84, identificación digital ASTM F3411 y hora UTC exacta de las aeronaves simuladas en **NavSim (NVIDIA Isaac Sim)**.

---

## 1. Arquitectura de Red y Esquema de Puertos UDP

La comunicación entre Isaac Sim y el monitor opera mediante datagramas **UDP**:

| Canal | Rango de Puertos | Propósito | Rol del Monitor |
| :--- | :--- | :--- | :--- |
| **TX (Telemetría)** | **`16000 + ID`** (16000, 16001, ..., 16999) | Emisión de telemetría desde Isaac Sim hacia el exterior | **Escucha (Receptor pasivo UDP)** |
| **RX (Comandos)** | **`15000 + ID`** (15000, 15001, ..., 15999) | Recepción de mandos y suscripciones en Isaac Sim | **Transmisor de suscripción periódica** |

> [!NOTE]
> **Ejecución en otra máquina**: El monitor envía periódicamente un datagrama ligero de presencia al puerto RX del dron en la máquina de Isaac Sim. NavSim registra la IP de origen y le redirige la telemetría saliente de forma automática.

---

## 2. Puesta en Marcha Rápida

### Requisitos previos
1. Python 3.10 o superior disponible en el entorno del sistema.
2. Iniciar la simulación en Isaac Sim y pulsar **PLAY**.

### Ejecución básica (UAV #0)
Por defecto, si no se pasan argumentos, monitoriza un único dron en `127.0.0.1:16000`:

```bash
python apps/telemetry_monitor/telemetry_monitor.py
```

*En Windows también puedes ejecutar [`run_telemetry_monitor.bat`](file:///c:/Users/rafae/code/navsim/apps/telemetry_monitor/run_telemetry_monitor.bat).*

---

## 3. Parámetros y Selección de Drones

Puedes especificar exactamente qué drones monitorizar indicando puertos, rangos o direcciones `IP:PUERTO`:

### A. Monitorizar un dron concreto por puerto
```bash
python apps/telemetry_monitor/telemetry_monitor.py 16004
```

### B. Monitorizar una lista o rango de drones
```bash
# Varios drones específicos
python apps/telemetry_monitor/telemetry_monitor.py 16000 16005 16010

# Rango continuo (drones 0 al 9)
python apps/telemetry_monitor/telemetry_monitor.py 16000-16009
```

### C. Isaac Sim ejecutándose en otra máquina
```bash
# Monitorizar un rango en una máquina remota
python apps/telemetry_monitor/telemetry_monitor.py 192.168.1.50:16000-16003

# Mezcla de hosts locales y remotos
python apps/telemetry_monitor/telemetry_monitor.py 16000 192.168.1.50:16002
```

### D. Modo Stream continuo (`--stream`)
Emite cada paquete línea a línea en lugar de refrescar la pantalla (ideal para redirigir a archivos de registro o pipes):
```bash
python apps/telemetry_monitor/telemetry_monitor.py 16000-16003 --stream > telemetria.log
```

### E. Ayuda en pantalla (`--help` o `-h`)
Muestra la guía completa de argumentos, formatos y ejemplos:
```bash
python apps/telemetry_monitor/telemetry_monitor.py --help
```

---

## 4. Estructura de la Pantalla

La consola se organiza en dos secciones sin desbordamiento horizontal ni vertical:

```text
NavSim Telemetry Monitor — MAVLink v2 · ASTM F3411 / ED-282
Hora del PC (UTC): 2026-10-24 14:30:24   Drones activos: 2/2
══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
   # UAS ID           Estado     Lat [°]     Lon [°] Alt HAE   AGL    Vh    Vv Rumbo   Hora UTC (dron)          Hz
>  0 ESP-ABJ-2026-00  AIRBORNE  +38.981800  -1.857000   686.5   0.5   0.0  +0.0 090° E  2026-10-24 14:30:24.250  1.0
   1 ESP-ABJ-2026-01  AIRBORNE  +38.981809  -1.857000   686.5   0.5   0.0  +0.0 090° E  2026-10-24 14:30:24.250  1.0
══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
── Detalle UAV #00 · ESP-ABJ-2026-00 ────────────────────────────────────────
 Identificación : ESP-ABJ-2026-00   Tipo ID: Registro CAA   Aeronave: Helicóptero / Multirrotor
 Operador       : ESP-OP-UCLM-100UAV   Misión: Abejorro 00
 Posición       : Lat +38.9818000°   Lon -1.8570000°   Alt HAE +686.53 m
 Alturas        : AGL 0.53 m   Baro +686.53 m   Ref: Suelo (AGL)
 Velocidad      : Horiz 0.00 m/s (0.0 km/h)   Vert +0.00 m/s (Nivelado)   Rumbo 90.0° (E)
 Tiempo         : 2026-10-24 14:30:24.250 UTC   ODID: 1824.2 s tras la hora en punto   (precisión 0,1 s)
 Exactitud      : Horiz <1 m | Vert <1 m | Vel <0,3 m/s
 Ruta 4D        : IDLE (En reposo)   Plan activo: Ninguno
 Enlace         : Isaac Sim 127.0.0.1  TX telemetría 16000  RX comandos 15000  Tasa 1.0 Hz
──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
↑/↓ seleccionar dron · q o Ctrl+C salir · --help para ver la ayuda
```

### Controles interactivos:
* **`↑` / `↓`**: Cambiar la aeronave seleccionada para el panel de detalle inferior.
* **`q`** o **`Ctrl+C`**: Salir limpiamente cerrando los sockets UDP.
