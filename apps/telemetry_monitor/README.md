# Manual de Usuario - Monitor de Telemetría MAVLink v2 (NavSim)

El **Monitor de Telemetría** es una herramienta de consola de alto rendimiento diseñada para inspeccionar en tiempo real el estado de vuelo, posicionamiento geodésico WGS-84, identificación digital ASTM F3411 y hora UTC exacta de las aeronaves simuladas en **NavSim (NVIDIA Isaac Sim)**.

---

## 1. Arquitectura de Red Pub/Sub sobre Puerto Único

La comunicación entre Isaac Sim y el monitor opera mediante un modelo **Publicador/Suscriptor bajo demanda** sobre puertos UDP únicos:

| Extremo | Puerto UDP | Propósito | Rol |
| :--- | :--- | :--- | :--- |
| **Simulador (NavSim)** | **`14550`** (Bidireccional) | Recibe comandos, planes y suscripciones; emite telemetría multiplexada | **Servidor / Publicador** |
| **Monitor (Cliente)** | **`16000`** (Bidireccional) | Emite solicitudes de suscripción y recibe la telemetría de toda la flota | **Cliente / Suscriptor** |

> [!NOTE]
> **Emisión bajo demanda**: El simulador no emite telemetría para ningún dron a menos que exista un suscriptor activo. El monitor envía periódicamente un datagrama de infraestructura `SIM:SUB <puerto_escucha> [target] [ttl]` al puerto `14550`. Si el monitor se cierra o desconecta, la suscripción expira tras 3 segundos y el simulador cesa el envío automáticamente.

---

## 2. Puesta en Marcha Rápida

### Requisitos previos
1. Python 3.10 o superior disponible en el entorno del sistema.
2. Iniciar la simulación en Isaac Sim y pulsar **PLAY**.

### Ejecución básica (Por defecto: Solo Dron 0)
Por defecto, si no se pasan argumentos, monitoriza únicamente el primer dron (**UAV #0**), manteniendo la pantalla limpia y sin desbordamientos:

```bash
python apps/telemetry_monitor/telemetry_monitor.py
```

*En Windows también puedes ejecutar [`run_telemetry_monitor.bat`](file:///c:/Users/rafae/code/navsim/apps/telemetry_monitor/run_telemetry_monitor.bat).*

---

## 3. Selección Sencilla de Drones (Por Número de Índice)

No necesitas escribir las cadenas largas de 20 caracteres de cada aeronave (`ESP-ABJ-2026-XX`). Basta con usar sus **números de índice** directos (0, 1, 2, ..., 99):

### A. Monitorizar un dron concreto por su número
```bash
python apps/telemetry_monitor/telemetry_monitor.py 4
```

### B. Monitorizar varios drones concretos
```bash
python apps/telemetry_monitor/telemetry_monitor.py 0 5 10
```

### C. Monitorizar un rango continuo de drones
```bash
# Primeros 5 drones (del 0 al 4)
python apps/telemetry_monitor/telemetry_monitor.py 0-4

# Drones del 10 al 19
python apps/telemetry_monitor/telemetry_monitor.py 10-19
```

### D. Monitorizar toda la flota completa (`all`)
Para monitorizar todos los drones simultáneamente, pasa la palabra clave `all`. El monitor incluye una **ventana de visualización desplazable (scroll viewport)** para que la tabla nunca desborde la consola y puedas navegar cómodamente con las flechas `↑` y `↓`:
```bash
python apps/telemetry_monitor/telemetry_monitor.py all
```

### E. Isaac Sim ejecutándose en otra máquina
```bash
# Suscribirse al simulador en una máquina remota
python apps/telemetry_monitor/telemetry_monitor.py 0-4 --host 192.168.1.50
```

### F. Múltiples monitores en la misma máquina
Si abres dos consolas para monitorizar la simulación simultáneamente, basta con asignar un puerto local diferente a la segunda instancia:
```bash
python apps/telemetry_monitor/telemetry_monitor.py 0-4 --listen-port 16001
```

### G. Modo Stream continuo (`--stream`)
Emite cada paquete línea a línea en lugar de refrescar la pantalla interactiva (ideal para pipes o redirección a logs):
```bash
python apps/telemetry_monitor/telemetry_monitor.py 0 --stream > telemetria_uav0.log
```

---

## 4. Estructura de la Pantalla

```text
NavSim Telemetry Monitor — MAVLink v2 · ASTM F3411 / ED-282 (Pub/Sub)
Hora PC (UTC): 2026-10-24 14:30:24   Drones activos: 1/1   [Simulador 127.0.0.1:14550 → Puerto Local 16000]
══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
   # UAS ID           Estado     Lat [°]     Lon [°] Alt HAE   AGL    Vh    Vv Rumbo   Hora UTC (dron)          Hz
>  0 ESP-ABJ-2026-00  AIRBORNE  +38.981800  -1.857000   686.5   0.5   0.0  +0.0 090° E  2026-10-24 14:30:24.250  1.0
══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
── Detalle UAV #00 · ESP-ABJ-2026-00 ────────────────────────────────────────
 Identificación : ESP-ABJ-2026-00   Tipo ID: Registro CAA   Aeronave: Helicóptero / Multirrotor
 Operador       : ESP-OP-UCLM-100UAV   Misión: Control Manual
 Posición       : Lat +38.9818249°   Lon -1.8569651°   Alt HAE +687.13 m
 Alturas        : AGL 1.13 m   Baro +687.13 m   Ref: Suelo (AGL)
 Velocidad      : Horiz 0.31 m/s (1.1 km/h)   Vert +0.02 m/s (Nivelado)   Rumbo 49.6° (NE)
 Tiempo         : 2026-10-24 18:57:16.960 UTC   ODID: 3437.0 s tras la hora en punto   (precisión 0,1 s)
 Exactitud      : Horiz <1 m | Vert <1 m | Vel <0,3 m/s
 Ruta 4D        : MANUAL (Control directo)   Plan activo: Ninguno
 Enlace         : Isaac Sim 127.0.0.1:14550  Puerto Escucha: 16000  Tasa: 1.0 Hz
──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
↑/↓ seleccionar dron · q o Ctrl+C salir · --help para ver la ayuda
```
