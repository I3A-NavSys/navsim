# Guía para Añadir Nuevos Modelos de Dron a `fleet/`

Esta carpeta contiene el catálogo de aeronaves disponibles para el simulador **NavSim**. El sistema está diseñado para que cualquier nuevo modelo sea descubierto y simulado de forma **100 % automática** sin necesidad de modificar el código del simulador ni de la extensión.

---

## 1. Estructura de Directorios

Para dar de alta un nuevo modelo de dron, crea una carpeta con el identificador del modelo:

```text
fleet/
└── <Nombre_Del_Modelo>/
    ├── 1_rb_model.usd      <-- Modelo 3D / Asset USD (Cuerpo Rígido Único)
    └── dynamics.yaml       <-- Ficha técnica: física, aerodinámica y matrices de control
```

> **Ejemplo:** Para un nuevo hexacóptero de carga llamado `Cargo_H6`, crearías la carpeta `fleet/Cargo_H6/` conteniendo `1_rb_model.usd` y `dynamics.yaml`.

---

## 2. Requisitos del Modelo 3D (`1_rb_model.usd`)

El simulador utiliza una arquitectura de **un único cuerpo rígido (`Single Rigid Body`)** por dron para permitir simular cientos de aparatos a más de 100 Hz en Fabric/PhysX:

1. **Cuerpo Rígido Único**: El modelo principal debe ser un prim con API de `PhysicsRigidBodyAPI` (sin articulaciones mecánicas ni rotores con físicas independientes).
2. **Centro de Masas (CoM)**: El origen de coordenadas local $(0,0,0)$ del modelo USD debe situarse en el Centro de Masas de la aeronave.
3. **Ejes de Coordenadas (Cuerpo)**:
   * $+X$: Hacia adelante (Proa / Morro).
   * $+Y$: Hacia la izquierda (Babor / West).
   * $+Z$: Hacia arriba (Cenit).
4. **Visualización de Rotores**: Si el modelo incluye mallas visuales para las hélices, deben ser elementos meramente visuales (hijos del prim principal o mallas estáticas), nunca cuerpos físicos independientes.

---

## 3. Ficha Técnica y Matrices de Control (`dynamics.yaml`)

Copia la plantilla base mostrada a continuación y ajústala con los valores de tu aeronave.

### Orden Canónico de Rotores
Todos los vectores y filas de matrices utilizan estrictamente el **orden canónico de 4 cuadrantes**:
* **Índice 0: NE** (North-East: Delantero Derecho, $+X, -Y$)
* **Índice 1: NW** (North-West: Delantero Izquierdo, $+X, +Y$)
* **Índice 2: SE** (South-East: Trasero Derecho, $-X, -Y$)
* **Índice 3: SW** (South-West: Trasero Izquierdo, $-X, +Y$)

---

### Plantilla `dynamics.yaml`

```yaml
# ==============================================================================
# Identificación del Modelo
# ==============================================================================
model_name: "Cargo_H6"              # Nombre canónico exacto (coincidente con la carpeta)
aliases:                            # Alias opcionales para asociarlo a prims existentes
  - "cargo_h6"
  - "heavy_cargo"
usd_file: "1_rb_model.usd"          # Nombre del archivo USD dentro de esta carpeta

# ==============================================================================
# Propiedades Físicas e Inerciales
# ==============================================================================
physics:
  gravity: 9.81                     # Aceleración gravitatoria [m/s²]
  mass: 12.5                        # Masa total de la aeronave [kg]
  inertia: [0.25, 0.25, 0.45]       # Matriz diagonal de inercia [Ix, Iy, Iz] [kg·m²]
  
  # Posición [x, y, z] en metros de cada punto respecto al Centro de Masas (0, 0, 0):
  #   - Índice 0: Centro de Masas (Fuselaje)
  #   - Índice 1: Rotor NE
  #   - Índice 2: Rotor NW
  #   - Índice 3: Rotor SE
  #   - Índice 4: Rotor SW
  bodies_positions:
    - [ 0.00,  0.00, 0.00]          # 0: CoM
    - [ 0.35, -0.35, 0.05]          # 1: Rotor NE (+X, -Y)
    - [ 0.35,  0.35, 0.05]          # 2: Rotor NW (+X, +Y)
    - [-0.35, -0.35, 0.05]          # 3: Rotor SE (-X, -Y)
    - [-0.35,  0.35, 0.05]          # 4: Rotor SW (-X, +Y)

# ==============================================================================
# Parámetros Aerodinámicos y de Motor
# ==============================================================================
aerodynamics:
  w_min: 0.0                        # Velocidad angular mínima de rotor [rad/s]
  w_max: 850.0                      # Velocidad angular máxima de rotor [rad/s]
  w_hov: 420.0                      # Velocidad angular en sustentación (hover) [rad/s]
  
  # Coeficientes de empuje por rotor FT_i = kFT_i * w_i² [NE, NW, SE, SW]
  kFT: [3.45e-05, 3.45e-05, 3.45e-05, 3.45e-05]
  
  # Coeficientes de par de arrastre de rotor en Z (incluye signo de rotación del motor):
  # MDR_z = sum(kMDR_i * w_i²)  [NE (+), NW (-), SE (-), SW (+)]
  kMDR: [8.5e-07, -8.5e-07, -8.5e-07, 8.5e-07]
  
  # Coeficientes de arrastre lineal del fuselaje: FD = -kFD * |v| * v  [x, y, z]
  kFD: [0.05, 0.05, 0.12]
  
  # Coeficientes de momento de fricción angular: MD = -kMD * |w| * w  [roll, pitch, yaw]
  kMD: [0.015, 0.015, 0.008]

# ==============================================================================
# Matrices de Control Lineal en Espacio de Estados
# ==============================================================================
# Ley de control implementada:
#   u = Hs - Kx @ x - Ky @ E
#
# Vector de estado x (8x1): [roll, pitch, wx, wy, wz, vx_body, vy_body, vz_body]^T
# Vector de error  e (4x1): [vx_body - r_vx, vy_body - r_vy, vz_body - r_vz, wz - r_wz]^T
# Vector de mando  u (4x1): [w_NE, w_NW, w_SE, w_SW]^T (velocidades de rotores)
control:
  t_to_solve: 2.0                   # Horizonte temporal de corrección de comando [s]
  E_max: 25.0                       # Límite de saturación para el error integral E
  Hs: [420.0, 420.0, 420.0, 420.0]  # Velocidades base de hover para los 4 rotores [rad/s]
  
  # Matriz de realimentación de estados Kx (dimensión 4x8)
  # Filas: [NE, NW, SE, SW]
  # Columnas: [roll, pitch, wx, wy, wz, vx_body, vy_body, vz_body]
  Kx:
    - [-25.10, -25.10, -4.80, -4.80,  180.20, -6.50,  6.50,  75.40]
    - [ 25.10, -25.10,  4.80, -4.80, -180.20, -6.50, -6.50,  75.40]
    - [-25.10,  25.10, -4.80,  4.80, -180.20,  6.50,  6.50,  75.40]
    - [ 25.10,  25.10,  4.80,  4.80,  180.20,  6.50, -6.50,  75.40]

  # Matriz de acción integral Ky (dimensión 4x4)
  # Filas: [NE, NW, SE, SW]
  # Columnas: [e_vx, e_vy, e_vz, e_wz]
  Ky:
    - [-4.50,  4.50, 150.00,  450.00]
    - [-4.50, -4.50, 150.00, -450.00]
    - [ 4.50,  4.50, 150.00, -450.00]
    - [ 4.50, -4.50, 150.00,  450.00]
```

---

## 4. Cómo se Instancia el Modelo en un Escenario USD

Una vez creada la carpeta:

1. Abre cualquier escenario en Isaac Sim (por ejemplo `worlds/madrid/main_scene.usd`).
2. Haz un **Reference** o arrastra el archivo `fleet/<Nombre_Del_Modelo>/1_rb_model.usd` al Stage.
3. Asigna al Prim una ruta que siga el patrón `.../UAV_<id>` (por ejemplo `/World/UAVs/UAV_05`).
4. *(Opcional)* En los atributos del Prim en USD puedes especificar:
   * `NavSim:id`: Identificador del dron (ej. `"UAV_05"`).
   * `NavSim:model`: Nombre exacto del modelo o carpeta (ej. `"Cargo_H6"`). Si no se especifica, el simulador lo deduce automáticamente por el nombre del Prim o la ruta del archivo referenciado.

Al pulsar **PLAY**, el simulador:
* Detectará el nuevo modelo.
* Aplicará automáticamente su masa e inercia configuradas.
* Integrará sus matrices $K_x$ y $K_y$ en el cálculo matricial vectorizado.
* Abrirá un puerto UDP MAVLink (`14550 + índice`) para recibir trayectorias y emitir telemetría.
