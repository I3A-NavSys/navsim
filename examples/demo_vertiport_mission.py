"""
Script de ejemplo: Misión de Vuelo entre Vertipuertos con NavSim FlightPlan.

Simula el ciclo de misión completo de un dron:
  1. Despegue vertical desde la plataforma (vertipuerto origen).
  2. Ascenso y aceleración hacia altitud de crucero.
  3. Paseo / circuito panorámico aéreo en 3D.
  4. Entrada al cono de aproximación vertical del vertipuerto.
  5. Descenso vertical controlado y toma de tierra (touchdown).

Visualiza la trayectoria en 3D y componentes cinemáticas mediante las primitivas
integradas de `FlightPlan.plot_position()`.
"""

import os
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

# Garantizar que libs/ sea accesible desde cualquier directorio de ejecución
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from libs.flight_plan import FlightPlan, Waypoint


def create_vertiport_mission() -> FlightPlan:
    """Construye un plan de vuelo completo con despegue, crucero y aterrizaje."""
    fp = FlightPlan(id="MISSION_VERTIPORT_01", priority=1, radius=2.5)

    print("Definiendo waypoints de la mision en el vertipuerto...")

    # --- FASE 1: DESPEGUE VERTICAL (Pad en [100, 100, 0]) ---
    fp.add_waypoint(
        id="WP0_PAD",
        time=0.0,
        pos=[100.0, 100.0, 0.0],
        vel=[0.0, 0.0, 0.0],
        heading=[1.0, 0.0]
    )
    fp.add_waypoint(
        id="WP1_CLIMB",
        time=15.0,
        pos=[100.0, 100.0, 30.0],
        vel=[0.0, 0.0, 2.0],
        heading=[1.0, 0.0]
    )

    # --- FASE 2: TRANSICION A CRUCERO HORIZONTAL ---
    fp.add_waypoint(
        id="WP2_TRANS",
        time=35.0,
        pos=[160.0, 130.0, 50.0],
        vel=[5.0, 2.5, 0.5],
        heading=[1.0, 0.5]
    )

    # --- FASE 3: PASEO PANORAMICO / CIRCUITO AEREO ---
    fp.add_waypoint(
        id="WP3_TURN1",
        time=65.0,
        pos=[280.0, 240.0, 65.0],
        vel=[4.0, 6.0, 0.0],
        heading=[0.5, 1.0]
    )
    fp.add_waypoint(
        id="WP4_APEX",
        time=95.0,
        pos=[220.0, 380.0, 60.0],
        vel=[-4.0, 5.0, -0.2],
        heading=[-0.5, 1.0]
    )
    fp.add_waypoint(
        id="WP5_WEST",
        time=125.0,
        pos=[60.0, 330.0, 55.0],
        vel=[-7.0, -2.0, 0.0],
        heading=[-1.0, -0.2]
    )
    fp.add_waypoint(
        id="WP6_TURN2",
        time=155.0,
        pos=[-10.0, 190.0, 48.0],
        vel=[-2.0, -6.0, -0.2],
        heading=[-0.2, -1.0]
    )

    # --- FASE 4: ENTRADA A CONO DE APROXIMACION VERTICAL ---
    fp.add_waypoint(
        id="WP7_APPROACH",
        time=185.0,
        pos=[65.0, 120.0, 38.0],
        vel=[3.5, -2.5, -0.5],
        heading=[0.8, -0.6]
    )
    fp.add_waypoint(
        id="WP8_HOVER_PAD",
        time=210.0,
        pos=[100.0, 100.0, 20.0],
        vel=[0.0, 0.0, -1.0],
        heading=[0.0, -1.0]
    )

    # --- FASE 5: DESCENSO CONTROLADO Y TOMA DE TIERRA (TOUCHDOWN) ---
    fp.add_waypoint(
        id="WP9_FINAL",
        time=225.0,
        pos=[100.0, 100.0, 4.0],
        vel=[0.0, 0.0, -0.4],
        heading=[0.0, -1.0]
    )
    fp.add_waypoint(
        id="WP10_TOUCHDOWN",
        time=235.0,
        pos=[100.0, 100.0, 0.0],
        vel=[0.0, 0.0, 0.0],
        heading=[0.0, -1.0]
    )

    # Conectar waypoints mediante interpolacion polinomica de 5º orden
    print("Conectando trayectoria con splines polinomicos de 5.o orden...")
    fp.connect_waypoints()

    return fp


def main():
    print("=" * 70)
    print("  NavSim - Demostracion de Plan de Vuelo en Vertipuerto (3D)")
    print("=" * 70)

    fp = create_vertiport_mission()

    print("\nResumen del plan de vuelo generado:")
    print(repr(fp))

    # Muestreo cinemático de la trayectoria completa
    time_step = 0.25  # Muestreo a 4 Hz
    trace = fp.trace(time_step=time_step)
    
    times = trace[:, 0]
    positions = trace[:, 1:4]
    velocities = trace[:, 4:7]
    accelerations = trace[:, 7:10]

    # Estadísticas clave de la misión
    total_time = times[-1] - times[0]
    distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    total_distance = np.sum(distances)
    speed_magnitudes = np.linalg.norm(velocities, axis=1)
    acc_magnitudes = np.linalg.norm(accelerations, axis=1)

    print("\n--- METRICAS DE LA MISION ---")
    print(f"Duracion total : {total_time:.1f} s ({total_time / 60:.2f} min)")
    print(f"Distancia total: {total_distance:.1f} m")
    print(f"Altitud maxima : {np.max(positions[:, 2]):.1f} m")
    print(f"Velocidad max  : {np.max(speed_magnitudes):.2f} m/s ({np.max(speed_magnitudes) * 3.6:.1f} km/h)")
    print(f"Aceleracion max: {np.max(acc_magnitudes):.2f} m/s^2")

    # Renderizado gráfico con las primitivas nativas de FlightPlan
    fig_name = "NavSim - Mision Vertipuerto 3D"
    print(f"\nGenerando visualizacion grafica 3D con '{fig_name}'...")
    fp.plot_position(fig_name=fig_name, time_step=time_step)

    # Guardar captura en el directorio de ejemplos
    output_png = Path(__file__).resolve().parent / "vertiport_mission_3d.png"
    plt.savefig(output_png, dpi=120, bbox_inches="tight")
    print(f"Captura guardada en: {output_png}")

    # Mostrar ventana interactiva de Matplotlib
    print("Abriendo ventana grafica interactiva 3D (puedes rotar el modelo con el raton)...")
    plt.show(block=True)


if __name__ == "__main__":
    main()
