clc
clear


sm = UTRAFMAN_SimManager;

simulationTime = sm.getSimulationTime();

disp(['Hora del sistema simulado: ' num2str(simulationTime)]);

% Pausar la simulación
sm.pauseSimulation();

% Hacer algo mientras la simulación está pausada...

% Reanudar la simulación
sm.playSimulation();

% Reiniciar la simulación en tiempo 0
sm.resetSimulation();


