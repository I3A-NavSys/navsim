clc
clear

% Ángulos de Euler en radianes (roll y pitch)
phi   = deg2rad(45);  % Roll
theta = deg2rad(45);  % Pitch

% Calcular la matriz de transformación angular
Omega_cuerpo_tierra = [0    0   cos(phi)*tan(theta); 
                       0    0  -sin(phi); 
                       0    0   cos(phi)/cos(theta)];

% Imprimir la matriz de transformación angular
disp('Matriz de transformación angular:');
disp(Omega_cuerpo_tierra);

% Velocidad angular alrededor del eje Z absoluto (en radianes por segundo)
dot_psi_absoluto = -1;

% Descomponer la velocidad angular alrededor del eje Z absoluto en velocidades angulares del cuerpo
angular_velocities_body = Omega_cuerpo_tierra * [0; 0; dot_psi_absoluto];

% Imprimir las velocidades angulares del cuerpo
disp('Velocidades angulares del cuerpo (p, q, r):');
disp(angular_velocities_body);
