clc
clear

current_velX_MAX = 35;
cmd_rotZ_MAX = 2;

% Valores de x
cmd_velX = linspace(0, current_velX_MAX, 100);



% Valores de lambda
lambdas = [0.065 0.1 0.2];

% Graficar la función de densidad de probabilidad para diferentes valores de lambda
clf;
hold on;
for lambda = lambdas
    factor = exp(-lambda * cmd_velX);
    plot(cmd_velX, factor, 'LineWidth', 1.5, 'DisplayName', ['\lambda = ' num2str(lambda)]);
end





title('Función de densidad de probabilidad exponencial negativa');
xlabel('cmd_velX');
ylabel('factor');
legend('Location', 'best');
grid on;
hold off;



