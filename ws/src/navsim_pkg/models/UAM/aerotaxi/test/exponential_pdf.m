% Valores de x
x = linspace(0, 5, 100);

% Valores de lambda
lambdas = [0.5, 1, 1.5];

% Graficar la función de densidad de probabilidad para diferentes valores de lambda
figure;
hold on;
for i = 1:length(lambdas)
    lambda = lambdas(i);
    y = lambda * exp(-lambda * x);
    plot(x, y, 'LineWidth', 1.5, 'DisplayName', ['\lambda = ' num2str(lambda)]);
end

title('Función de densidad de probabilidad exponencial negativa');
xlabel('x');
ylabel('f(x)');
legend('Location', 'best');
grid on;
hold off;



