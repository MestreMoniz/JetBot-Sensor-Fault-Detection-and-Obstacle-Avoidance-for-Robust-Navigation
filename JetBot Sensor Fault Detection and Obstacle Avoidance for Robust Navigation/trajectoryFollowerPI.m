function [u, integral_error_new] = trajectoryFollowerPI(current_pose, target_point, params, integral_error_prev)
% trajectoryFollowerPI - Implementa um controlador PI para seguir um ponto.
% Baseado na formulação de "pursuit point".
%
% ENTRADAS:
%   current_pose        - A pose atual do robô [x; y; theta]
%   target_point        - O ponto alvo na trajetória a seguir [x*; y*]
%   params              - Uma struct com os parâmetros do controlador:
%                         .K_v (ganho proporcional da velocidade, Kp do livro)
%                         .K_i (ganho integral da velocidade)
%                         .K_h (ganho proporcional do ângulo)
%                         .d_lookahead (distância de seguimento desejada, d*)
%                         .dt (passo de tempo para a integração)
%   integral_error_prev - O valor do erro integral acumulado do passo anterior.
%
% SAÍDAS:
%   u                   - O comando de controlo calculado [v; w]
%   integral_error_new  - O novo valor do erro integral para usar no próximo passo.

% --- Desempacotar variáveis para clareza ---
x = current_pose(1);
y = current_pose(2);
theta = current_pose(3);

x_star = target_point(1);
y_star = target_point(2);

% --- 1. Calcular o erro de distância 'e' ---
% Distância atual ao ponto alvo
distance_to_target = sqrt((x_star - x)^2 + (y_star - y)^2);
% Erro em relação à distância de seguimento desejada
e = distance_to_target - params.d_lookahead;

% --- 2. Controlador PI para a Velocidade Linear 'v' ---
% Atualizar o termo integral
integral_error_new = integral_error_prev + e * params.dt;
% Calcular a velocidade linear
v = params.K_v * e + params.K_i * integral_error_new;

% --- 3. Controlador P para a Velocidade Angular 'w' ---
% Calcular o ângulo desejado (apontar para o ponto alvo)
theta_star = atan2(y_star - y, x_star - x);
% Calcular o erro de ângulo
angle_error = wrapToPi(theta_star - theta);
% Calcular a velocidade angular
w = params.K_h * angle_error;


% --- 4. Segurança e Saída ---
% Limitar os comandos para valores realistas
v = min(v, 1.5);      % Limitar velocidade máxima
w = max(-2.0, min(w, 2.0)); % Limitar velocidade angular

% Montar o vetor de saída
u = [v; w];

end