% =========================================================================
% Script para Deteção de Falhas usando um Filtro de Kalman Estendido (EKF)
% Lógica: Compara a trajetória medida com a trajetória estimada pelo EKF.
%         Uma divergência significativa indica uma falha no sistema.
% =========================================================================

% --- 1. Inicialização ---
clear; close all; clc;

% Carregar e formatar os dados
load('data.mat');
statehist = state_hist';
actions = actions';
time = time';
N = length(time);

% Parâmetros do EKF e do Detetor
x0 = statehist(:,1);    % Estimativa inicial é o primeiro estado real
P0 = 0.01 * eye(3);     % Incerteza inicial pequena
Q = diag([0.01^2, 0.01^2, (0.01*pi/180)^2]); % Ruído do processo (modelo)
R = diag([1^2, 1^2, (10*pi/180)^2]);         % Ruído da medição (sensor)
threshold = 0.2;        % Limiar de deteção da falha [metros] - Ajustável

%% --- 2. Execução do EKF e Deteção da Falha ---

% Executar o EKF para obter a trajetória estimada
[xhat_hist, P_hist] = run_EKF(statehist, actions, time, Q, R, x0, P0);

% Procurar o primeiro ponto em que a divergência excede o limiar
detected_k = -1; % Inicializa como "não encontrado"
for k = 1:N-1
    error_k = norm(xhat_hist(1:2,k) - statehist(1:2,k));
    if error_k > threshold
        fprintf('Falha detetada em t = %.2f s (k = %d), com um erro de %.3f m\n', time(k), k, error_k);
        detected_k = k;
        break; % Para após a primeira deteção
    end
end

%% --- 3. Visualização dos Resultados ---

figure('Position', [100, 100, 800, 600]);
hold on; 
axis equal; 
grid on;

% Plot das trajetórias
h_true = plot(statehist(1,:), statehist(2,:), 'k-', 'LineWidth', 1.5, ...
                 'DisplayName', 'Trajetória Real (Medida)');
h_est = plot(xhat_hist(1,:), xhat_hist(2,:), 'r--', 'LineWidth', 1.5, ...
                'DisplayName', 'Trajetória Estimada (EKF)');

% Marcar o ponto da falha, se detetada
if detected_k > 0
    h_fault = scatter(statehist(1, detected_k), statehist(2, detected_k), ...
                      150, 'm', 'p', 'filled', 'MarkerEdgeColor', 'k', ...
                      'DisplayName', 'Ponto de Deteção da Falha');
    legend([h_true, h_est, h_fault], 'Location', 'northwest');
else
    legend([h_true, h_est], 'Location', 'northwest');
    disp('Nenhuma falha foi detetada com o limiar atual.');
end

title('Deteção de Falhas por Divergência de Estado do EKF');
xlabel('Posição x (m)');
ylabel('Posição y (m)');
hold off;


%% --- Funções Auxiliares ---

function dx = dynamics_unicycle(x, u)
    % Modelo de movimento não-linear do robô uniciclo.
    theta = x(3);
    v = u(1);
    omega = u(2);

    dx = [v*cos(theta);
          v*sin(theta);
          omega];
end

function [A, B] = linearize_unicycle(x, u, dt)
    % Lineariza o modelo de movimento do uniciclo (cálculo das Jacobianas).
    theta = x(3);
    v = u(1);
    
    A = eye(3);
    A(1,3) = -dt * v * sin(theta);
    A(2,3) = dt * v * cos(theta);

    B = zeros(3,2); % A Jacobiana B não é usada neste EKF, mas está aqui por completude.
    B(1,1) = dt * cos(theta);
    B(2,1) = dt * sin(theta);
    B(3,2) = dt;
end

function [xhat_hist, P_hist] = run_EKF(statehist, actions, time, Q, R, x0, P0)
    % Função principal que implementa o ciclo Predição-Atualização do EKF.
    
    N = size(statehist, 2);
    dt_vec = diff(time);
    xhat = x0;
    P = P0;

    xhat_hist = zeros(3, N);
    P_hist = cell(1,N);
    
    xhat_hist(:,1) = xhat;
    P_hist{1} = P;

    for k = 1:N-1
        dt = dt_vec(k);
        u = actions(:,k);
        z = statehist(:,k+1); % A medição é o próximo estado real

        % --- Fase de Predição ---
        % Integração numérica para maior precisão na predição de estado
        [~, x_ode] = ode45(@(t,x) dynamics_unicycle(x, u), [0 dt], xhat);
        x_pred = x_ode(end, :)';
        
        A = linearize_unicycle(xhat, u, dt); % Lineariza em torno do estado anterior
        P_pred = A * P * A' + Q;

        % --- Fase de Atualização ---
        H = eye(3); % Matriz de medição (observamos o estado diretamente)
        y_tilde = z - x_pred; % Inovação
        S = H * P_pred * H' + R;
        K = P_pred * H' / S; % Ganho de Kalman

        xhat = x_pred + K * y_tilde; % Estado atualizado
        P = (eye(3) - K * H) * P_pred; % Incerteza atualizada

        % Guardar resultados
        xhat_hist(:,k+1) = xhat;
        P_hist{k+1} = P;
    end
end