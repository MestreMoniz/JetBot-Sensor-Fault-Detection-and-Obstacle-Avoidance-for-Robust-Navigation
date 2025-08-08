% =========================================================================
% SCRIPT DE SIMULAÇÃO DE DESVIO DE OBSTÁCULOS COM CONTROLO HÍBRIDO
% Arquitetura:
%   - Modo Normal: Segue as ações pré-gravadas do ficheiro `data.mat`.
%   - Modo Avoidance: Usa um controlador PI para seguir o arco de desvio.
%   - Modo Resume: Usa um controlador de Kanayama (Go-to-Pose) para o
%     alinhamento final com a pose de reencontro.
% =========================================================================

clear; close all; clc;

%% --- 1. Carregar e Preparar Dados ---
% Carrega os dados experimentais e formata as matrizes para a simulação.
load('data.mat');
statehist = state_hist';
actions = actions';
time = time';
N = length(time);

%% --- 2. Parâmetros da Simulação e do Ambiente ---
dt = 0.1;
obs_center = [0.5; 0.5];
obs_radius = 0.25;
safety_margin = 0.05;

% Parâmetro da estratégia de desvio
rejoin_offset = 30; % Define quantos passos à frente na trajetória original
                    % será o nosso ponto de reencontro.

%% --- 3. Parâmetros dos Controladores ---
% Esta secção define os ganhos para os dois controladores autónomos.

% Parâmetros para o controlador de Seguimento de Trajetória (PI)
% Usado no modo 'avoidance' para seguir o arco.
follower_params.K_v = 1.0;   % Ganho Proporcional da velocidade
follower_params.K_i = 0.2;   % Ganho Integral da velocidade
follower_params.K_h = 2.0;   % Ganho Proporcional do ângulo
follower_params.d_lookahead = 0.0; % Distância desejada ao ponto alvo
follower_params.dt = dt;     % Passo de tempo para a integração

% Parâmetros para o controlador Go-to-Pose (Kanayama)
% Usado no modo 'resume' para o alinhamento de pose.
goToPose_params.K_rho = 0.8;
goToPose_params.K_alpha = 1.5;
goToPose_params.K_beta = -0.6;

%% --- 4. Inicialização da Simulação ---
% Prepara todas as variáveis de estado e de histórico antes do loop.
N_sim = N + rejoin_offset;
x = zeros(3, N_sim);
x(:, 1) = statehist(:, 1);
u = actions(:, 1);

% Máquina de Estados e Variáveis de Suporte
mode = "normal";
mode_log = strings(1, N_sim);
k_actions = 1; % Índice para ler as ações do ficheiro 'actions'

% Variáveis de Desvio
avoid_ref = [];
k_avoid = 1;
rejoin_waypoint = [];
k_decision = -1;
follower_integral_error = 0; % Estado do integrador do controlador PI
x_pred_collision_h = []; % Histórico das predições de colisão

%% --- 5. Loop Principal da Simulação ---
for k = 1:N_sim - 1 % Ajustado para um loop de tamanho correto
    mode_log(k) = mode;
    
    % A máquina de estados decide qual controlador usar em cada passo.
    switch mode
        case "normal"
            % No modo normal, o robô executa passivamente as ações do ficheiro
            % enquanto verifica continuamente se a trajetória é segura.
            if k_actions <= N
                u = actions(:, k_actions);
                k_actions = k_actions + 1;
            else
                u = [0; 0]; % Fim dos dados, o robô para.
            end
            
            [x_pred_collision, new_avoid_ref, is_avoid_needed] = avoidanceController(x(:,k), u, obs_center, obs_radius, safety_margin);
            
            if is_avoid_needed
                % Se um risco é detetado, o sistema assume o controlo.
                disp('Colisão iminente! A ativar modo de desvio.');
                mode = "avoidance";
                k_decision = k_actions - 1;
                avoid_ref = new_avoid_ref;
                k_avoid = 1;
                rejoin_index = min(k_decision + rejoin_offset, N);
                rejoin_waypoint = statehist(:, rejoin_index); % O alvo é a POSE completa
                follower_integral_error = 0; % Reinicia o integrador
                x_pred_collision_h = [x_pred_collision_h; x_pred_collision]; % Guarda para visualização
            end
    
       case "avoidance"
            % No modo de desvio, o robô usa o controlador PI para seguir o arco
            % enquanto verifica se já pode ir em segurança para o ponto de reencontro.
            path_is_now_clear = isPathClear(x(1:2, k), rejoin_waypoint(1:2), obs_center, obs_radius + safety_margin);
            
            if path_is_now_clear
                disp('Caminho para o reencontro livre. A iniciar alinhamento final.');
                mode = "resume";
            else
                % Se o caminho não está livre, segue o arco.
                if k_avoid <= size(avoid_ref, 2)
                    target_point_on_arc = avoid_ref(1:2, k_avoid);
                    [u, follower_integral_error] = trajectoryFollowerPI(x(:,k), target_point_on_arc, follower_params, follower_integral_error);
                    k_avoid = k_avoid + 1;
                else
                    disp('Fim do arco. A forçar modo de alinhamento.');
                    mode = "resume";
                end
            end
            
        case "resume"
            % No modo de retoma, o objetivo é atingir a POSE de reencontro.
            % O controlador de Kanayama (Go-to-Pose) é a ferramenta ideal para esta tarefa.
            dist_to_wp = norm(x(1:2, k) - rejoin_waypoint(1:2));
            angle_diff_to_wp = abs(wrapToPi(x(3, k) - rejoin_waypoint(3)));
            
            if dist_to_wp < 0.1 && angle_diff_to_wp < 0.1
                disp('Chegámos à pose de reencontro. A retomar controlo original.');
                mode = "normal";
                k_actions = min(k_decision + rejoin_offset, N); % Sincroniza o leitor de ações
            else
                u = goToPoseController(x(:,k), rejoin_waypoint, goToPose_params.K_rho, goToPose_params.K_alpha, goToPose_params.K_beta);
            end
    end
    
    % --- ATUALIZAÇÃO DA DINÂMICA DO ROBÔ ---
    % Simula o movimento do robô com o comando 'u' final (seja ele do ficheiro ou calculado).
    x(1, k+1) = x(1, k) + dt * u(1) * cos(x(3, k));
    x(2, k+1) = x(2, k) + dt * u(1) * sin(x(3, k));
    x(3, k+1) = wrapToPi(x(3, k) + dt * u(2));
end

mode_log(N_sim) = mode;
    
%% --- 6. VISUALIZAÇÃO ---

figure('Position', [100, 100, 1000, 700]); % Made figure slightly wider for legend
hold on;
axis equal;
grid on;

% 1. Plot the original reference trajectory
plot(statehist(1, :), statehist(2, :), '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 2, ...
    'DisplayName', 'Trajetória Original');

% 2. Plot Obstacle and Safety Margin
% Note: viscircles does not support DisplayName directly. We create dummy plots for the legend later.
viscircles(obs_center', obs_radius, 'Color', 'r', 'LineWidth', 2);
viscircles(obs_center', obs_radius + safety_margin, 'Color', [0.8 0 0], 'LineStyle', '--');

% 3. Plot the avoidance arc reference path (if it exists)
if ~isempty(avoid_ref)
    plot(avoid_ref(1,:), avoid_ref(2,:), 'g:', 'LineWidth', 2, ...
        'DisplayName', 'Ref. de Desvio (Arco)');
end

% 4. Plot the rejoin waypoint (if it exists)
if ~isempty(rejoin_waypoint)
    scatter(rejoin_waypoint(1), rejoin_waypoint(2), 150, 'c', 'd', 'filled', ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Ponto de Reencontro');
end

% 5. Plot the actual robot path, highlighted by mode
final_k = find(any(x, 1), 1, 'last')-1;
if ~isempty(final_k)
    % Find indices for each mode up to the final point
    normal_indices = find(mode_log(1:final_k) == "normal");
    avoid_indices = find(mode_log(1:final_k) == "avoidance");
    resume_indices = find(mode_log(1:final_k) == "resume");
    
    % Plot each segment with its own name
    plot(x(1, normal_indices), x(2, normal_indices), 'b-', 'LineWidth', 2, ...
        'DisplayName', 'Trajetória - Modo Normal');
    plot(x(1, avoid_indices), x(2, avoid_indices), '-', 'Color', [1 0.5 0], 'LineWidth', 4, ...
        'DisplayName', 'Trajetória - Modo Desvio');
    plot(x(1, resume_indices), x(2, resume_indices), 'm-', 'LineWidth', 3, ...
        'DisplayName', 'Trajetória - Modo Alinhamento');
end

% 6. Plot the predicted trajectory that triggered the switch (if it exists)
if  ~isempty(x_pred_collision_h)
    plot(x_pred_collision_h(1,:), x_pred_collision_h(2,:), 'k--', 'LineWidth', 1.5, ...
        'DisplayName', 'Predição de Colisão');
end

% 7. Create Dummy Plots for items that don't support DisplayName (like viscircles)
% We plot them outside the visible area so they don't show up.
h_obs = plot(NaN, NaN, '-r', 'LineWidth', 2, 'DisplayName', 'Obstáculo');
h_margin = plot(NaN, NaN, '--', 'Color', [0.8 0 0], 'DisplayName', 'Margem de Segurança');


% 8. Generate the legend automatically and set its location
legend('show', 'Location', 'eastoutside');

title('Simulação com Desvio e Reencontro Estratégico');
xlabel('x (metros)');
ylabel('y (metros)');
hold off;