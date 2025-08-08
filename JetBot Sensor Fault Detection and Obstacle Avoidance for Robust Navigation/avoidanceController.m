function [x_pred_collision,ref_trajectory, avoidance_active] = avoidanceController(x_current, u_current, obs_center, obs_radius,safety_margin)
% avoidanceController - Decide se uma manobra de desvio é necessária e, se for, calcula-a.
%
% ENTRADAS:
%   x_current  - O estado atual do robô [x; y; theta]
%   u_current  - O comando de controlo atual [v; w]
%   obs_center - O centro do obstáculo [x_c; y_c]
%   obs_radius - O raio do obstáculo
%   safety_margin - % Margem de segurança adicional ao raio do obstáculo

%
% SAÍDAS:
%                      (Pode ser o mesmo que u_current ou um novo para o desvio)
%   ref_trajectory   - A trajetória de referência para o desvio. Fica vazia se não houver desvio.
%   avoidance_active - Um booleano (true/false) que indica se o modo de desvio foi ativado.

% --- Parâmetros de Configuração do Controlador ---
% Estes parâmetros podem ser passados como argumentos se quiser mais flexibilidade.
dt_pred = 0.1;       % Passo de tempo para a predição (deve corresponder ao da simulação)
N_pred = 5;          % Horizonte de predição (quantos passos à frente olhar)
avoid_steps = 80;    % Duração (em passos) do arco de desvio

% Inicializar saídas para o caso de não haver colisão
ref_trajectory = [];
avoidance_active = false;
x_pred_collision = [];

% --- 1. Predição da Trajetória Futura ---
% Usando o estado atual (x_current) e o controlo atual (u_current), prevemos onde o robô estará.
x_pred = predictTrajectory(x_current, u_current, N_pred, dt_pred);

% --- 2. Deteção de Colisão ---
% Verificamos se a trajetória prevista colide com a zona de segurança do obstáculo.
if isCollision(x_pred, obs_center, obs_radius, safety_margin)
    fprintf('ALERTA: Colisão prevista! A ativar manobra de desvio.\n');
    avoidance_active = true;
    
    % --- 3. Geração da Trajetória de Desvio (se houver colisão) ---
    
    % a) Determinar a direção do desvio (esquerda/direita)
    vec_to_obs = obs_center - x_current(1:2);
    heading_vec = [cos(x_current(3)); sin(x_current(3))];
    cross_prod_z = heading_vec(1) * vec_to_obs(2) - heading_vec(2) * vec_to_obs(1);
    avoid_dir = sign(cross_prod_z);
    if avoid_dir == 0, avoid_dir = 1; end % Caso especial: a ir direto ao centro
    
    % b) Gerar a trajetória de referência circular
    effective_radius = obs_radius + safety_margin;
    ref_trajectory = generateCircularRef(x_current, obs_center, effective_radius, avoid_dir, avoid_steps);
    x_pred_collision = x_pred;
    
else
    % Se não houver colisão, não fazemos nada. As saídas mantêm os valores padrão.
end

end

% --- Funções Auxiliares (podem estar no mesmo ficheiro ou em ficheiros separados) ---

function x_pred = predictTrajectory(x0, u, N, dt)
    x_pred = zeros(3, N);
    x = x0;
    v = u(1);
    omega = u(2);
    for k = 1:N
        theta = x(3);
        x = x + dt * [v * cos(theta); v * sin(theta); omega];
        x_pred(:,k) = x;
    end
end

function flag = isCollision(x_pred, center, radius, margin)
    flag = false;
    for k = 1:size(x_pred,2)
        dist_to_center = norm(x_pred(1:2,k) - center);
        if dist_to_center < (radius + margin)
            flag = true;
            return;
        end
    end
end

function ref = generateCircularRef(x_start, center, radius, dir, total_steps)
    ref = zeros(3, total_steps);
    offset = x_start(1:2) - center;
    angle0 = atan2(offset(2), offset(1));
    for k = 1:total_steps
        angle = angle0 + dir * pi * k / total_steps;
        pos = center + radius * [cos(angle); sin(angle)];
        theta = angle + dir * pi / 2;
        ref(:,k) = [pos; theta];
    end
end


