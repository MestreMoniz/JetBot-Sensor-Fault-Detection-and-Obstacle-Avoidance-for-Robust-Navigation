function u = goToPoseController(current_pose, target_pose, k_rho, k_alpha, k_beta)
% GOOTOPOSECONTROLLER - Calcula v e w para levar o robô a uma pose final.
%
% ENTRADAS:
%   current_pose - [x; y; theta] do robô
%   target_pose  - [x; y; theta] do alvo
%   k_rho, k_alpha, k_beta - Ganhos do controlador
%
% SAÍDA:
%   u - Comando de controlo [v; w]

    % 1. Calcular rho, alpha, e beta
    delta = target_pose(1:2) - current_pose(1:2);
    rho = norm(delta);
    alpha = wrapToPi(atan2(delta(2), delta(1)) - current_pose(3));
    
    % Se estivermos muito perto, beta não é necessário para evitar instabilidade
    if rho < 0.01
        beta = 0;
    else
        beta = wrapToPi(target_pose(3) - atan2(delta(2), delta(1)));
    end

    % 2. Leis de controlo
    v = k_rho * rho;
    w = k_alpha * alpha + k_beta * beta;
    
    % Clamp para segurança (muito importante!)
    v = min(v, 1.5);
    w = max(-2.0, min(w, 2.0));
    
    % Condição de paragem: se estivermos perto e a apontar na direção certa,
    % paramos a rotação para evitar oscilações.
    if abs(alpha) > pi/2
        % Se o alvo estiver para trás, não andamos para a frente. Forçamos a virar primeiro.
        v = 0;
    end

    u = [v; w];
end