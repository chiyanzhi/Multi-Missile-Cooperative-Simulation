function [optimal_Z_bias, P_cov_final, P_over_final, runtime, chi_union_final] = DualConstraint(N, target_params, missile_params, weight_params, X_go, Y, Z, P_density, grid_step)
    % 双约束：接收真实蒙特卡洛数据驱动的异构弹群双约束优化引擎
    tic; 
    
    a_t = target_params.a; b_t = target_params.b; 
    a_m = missile_params.a; b_m = missile_params.b; 
    w1 = weight_params.w1; w2 = weight_params.w2; 
    
    % 提取新模型参数 (设置默认值防止报错)
    if isfield(missile_params, 'Pmax'), Pmax = missile_params.Pmax; else, Pmax = 1.0; end
    if isfield(missile_params, 'alpha'), alpha = missile_params.alpha; else, alpha = 10; end
    
    k_SDF = 10; 
    % 根据网格尺度动态设定避碰距离
    d_safe = max(a_t, b_t) * 0.05; 
    dy = grid_step; dz = grid_step; 
    
    % 计算异构导弹各自的物理视场截面半径
    R_fov = X_go .* tan(deg2rad(missile_params.maxFOV));
    
    % 🚨 核心逻辑：强行截断 KDE 场在物理边界之外的拖尾，确保覆盖率分母绝对严谨！
    target_ellipse_val = (Y.^2)/(a_t^2) + (Z.^2)/(b_t^2);
    P_density(target_ellipse_val > 1) = 0; 
    P_density = P_density / (sum(P_density(:)) * dy * dz); % 重新归一化
    
    % --- 初始猜测点 (均匀分布在黑框内部) ---
    z0 = zeros(1, 2*N); 
    for i = 1:N
        theta = 2 * pi * i / N;
        z0(2*i-1) = 0.3 * a_t * cos(theta); 
        z0(2*i) = 0.3 * b_t * sin(theta);   
    end
    
    % --- 优化器设置 ---
    options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
        'Display', 'off', 'MaxFunctionEvaluations', 2000, 'StepTolerance', 1e-6);
    
    % 目标函数与约束 (将 Pmax 和 alpha 传入)
    obj_func = @(vars) -calculateDualConstraintScore(vars, Y, Z, P_density, a_m, b_m, k_SDF, R_fov, Pmax, alpha, w1, w2, dy, dz, N);
    nonlcon = @(vars) collisionConstraint(vars, N, d_safe);
    
    % 上下界设为目标的物理逃逸边界
    lb = repmat([-a_t, -b_t], 1, N);
    ub = repmat([a_t, b_t], 1, N);
    
    % 求解最优偏置点
    optimal_Z_bias = fmincon(obj_func, z0, [], [], [], [], lb, ub, nonlcon, options);
    
    % 提取包含真实概率场并集的输出
    [~, P_cov_final, P_over_final, chi_union_final] = calculateDualConstraintScore(optimal_Z_bias, Y, Z, P_density, a_m, b_m, k_SDF, R_fov, Pmax, alpha, w1, w2, dy, dz, N);
    runtime = toc;
end

%% ====== 计分引擎：计算双重约束的有效得分 ======
function [score, P_cov, P_over, chi_union] = calculateDualConstraintScore(vars, Y, Z, P_density, a_m, b_m, k_SDF, R_fov, Pmax, alpha, w1, w2, dy, dz, N)
    chi_union_inv = ones(size(Y)); 
    sum_chi_i = zeros(size(Y));    
    
    for i = 1:N
        y_i = vars(2*i-1); z_i = vars(2*i);
        
        a_m_i = a_m(i); 
        b_m_i = b_m(i);
        R_fov_i = R_fov(i);
        
        % 支持 Pmax 和 alpha 设为标量或数组
        Pmax_i = Pmax; if length(Pmax) >= i, Pmax_i = Pmax(i); end
        alpha_i = alpha; if length(alpha) >= i, alpha_i = alpha(i); end
        
        % 1. 运动学 SDF (飞得到) -> 保持原有平滑逻辑
        d_kin = 1 - sqrt(((Y - y_i).^2)/(a_m_i^2) + ((Z - z_i).^2)/(b_m_i^2));
        chi_kin = 1 ./ (1 + exp(-k_SDF * d_kin));
        
        % 2. 探测学 SDF (看得到) -> 🚨 严格替换为论文公式 (4)
        dist_to_center = sqrt((Y - y_i).^2 + (Z - z_i).^2);
        chi_fov = Pmax_i ./ (1 + exp(alpha_i .* ((dist_to_center ./ R_fov_i) - 1)));
        
        % 3. 双约束逻辑与融合
        chi_eff = chi_kin .* chi_fov;
        
        chi_union_inv = chi_union_inv .* (1 - chi_eff);
        sum_chi_i = sum_chi_i + chi_eff;
    end
    
    % 计算全弹群的有效覆盖并集
    chi_union = 1 - chi_union_inv;
    
    P_cov = sum(P_density(:) .* chi_union(:)) * dy * dz;
    P_over = sum(P_density(:) .* sum_chi_i(:)) * dy * dz - P_cov;
    
    score = w1 * P_cov + w2 * P_over;
end

%% ====== 避碰约束 ======
function [c, ceq] = collisionConstraint(vars, N, d_safe)
    ceq = []; c = []; idx = 1;
    for i = 1:N-1
        for j = i+1:N
            y_i = vars(2*i-1); z_i = vars(2*i);
            y_j = vars(2*j-1); z_j = vars(2*j);
            dist = sqrt((y_i - y_j)^2 + (z_i - z_j)^2);
            c(idx) = d_safe - dist; 
            idx = idx + 1;
        end
    end
end