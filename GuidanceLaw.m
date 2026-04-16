function guidanceAccel = GuidanceLaw(missileState, targetState, guidanceParams, environment)
    % GUIDANCELAW 协同偏置比例导引 (Cooperative Biased PN)
    % 融合真实目标追踪与多弹3D空间阵位成型约束
    
    % ================= 1. 基础状态计算 =================
    r_vec = targetState.pos - missileState.pos;
    dist = norm(r_vec);
    v_rel = targetState.vel - missileState.vel;
    Vm = norm(missileState.vel);
    missileDir = missileState.vel / (Vm + 1e-6);
    g = environment.g;
    
    % 获取初始距离与交接班距离 (防报错保护)
    if isfield(guidanceParams, 'initial_dist'), D0 = guidanceParams.initial_dist; else, D0 = 100000; end
    if isfield(guidanceParams, 'handover_dist'), X_go = guidanceParams.handover_dist; else, X_go = 27000; end
    if isfield(guidanceParams, 'N'), N = guidanceParams.N; else, N = 4; end
    
    % ================= 2. 基础 PN 计算 (始终紧盯真实目标) =================
    % 视线角速度 (Omega)
    omega = cross(r_vec, v_rel) / (dot(r_vec, r_vec) + 1e-6);
    
    % 纯比例导引指令 (Pure PN)
    acc_pn = N * Vm * cross(omega, missileDir);
    
    % ================= 3. 3D 空间协同偏置逻辑 (Cooperative Bias) =================
    bias_vec = [0, 0, 0];
    
    % 检查是否传入了协同占位参数 y_offset 和 z_offset (多弹模式)
    if isfield(guidanceParams, 'y_offset') && isfield(guidanceParams, 'z_offset')
        % 处于中制导段 (大于交接班距离)
        if dist > X_go
            % a. 计算平滑衰减权重 W (从发射到交接班，W由 1 平滑降至 0)
            W = (dist - X_go) / (D0 - X_go);
            W = max(0, min(1, W)); 
            
            % b. 计算空间位置误差 (理想目标管道 - 当前导弹位置)
            err_y = (targetState.pos(2) + guidanceParams.y_offset) - missileState.pos(2);
            err_z = (targetState.pos(3) + guidanceParams.z_offset) - missileState.pos(3);
            
            % c. 计算速度误差 (用于提供阻尼，防止弹道震荡)
            err_vy = targetState.vel(2) - missileState.vel(2);
            err_vz = targetState.vel(3) - missileState.vel(3);
            
            % d. PD 控制器生成偏置加速度 (参数 Kp, Kd 经过工程估算)
            Kp = 0.5;  % 刚度系数 (越大找点越积极)
            Kd = 1.0;  % 阻尼系数 (防止修正过度)
            
            raw_bias_y = Kp * err_y + Kd * err_vy;
            raw_bias_z = Kp * err_z + Kd * err_vz;
            
            % 重力补偿 (因为我们想保持在 z_offset 相对高度)
            raw_bias_z = raw_bias_z + g;
            
            % e. 施加平滑权重，生成最终偏置向量
            bias_vec = [0, raw_bias_y * W, raw_bias_z * W];
        else
            % [末端段] < X_go，导引头已开机，偏置彻底归零，纯PN追击
            bias_vec = [0, 0, g]; % 仅保留基本的重力补偿
        end
        
    else
        % --- 兼容你之前的单弹模式 (纯纵向高抛) ---
        dist_climb_end = D0 * guidanceParams.climb_ratio;
        dist_dive_start = D0 * guidanceParams.dive_ratio;
        
        if dist > dist_climb_end
            bias_vec = [0, 0, 2 * g];
        elseif dist > dist_dive_start
            ratio = (dist - dist_dive_start) / (dist_climb_end - dist_dive_start);
            bias_vec = [0, 0, ratio * 2 * g];
        else
            bias_vec = [0, 0, g];
        end
    end

    % ================= 4. 合成最终指令 =================
    guidanceAccel = acc_pn + bias_vec;
    
    % ================= 5. 饱和限制 =================
    % 限制最大过载 (防止指令过大导致失速)
    if dist > 30000
        max_G = 15 * g; % 中段放宽一点给协同偏置留余量
    else
        max_G = 35 * g; % 末端全力机动
    end
    
    if norm(guidanceAccel) > max_G
        guidanceAccel = guidanceAccel / norm(guidanceAccel) * max_G;
    end
end