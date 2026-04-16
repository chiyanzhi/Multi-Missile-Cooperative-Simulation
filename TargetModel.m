function [nextState, logData] = TargetModel(currentState, escapeAccCmd, dt, params, environment)
    % --- 初始化 ---
    pos = currentState.pos;
    vel = currentState.vel;
    if isfield(currentState, 'acc')
        acc_prev = currentState.acc;
    else
        acc_prev = [0, 0, 0];
    end

    g = environment.g;
    speed = norm(vel);
    % 速度方向单位向量（保护除零）
    if speed > 1e-3
        vel_unit = vel / speed;
    else
        vel_unit = [1, 0, 0]; % 速度为0时默认方向
    end

    % --- 1. 策略指令输入（外部传入，不再内部计算） ---
    accCmd = escapeAccCmd;

    % --- 2. 物理动力学限制 ---
    % A. 指令限幅（区分巡航/逃逸过载）
    accMag = norm(accCmd);
    maxAcc = params.maxG_evade * g; % 固定用逃逸过载（9G）
    if accMag > maxAcc
        accCmd = accCmd / accMag * maxAcc;
    end

    % B. 一阶惯性环节（飞控延迟）
    real_acc = acc_prev + (accCmd - acc_prev) * (dt / params.tau);

    % C. 能量守恒与诱导阻力（引入环境模型）
    current_G = norm(real_acc) / g;
    thrust_g = params.thrust_max; 
    if speed > params.maxSpeed, thrust_g = 0; end 
    
    % --- [修改开始] 引入大气密度影响 ---
    % 获取当前高度的大气密度
    rho = environment.getDensity(pos(3));
    rho0 = 1.225; % 海平面标准密度
    density_ratio = rho / rho0;
    
    % 阻力计算优化：
    % 原公式隐含假设海平面密度。现在乘以密度比，模拟高空阻力减小。
    % 阻力 F_drag = 0.5 * rho * v^2 * Cd * S
    % 这里 drag_g 是过载形式，正比于 rho * v^2
    base_drag = (0.05 + params.dragFactor * current_G^2);
    
    % 动压项：(speed/340)^2 代表马赫数平方项，需修正密度影响
    drag_g = base_drag * (speed/340)^2 * density_ratio; 
 
    vel_dir = vel / (speed + 1e-6);
    acc_tangential = (thrust_g * g - drag_g * g) * vel_dir;

    % 总加速度
    final_acc = real_acc + acc_tangential - [0,0,g]; 

    % --- 3. 积分更新 ---
    nextVel = vel + final_acc * dt;
    % 强制最小速度限制（防止失速）
    nextSpeed = norm(nextVel);
    if nextSpeed < params.minSpeed
        nextVel = nextVel / nextSpeed * params.minSpeed;
    end
    nextPos = pos + nextVel * dt;
    if nextPos(3) < 10, nextPos(3) = 10; nextVel(3) = 0; end % 最低高度限制

    % --- 4. 输出 ---
    nextState.pos = nextPos;
    nextState.vel = nextVel;
    nextState.acc = real_acc;
    nextState.time = currentState.time + dt;

    logData.acc = real_acc;
    %logData.mode = currentMode;
    logData.speed = norm(nextVel);
end