function [nextState, logData] = MissileModel(currentState, guidanceAccel, dt, environment, missileParams)
    % --- 0. 状态提取 ---
    pos = currentState.pos;
    vel = currentState.vel;
    mass = currentState.mass;
    time = currentState.time;
    
    speed = norm(vel);
    if speed > 1e-3, velDir = vel / speed; else, velDir = [1,0,0]; end
    Mach = speed / environment.sound_speed; % 计算马赫数
    
    status = 'TRACKING';
    
    % --- 1. 发动机逻辑 (双脉冲推力) ---

    % 0 < t < 6s:   30 kN (一级点火)
    % 6s < t < 21s: 0 kN  (滑行)
    % 21s < t < 26s: 15 kN (二级点火)
    % t > 26s:      0 kN  (燃尽)
    if time < missileParams.motor.stage1_time
        % 第一脉冲阶段
        thrust = missileParams.motor.stage1_thrust;
        fuelFlow = missileParams.fuelMass_1 / missileParams.motor.stage1_time; 
    elseif time >= missileParams.motor.coast_start && time < missileParams.motor.coast_end
        % 滑行阶段
        thrust = 0;
        fuelFlow = 0;
    elseif time >= missileParams.motor.stage2_start && time < missileParams.motor.stage2_end
        % 第二脉冲阶段
        thrust = missileParams.motor.stage2_thrust;
        fuelFlow = missileParams.fuelMass_2 / (missileParams.motor.stage2_end - missileParams.motor.stage2_start); 
    else
        thrust = 0;
        fuelFlow = 0;
    end
    
    % --- 2. 空气动力学  ---
    rho = environment.getDensity(pos(3)); % 获取当前高度的空气密度
    q = 0.5 * rho * speed^2;              % 动压 (Dynamic Pressure)
    
    % 基础阻力系数 (随马赫数变化)
    Cd0 = interp1(missileParams.aero.machPoints, missileParams.aero.cdValues, Mach, 'linear', 'extrap');
    % 升力公式: L = q * S * CL
    % 最大过载 = 最大升力 / 重力
    maxCL = 1.2; % 导弹最大升力系数
    if speed > 10 
        maxLift_phys = q * missileParams.refArea * maxCL; % 物理最大升力
        maxG_phys = maxLift_phys / (mass * environment.g); % 换算成 G 值
    else
        maxG_phys = 0;
    end
    
    % 最终限制：取 结构限制(35G) 和 气动限制(maxG_phys) 的最小值
    limitG = min(missileParams.aero.maxG, maxG_phys);
    
    % --- 3. 指令执行与饱和处理 ---
    accCmdMag = norm(guidanceAccel); 
    realAccCmd = guidanceAccel;
    
    if accCmdMag > limitG * environment.g
        if accCmdMag > 0
            realAccCmd = (guidanceAccel / accCmdMag) * (limitG * environment.g);
        end
        status = 'SATURATED'; 
    end
    
    % --- 4. 诱导阻力计算  ---
    if q > 1
        currentLift = mass * norm(realAccCmd);
        actCL = currentLift / (q * missileParams.refArea); % 反推当前的升力系数
    else
        actCL = 0;
    end
    
    k_ind = 0.15; % 诱导阻力因子
    Cd_total = Cd0 + k_ind * actCL^2; % 总阻力 = 零升阻力 + 诱导阻力
    
    dragForce = q * missileParams.refArea * Cd_total; % 最终阻力力值
    
    % --- 5. 运动积分 (F = ma) ---
    F_thrust = thrust * velDir;              % 推力
    F_drag   = -dragForce * velDir;          % 阻力 (永远反方向)
    F_grav   = [0, 0, -mass * environment.g];% 重力
    F_control = mass * realAccCmd;           % 气动控制力(升力/侧力)
    
    totalForce = F_thrust + F_drag + F_grav + F_control;
    accel = totalForce / mass;
    % 更新速度和位置 
    nextVel = vel + accel * dt;
    nextPos = pos + nextVel * dt;
    
    % 坠地检测
    if nextPos(3) < 0, nextPos(3) = 0; nextVel = [0,0,0]; status = 'CRASHED'; end
    
    % 质量更新 
    nextMass = max(missileParams.dryMass, mass - fuelFlow * dt);
    
    % --- 6. 输出打包 ---
    nextState.pos = nextPos;
    nextState.vel = nextVel;
    nextState.mass = nextMass;
    nextState.time = time + dt;
    nextState.status = status;

    if isfield(currentState, 'isSeekerActive')
        nextState.isSeekerActive = currentState.isSeekerActive;
    else
        nextState.isSeekerActive = false; % 默认值，防报错
    end
    
    logData.thrust = thrust;
    
    logData.thrust = thrust;
    logData.mach = Mach;
    logData.gLoad = norm(accel) / environment.g; 
    logData.limitG = limitG; 
    logData.status = status;
end