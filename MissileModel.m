function [nextState, logData] = MissileModel(currentState, guidanceAccel, dt, environment, missileParams)
    % MissileModel_Core 通用导弹物理内核
    % 这里的逻辑适用于所有类型的导弹，只要 params 给得对
    
    % 1. 状态提取
    pos = currentState.pos;
    vel = currentState.vel;
    mass = currentState.mass;
    time = currentState.time;
    
    speed = norm(vel);
    if speed > 0, velDir = vel/speed; else, velDir = [1,0,0]; end
    Mach = speed / environment.sound_speed; % 假设 env 里有声速
    
    % 2. 导弹发动机逻辑 (支持 Boost-Sustain 两段式)
    thrust = 0;
    fuelFlow = 0;
    
    if time <= missileParams.motor.boostTime
        % 助推段
        thrust = missileParams.motor.boostThrust;
        % 假设助推段消耗 65% 的总燃料
        totalFuel = missileParams.initialMass - missileParams.dryMass;
        fuelFlow = (totalFuel * 0.65) / missileParams.motor.boostTime;
        
    elseif time <= missileParams.motor.totalBurnTime
        % 续航段
        thrust = missileParams.motor.sustainThrust;
        totalFuel = missileParams.initialMass - missileParams.dryMass;
        fuelFlow = (totalFuel * 0.35) / missileParams.motor.sustainTime;
        
    else
        % 惯性段
        thrust = 0;
        fuelFlow = 0;
    end
    
    % 3. 气动阻力
    rho = environment.getDensity(pos(3));
    % 获取 Cd
    Cd = interp1(missileParams.aero.machPoints, missileParams.aero.cdValues, ...
                 Mach, 'linear', 'extrap');
    dragMag = 0.5 * rho * speed^2 * missileParams.refArea * Cd;
    
    % 4. 动力学积分
    F_thrust = thrust * velDir;
    F_drag   = -dragMag * velDir;
    F_grav   = [0, 0, -mass * environment.g];
    
    % 5. 控制力与过载限制
    F_req = mass * guidanceAccel;
    maxF  = mass * missileParams.aero.maxG * environment.g;
    if norm(F_req) > maxF
        F_control = F_req / norm(F_req) * maxF;
    else
        F_control = F_req;
    end
    
    accel = (F_thrust + F_drag + F_grav + F_control) / mass;
    
    nextVel = vel + accel * dt;
    % 极速限制
    if norm(nextVel) > missileParams.aero.maxSpeed
        nextVel = nextVel / norm(nextVel) * missileParams.aero.maxSpeed;
    end
    nextPos = pos + nextVel * dt;
    
    % 6. 输出
    nextState.pos = nextPos;
    nextState.vel = nextVel;
    nextState.mass = max(missileParams.dryMass, mass - fuelFlow * dt);
    nextState.time = time + dt;
    
    logData.thrust = thrust;
    logData.mach = Mach;
    logData.gLoad = norm(accel) / environment.g;
end