function [nextState, logData] = TargetModel(currentState, threatState, dt, params)
    % TargetModel 目标智能体 (含博弈逻辑)
    % 输入:
    %   currentState: 目标当前状态 (pos, vel, time, maneuverType...)
    %   threatState:  最具威胁的导弹状态 (pos, vel) - 相当于RWR数据
    %   params:       TargetParams 返回的结构体
    
    % 提取状态
    pos = currentState.pos;
    vel = currentState.vel;
    time = currentState.time;
    
    % 1. 态势感知 (Situation Awareness)
    % 计算与威胁源的相对关系
    R_vec = threatState.pos - pos;
    dist = norm(R_vec);
    
    % 默认加速度
    accCmd = [0, 0, 0];
    currentMode = 'CRUISE';
    
    % 2. 博弈决策逻辑 (Game Logic)
    
    if dist > params.RWR_range
        % --- 阶段 A: 巡航 ---
        currentMode = 'PATROL';
        accCmd = [0, 0, 0]; 
        
    elseif dist > params.Panic_range
        % --- 阶段 B: 战术规避 ---
        currentMode = 'TACTICAL';
        
        % 策略：做蛇形机动消耗导弹能量 (Energy Bleeding)
        % 垂直于导弹来袭方向做正弦运动
        threatDir = R_vec / dist;
        velDir = vel / norm(vel);
        
        % 计算侧向向量 (水平面的左/右)
        sideDir = cross(threatDir, [0,0,1]); 
        if norm(sideDir) < 0.1, sideDir = [1,0,0]; end
        sideDir = sideDir / norm(sideDir);
        
        % 5G 的蛇形机动
        g_force = 5 * 9.81 * sin(0.5 * time); 
        accCmd = sideDir * g_force;
        
    else
        % --- 阶段 C: 终极规避 ---
        % 引入随机性
        
        % 利用 time 简单模拟随机切换 (每5秒换一招)
        seed = floor(time / params.maneuverChangeInterval);
        strategy = mod(seed, 3); % 0, 1, 2 三种随机策略
        
        threatDir = R_vec / dist;
        v_mag = norm(vel);
        v_dir = vel / v_mag;
        
        switch strategy
            case 0 % 策略: 3-9线机动 
                % 试图与导弹成90度夹角，利用多普勒盲区，同时最大化导弹过载
                currentMode = '3-9线机动';
                
                % 目标航向：垂直于导弹连线
                desiredDir = cross(threatDir, [0,0,1]); 
                desiredDir = desiredDir / norm(desiredDir);
                errorDir = desiredDir - v_dir;
                accCmd = errorDir * params.maxG_evade * 9.81;
                
            case 1 % 策略: 桶滚/螺旋下坠
                currentMode = '桶滚/螺旋下坠';                
                accCmd = [0, 0, -params.maxG_evade * 9.81];
                
            case 2 % 策略: 迎头对冲 
                currentMode = '迎头对冲';
                accCmd = (threatDir - v_dir) * 3 * 9.81;
        end
    end
    
   
    % 限制最大过载
    accMag = norm(accCmd);
    maxAcc = params.maxG_evade * 9.81;
    if accMag > maxAcc
        accCmd = accCmd / accMag * maxAcc;
    end
    
    % 动力学积分
    nextVel = vel + accCmd * dt;
    
    % 速度限制 
    spd = norm(nextVel);
    if spd > params.maxSpeed
        nextVel = nextVel / spd * params.maxSpeed;
    elseif spd < params.minSpeed
        nextVel = nextVel / spd * params.minSpeed;
    end
    
    nextPos = pos + nextVel * dt;
    
    % 防止钻地
    if nextPos(3) < 50
        nextPos(3) = 50; 
        nextVel(3) = 0; 
    end
    
    % 5. 输出状态
    nextState.pos = nextPos;
    nextState.vel = nextVel;
    nextState.time = time + dt;
    nextState.mode = currentMode; % 记录当前在干嘛
    
    logData.acc = accCmd;
    logData.mode = currentMode;
end