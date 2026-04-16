function params = MissileParams(~)
   
    params.name = 'Missile';
    
    % --- 质量特性 ---
    params.initialMass = 216;        % 总质量 (kg)
    params.fuelMass_1  = 54;         % 一级装药量 (kg)
    params.fuelMass_2  = 30;         % 二级装药量 (kg)
    % 干质量 = 总质量 - 所有燃料
    params.dryMass     = params.initialMass - params.fuelMass_1 - params.fuelMass_2; 
    
    params.refArea     = 0.025;      % 面积 (m^2) -
    
    % --- 2. 动力系统 ---
    % 阶段 1: 0 < t < 6s
    params.motor.stage1_thrust   = 30000; % 30 kN
    params.motor.stage1_time     = 6.0;   % s
    
    % 阶段 2 (滑行): 6 < t < 21s
    params.motor.coast_start     = 6.0;
    params.motor.coast_end       = 21.0;
    
    % 阶段 3: 21 < t < 26s
    params.motor.stage2_thrust   = 15000; % 15 kN
    params.motor.stage2_start    = 21.0;
    params.motor.stage2_end      = 26.0;
    
    % --- 3. 气动特性 ---

    params.aero.maxG     = 35;   % 结构最大过载
    params.aero.maxSpeed = 1500; 
    params.aero.machPoints = [0.0,  0.8,  0.95, 1.05, 1.2,  1.5,  2.5,  4.0,  6.0];
    params.aero.cdValues   = [0.25, 0.25, 0.65, 0.78, 0.70, 0.55, 0.42, 0.35, 0.30];
    
    % --- 4. 传感器限制 ---
    params.sensor.seekRange = 27000; % 27 km
    params.sensor.maxFOV    = 60;    % +/- 60度 
    
    % --- 5. 战斗部与制导 ---
    params.warhead.killRadius = 12; 
   
end