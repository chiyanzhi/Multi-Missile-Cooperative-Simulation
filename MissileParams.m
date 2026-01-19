function params = MissileParams(missileType)  
    switch upper(missileType)
        case 'MISSILE-A'% 特点：中距空空导弹，轻快，加速度大，但存速能力不如重型弹
            params.name = 'MISSILE-A';
            params.initialMass = 152;   % kg
            params.dryMass     = 90;    % kg
            params.refArea     = 0.025; % m^2 (直径 180mm)
            
            % 动力：典型的 Boost-Sustain 双推力
            params.motor.boostThrust   = 25000; % N
            params.motor.boostTime     = 3.0;   % s 
            params.motor.sustainThrust = 6000;  % N
            params.motor.sustainTime   = 6.0;   % s
            params.motor.totalBurnTime = 9.0;
            
            % 气动与机动
            params.aero.maxG = 40;
            params.aero.maxSpeed = 1360; % ~Mach 4
            % 阻力系数 (细长体，阻力较小)
            params.aero.machPoints = [0, 1, 4];
            params.aero.cdValues   = [0.2, 0.4, 0.25];
            
            % 战斗部
            params.warhead.killRadius = 12; % m 
            params.fuse.detonationRange = 15;
            
            % 导引头
            params.sensor.seekRange = 25000; % 25km 主动段
            
        case 'MISSILE-B' % 特点：中距空空导弹，体积受限，射程极远
            params.name = 'MISSILE-B';
            params.initialMass = 220;   % kg 
            params.dryMass     = 110;   % kg
            params.refArea     = 0.025; % m^2 

            % 动力
            params.motor.boostThrust   = 30000; % N
            params.motor.boostTime     = 4.0;   % s
            params.motor.sustainThrust = 8000;  % N
            params.motor.sustainTime   = 15.0;  % s 
            params.motor.totalBurnTime = 19.0;
            
            % 气动
            params.aero.maxG = 35; 
            params.aero.maxSpeed = 1700; % ~Mach 5
            params.aero.machPoints = [0, 1, 5];
            params.aero.cdValues   = [0.2, 0.35, 0.22]; 
            
            % 战斗部 
            params.warhead.killRadius = 15;
            params.fuse.detonationRange = 18;
            
            % 导引头 
            params.sensor.seekRange = 40000; 

        case 'MISSILE-C'% 特点：远距空空导弹，质量大
            params.name = 'MISSILE-C';
            params.initialMass = 860;   
            params.dryMass     = 380;
            params.refArea     = 0.09;  
            
            % 动力
            params.motor.boostThrust   = 85000;
            params.motor.boostTime     = 8.0;
            params.motor.sustainThrust = 18000;
            params.motor.sustainTime   = 15.0;
            params.motor.totalBurnTime = 23.0;
            
            % 气动
            params.aero.maxG = 30; 
            params.aero.maxSpeed = 1500;
            params.aero.machPoints = [0, 1, 4];
            params.aero.cdValues   = [0.28, 0.6, 0.3];
            
            % 战斗部 
            params.warhead.killRadius = 25;
            params.fuse.detonationRange = 30;
            
            params.sensor.seekRange = 80000; 

        otherwise
            error('未知的导弹型号: %s', missileType);
    end
    
    % --- 通用补充处理 (避免参数缺失) ---
    % 如果某些共性参数没写，可以在这里补默认值
    if ~isfield(params, 'guidance'), params.guidance.N = 4; end
    if ~isfield(params.warhead, 'pks')
        % 默认杀伤曲线
        params.warhead.ranges = [0, 2, params.warhead.killRadius, params.warhead.killRadius*1.5];
        params.warhead.pks    = [1.0, 0.9, 0.5, 0.0];
    end
end