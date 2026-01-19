function params = TargetParams()
    % TargetParams 定义目标飞机的性能参数
    
    params.name = 'Target';
    
    % 1. 机动能力限制
    params.maxG_cruise = 2.0;    % 巡航过载
    params.maxG_evade  = 9.0;    % 逃逸最大过载 (9G)
    params.maxSpeed    = 600;    % m/s (约 Mach 1.8)
    params.minSpeed    = 200;    % m/s (防止失速)
    
    % 2. 态势感知 
    params.RWR_range   = 80000;  % [m] 80km外发现导弹并报警
    params.Panic_range = 25000;  % [m] 25km内进行规避
    
    % 3. 策略参数
    params.reactionTime = 0.5;   % [s] 飞行员反应延迟
    params.maneuverChangeInterval = 5.0; % [s] 每隔几秒切换一次随机战术
end