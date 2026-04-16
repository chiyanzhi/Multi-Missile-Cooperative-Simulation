function params = TargetParams()
    params.name = 'Fifth-Gen Fighter';
    
    % --- 动力学极限 ---
    params.maxG_cruise = 5.0;  
    params.maxG_evade  = 9.0;  % 极限过载 9G
    
    % --- 速度特性 ---
    params.maxSpeed    = 680;  % 
    params.minSpeed    = 150;  
    
    % --- 物理模型参数 ---
    params.tau         = 0.5;  % 飞控延迟
    params.dragFactor  = 0.04; 
    params.thrust_max  = 0.6;  % 推重比
    
    % --- 新增：特定机动参数 ---
    params.snake_freq  = 0.15; % 蛇形机动频率 (Hz)
    params.snake_maxG  = 4.0;  % 蛇形机动最大过载 (6G)
    params.turn_radius = 3000;   % 盘旋半径 3000米 
    params.loop_radius = 3000;   % 筋斗半径 3000米
   
    
    % --- 传感器/对抗 ---
    params.RCS = @(angle) 0.01 + 2.0 * abs(sin(angle))^4; 
    params.RWR_range   = 10000; 
    params.reactionTime = 0.3;

    % --- 新增：Split-S 机动状态标记 ---
    params.splits_complete = false;  % 初始化：未完成 Split-S 机动

end