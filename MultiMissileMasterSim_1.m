% =========================================================================
% 多弹协同主控仿真 (Multi-Missile Master Simulation)
% 包含：双约束最优占位规划 + 中制导协同偏置 + 末制导交接 + 目标智能机动
% =========================================================================
clear; clc; close all;

fprintf('🚀 启动多弹协同全流程仿真...\n');

%% ================= 1. 环境与参数初始化 =================
env = Environment();
tParams = TargetParams();
mParams = MissileParams();

% 加载之前跑出来的目标逃逸概率密度数据 (用于双约束规划)
if ~exist('Target_PDF_Data.mat', 'file')
    error('❌ 找不到 Target_PDF_Data.mat，请先运行 quanjidongdaba.m 生成数据！');
end
load('Target_PDF_Data.mat', 'Y', 'Z', 'P_density_opt', 'grid_step', 'max_y_disp', 'max_z_disp', 'X_go');

dt = 0.02;          % 仿真步长 (s)
t_max = 150;        % 最大仿真时间 (s)
N_missiles = 3;     % 协同导弹数量 (可修改)

%% ================= 2. 阶段一：协同占位规划 (Pre-launch Planning) =================
fprintf('🎯 正在调用 DualConstraint 进行 %d 枚导弹的阵位规划...\n', N_missiles);

% 构造规划所需的参数结构体
plan_t_params = struct('a', max_y_disp, 'b', max_z_disp); 
plan_m_params.maxFOV = repmat(8.0, 1, N_missiles); % 假设都是 8度 FOV
plan_m_params.a = repmat(9000, 1, N_missiles);     % 假设运动学长轴 9000m
plan_m_params.b = repmat(8000, 1, N_missiles);     % 假设运动学短轴 8000m
plan_m_params.Pmax = 0.95; 
plan_m_params.alpha = 12.0; 
weight_params = struct('w1', 1.0, 'w2', 0.1);

% 调用你写的双约束引擎获取最优偏置点 [y1, z1, y2, z2, ...]
[optimal_Z_bias, ~, ~, ~, ~] = DualConstraint(...
    N_missiles, plan_t_params, plan_m_params, weight_params, X_go, Y, Z, P_density_opt, grid_step);

fprintf('✅ 阵位规划完成！\n');

%% ================= 3. 初始状态设置 (Initial States) =================
% 目标初始状态 (在 100km 外，高度 10000m，迎头飞来)
initial_distance = 100000;
targetState.pos = [initial_distance, 0, 10000];
targetState.vel = [-tParams.maxSpeed, 0, 0]; 
targetState.time = 0;

% 导弹初始状态 (在原点附近，高度 10000m，有微小横向散布避免初始碰撞)
missileStates = cell(1, N_missiles);
guidanceParams = cell(1, N_missiles);
for i = 1:N_missiles
    missileStates{i}.pos = [0, (i - (N_missiles+1)/2) * 50, 10000]; % 初始间隔 50m
    missileStates{i}.vel = [1000, 0, 0]; % 初始速度 ~3马赫
    missileStates{i}.mass = mParams.initialMass;
    missileStates{i}.time = 0;
    missileStates{i}.status = 'TRACKING';
    missileStates{i}.isSeekerActive = false;
    
    % 配置专属导引参数 (注入刚刚算出来的最优偏置！)
    guidanceParams{i}.initial_dist = initial_distance;
    guidanceParams{i}.handover_dist = X_go; % 27000m
    guidanceParams{i}.N = 4;
    guidanceParams{i}.y_offset = optimal_Z_bias(2*i - 1); % 横向占位
    guidanceParams{i}.z_offset = optimal_Z_bias(2*i);     % 纵向占位
end

%% ================= 4. 阶段二 & 三：动态飞行主循环 (Dynamic Sim Loop) =================
fprintf('✈️ 导弹发射，开始中末制导飞行...\n');

% 数据记录预分配
time_log = [];
target_pos_log = [];
missile_pos_log = cell(1, N_missiles);
missile_dist_log = cell(1, N_missiles);

sim_running = true;
current_time = 0;

while sim_running && current_time < t_max
    % --- 记录当前步数据 ---
    time_log = [time_log; current_time];
    target_pos_log = [target_pos_log; targetState.pos];
    for i = 1:N_missiles
        missile_pos_log{i} = [missile_pos_log{i}; missileStates{i}.pos];
        missile_dist_log{i} = [missile_dist_log{i}; norm(targetState.pos - missileStates{i}.pos)];
    end
    
    % --- 4.1 目标机动更新 (阶段四) ---
    % 取距离最近的导弹作为威胁评估基准
    min_dist = min(cellfun(@(x) norm(targetState.pos - x.pos), missileStates));
    
    % 调用你的目标智能规避策略 ('auto' 自动根据威胁等级机动)
    escapeAccCmd = TargetEscapeStrategy(targetState, missileStates{1}, tParams, 'auto', env);
    [targetState, ~] = TargetModel(targetState, escapeAccCmd, dt, tParams, env);
    
    % --- 4.2 导弹群体更新 ---
    all_missed = true;
    for i = 1:N_missiles
        if strcmp(missileStates{i}.status, 'CRASHED') || strcmp(missileStates{i}.status, 'MISSED')
            continue; 
        end
        all_missed = false;
        
        dist_to_target = norm(targetState.pos - missileStates{i}.pos);
        
        % 判断是否到达交接班距离，开启导引头
        if dist_to_target <= guidanceParams{i}.handover_dist
            missileStates{i}.isSeekerActive = true;
        end
        
        % 命中判定
        if dist_to_target < mParams.warhead.killRadius
            fprintf('💥 导弹 %d 成功击中目标！拦截耗时: %.2fs\n', i, current_time);
            sim_running = false;
            break;
        end
        % 脱靶判定 (飞过头了)
        if (targetState.pos(1) - missileStates{i}.pos(1)) < -1000 
             missileStates{i}.status = 'MISSED';
        end
        
        % 计算导引指令并更新动力学
        guidanceAccel = GuidanceLaw(missileStates{i}, targetState, guidanceParams{i}, env);
        [missileStates{i}, ~] = MissileModel(missileStates{i}, guidanceAccel, dt, env, mParams);
    end
    
    if all_missed
        fprintf('❌ 所有导弹均脱靶，拦截失败。\n');
        break;
    end
    
    current_time = current_time + dt;
end

%% ================= 5. 可视化：3D 飞行轨迹 =================
fprintf('📊 正在生成 3D 飞行轨迹图...\n');

fig = figure('Name', 'Multi-Missile 3D Trajectory', 'Color', 'w', 'Position', [100, 100, 1000, 700]);
hold on; grid on; view(30, 20);

% 绘制目标轨迹
plot3(target_pos_log(:,1), target_pos_log(:,2), target_pos_log(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Target');
plot3(targetState.pos(1), targetState.pos(2), targetState.pos(3), 'k*', 'MarkerSize', 8, 'DisplayName', 'Impact Point');

% 绘制导弹轨迹
colors = lines(N_missiles);
for i = 1:N_missiles
    m_pos = missile_pos_log{i};
    plot3(m_pos(:,1), m_pos(:,2), m_pos(:,3), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Missile %d', i));
    
    % 标出交接班开机点
    idx_handover = find(missile_dist_log{i} <= X_go, 1);
    if ~isempty(idx_handover)
        plot3(m_pos(idx_handover,1), m_pos(idx_handover,2), m_pos(idx_handover,3), 'o', ...
            'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'w', 'MarkerSize', 6, 'HandleVisibility', 'off');
    end
end

% 绘制交接班虚拟平面 (X_go 面)
% 找到目标在交接班时刻的大致 X 坐标
idx_handover_target = find(min(cell2mat(missile_dist_log), [], 2) <= X_go, 1);
if ~isempty(idx_handover_target)
    X_plane = target_pos_log(idx_handover_target, 1) - X_go; % 导弹当时的X坐标
    [Yp, Zp] = meshgrid(-15000:5000:15000, 0:5000:20000);
    Xp = ones(size(Yp)) * X_plane;
    surf(Xp, Yp, Zp, 'FaceColor', 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Handover Plane (27km)');
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude Z (m)');
title('Multi-Missile Cooperative Interception with Trajectory Shaping');
legend('Location', 'best');
axis equal; 
% 调整视角让轨迹更清晰
xlim([0, initial_distance]);
ylim([-20000, 20000]);
zlim([0, 20000]);

fprintf('🎉 仿真全流程执行完毕！\n');