% =========================================================================
% 多弹协同主控仿真 (Multi-Missile Master Simulation)
% 包含：双约束最优占位规划 + 中制导协同偏置 + 末制导交接 + 目标智能机动 + FOV限制
% =========================================================================
clear; clc; close all;

fprintf('🚀 启动多弹协同全流程仿真...\n');

%% ================= 1. 环境与参数初始化 =================
env = Environment();
tParams = TargetParams();
mParams = MissileParams();

if ~exist('Target_PDF_Data.mat', 'file')
    error('❌ 找不到 Target_PDF_Data.mat，请先运行 quanjidongdaba.m 生成数据！');
end
load('Target_PDF_Data.mat', 'Y', 'Z', 'P_density_opt', 'grid_step', 'max_y_disp', 'max_z_disp', 'X_go');

dt = 0.02;          
t_max = 150;        
N_missiles = 3;     

%% ================= 2. 阶段一：协同占位规划 (Pre-launch Planning) =================
fprintf('🎯 正在调用 DualConstraint 进行 %d 枚导弹的阵位规划...\n', N_missiles);

plan_t_params = struct('a', max_y_disp, 'b', max_z_disp); 
plan_m_params.maxFOV = repmat(8.0, 1, N_missiles); 
plan_m_params.a = repmat(9000, 1, N_missiles);     
plan_m_params.b = repmat(8000, 1, N_missiles);     
plan_m_params.Pmax = 0.95; 
plan_m_params.alpha = 12.0; 
weight_params = struct('w1', 1.0, 'w2', 0.1);

[optimal_Z_bias, ~, ~, ~, ~] = DualConstraint(...
    N_missiles, plan_t_params, plan_m_params, weight_params, X_go, Y, Z, P_density_opt, grid_step);

fprintf('✅ 阵位规划完成！\n');

%% ================= 3. 初始状态设置 (Initial States) =================
initial_distance = 100000;
targetState.pos = [initial_distance, 0, 10000];
targetState.vel = [-tParams.maxSpeed, 0, 0]; 
targetState.time = 0;

missileStates = cell(1, N_missiles);
guidanceParams = cell(1, N_missiles);
for i = 1:N_missiles
    missileStates{i}.pos = [0, (i - (N_missiles+1)/2) * 50, 10000]; 
    missileStates{i}.vel = [1000, 0, 0]; 
    missileStates{i}.mass = mParams.initialMass;
    missileStates{i}.time = 0;
    missileStates{i}.status = 'TRACKING';
    missileStates{i}.isSeekerActive = false;
    
    guidanceParams{i}.initial_dist = initial_distance;
    guidanceParams{i}.handover_dist = X_go; 
    guidanceParams{i}.N = 4;
    guidanceParams{i}.y_offset = optimal_Z_bias(2*i - 1); 
    guidanceParams{i}.z_offset = optimal_Z_bias(2*i);     
end

%% ================= 4. 阶段二 & 三：动态飞行主循环 (Dynamic Sim Loop) =================
fprintf('✈️ 导弹发射，开始中末制导飞行...\n');

time_log = [];
target_pos_log = [];
missile_pos_log = cell(1, N_missiles);
missile_dist_log = cell(1, N_missiles);

sim_running = true;
current_time = 0;

while sim_running && current_time < t_max
    time_log = [time_log; current_time];
    target_pos_log = [target_pos_log; targetState.pos];
    for i = 1:N_missiles
        missile_pos_log{i} = [missile_pos_log{i}; missileStates{i}.pos];
        missile_dist_log{i} = [missile_dist_log{i}; norm(targetState.pos - missileStates{i}.pos)];
    end
    
    % --- 4.1 目标机动更新 ---
    escapeAccCmd = TargetEscapeStrategy(targetState, missileStates{1}, tParams, 'auto', env);
    [targetState, ~] = TargetModel(targetState, escapeAccCmd, dt, tParams, env);
    
    % --- 4.2 导弹群体更新 ---
    all_missed = true;
    for i = 1:N_missiles
        if strcmp(missileStates{i}.status, 'CRASHED') || strcmp(missileStates{i}.status, 'MISSED') || strcmp(missileStates{i}.status, 'LOST_FOV')
            continue; 
        end
        all_missed = false;
        
        dist_to_target = norm(targetState.pos - missileStates{i}.pos);
        
        % 判断是否到达交接班距离，开启导引头，并进行FOV检测
        if dist_to_target <= guidanceParams{i}.handover_dist
            missileStates{i}.isSeekerActive = true;
            
            % 🚨 新增：计算视线向量与导弹速度向量的夹角 (FOV检测)
            los_vec = targetState.pos - missileStates{i}.pos;
            los_vec = los_vec / (norm(los_vec) + 1e-6);
            v_m_dir = missileStates{i}.vel / (norm(missileStates{i}.vel) + 1e-6);
            
            look_angle_rad = acos(max(-1, min(1, dot(los_vec, v_m_dir))));
            look_angle_deg = rad2deg(look_angle_rad);
            
            if look_angle_deg > mParams.sensor.maxFOV
                missileStates{i}.status = 'LOST_FOV'; 
                fprintf('⚠️ 导弹 %d 导引头丢失目标！偏角 %.1f° > 视场角 %.1f°\n', ...
                        i, look_angle_deg, mParams.sensor.maxFOV);
                continue; % 直接跳过本弹的后续计算
            end
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
        fprintf('❌ 所有导弹均已失效 (脱靶或丢失视角)，拦截失败。\n');
        break;
    end
    
    current_time = current_time + dt;
end

%% ================= 5. 可视化：3D 飞行轨迹 =================
fprintf('📊 正在生成 3D 飞行轨迹图...\n');
fig = figure('Name', 'Multi-Missile 3D Trajectory', 'Color', 'w', 'Position', [100, 100, 1000, 700]);
hold on; grid on; view(30, 20);

plot3(target_pos_log(:,1), target_pos_log(:,2), target_pos_log(:,3), 'k-', 'LineWidth', 2, 'DisplayName', 'Target');
plot3(targetState.pos(1), targetState.pos(2), targetState.pos(3), 'k*', 'MarkerSize', 8, 'DisplayName', 'Impact Point');

colors = lines(N_missiles);
for i = 1:N_missiles
    m_pos = missile_pos_log{i};
    plot3(m_pos(:,1), m_pos(:,2), m_pos(:,3), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Missile %d', i));
    
    idx_handover = find(missile_dist_log{i} <= X_go, 1);
    if ~isempty(idx_handover)
        plot3(m_pos(idx_handover,1), m_pos(idx_handover,2), m_pos(idx_handover,3), 'o', ...
            'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'w', 'MarkerSize', 6, 'HandleVisibility', 'off');
    end
end

idx_handover_target = find(min(cell2mat(missile_dist_log), [], 2) <= X_go, 1);
if ~isempty(idx_handover_target)
    X_plane = target_pos_log(idx_handover_target, 1) - X_go; 
    [Yp, Zp] = meshgrid(-15000:5000:15000, 0:5000:20000);
    Xp = ones(size(Yp)) * X_plane;
    surf(Xp, Yp, Zp, 'FaceColor', 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Handover Plane (27km)');
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude Z (m)');
title('Multi-Missile Cooperative Interception with Trajectory Shaping');
legend('Location', 'best');
axis equal; 
xlim([0, initial_distance]); ylim([-20000, 20000]); zlim([0, 20000]);
fprintf('🎉 仿真全流程执行完毕！\n');