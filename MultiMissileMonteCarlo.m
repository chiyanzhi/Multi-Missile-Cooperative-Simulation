% =========================================================================
% 蒙特卡洛多弹协同打靶仿真 (Monte Carlo Multi-Missile Simulation)
% 目的：统计多弹协同双约束 TSG 制导下的真实杀伤概率 (Kill Probability)
% =========================================================================
clear; clc; close all;

fprintf('🚀 启动多弹协同蒙特卡洛打靶仿真...\n');

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
N_missiles = 3;     % 协同导弹数量
N_runs = 1000;       % 蒙特卡洛打靶总次数 (可根据电脑性能调整，比如 500)

%% ================= 2. 阶段一：协同占位规划 (仅需执行一次) =================
fprintf('🎯 正在调用 DualConstraint 获取 %d 枚导弹的阵位基准...\n', N_missiles);
plan_t_params = struct('a', max_y_disp, 'b', max_z_disp); 
plan_m_params.maxFOV = repmat(8.0, 1, N_missiles); 
plan_m_params.a = repmat(9000, 1, N_missiles);     
plan_m_params.b = repmat(8000, 1, N_missiles);     
plan_m_params.Pmax = 0.95; plan_m_params.alpha = 12.0; 
weight_params = struct('w1', 1.0, 'w2', 0.1);

[optimal_Z_bias, ~, ~, ~, ~] = DualConstraint(...
    N_missiles, plan_t_params, plan_m_params, weight_params, X_go, Y, Z, P_density_opt, grid_step);
fprintf('✅ 阵位规划完成！准备开始 %d 次打靶...\n', N_runs);

%% ================= 3. 蒙特卡洛循环 =================
% 统计变量
success_count = 0;
miss_count = 0;
intercept_points = []; % 记录命中时的三维坐标
intercept_times = [];  % 记录拦截耗时

initial_distance = 100000;

% 进度条
wb = waitbar(0, '正在进行蒙特卡洛打靶，请稍候...');

tic; % 开始计时
for run_idx = 1:N_runs
    
    % 更新进度条
    waitbar(run_idx / N_runs, wb, sprintf('打靶进度: %d / %d', run_idx, N_runs));
    
    % --- 3.1 状态重置 ---
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

    % --- 3.2 单次动态仿真内循环 ---
    sim_running = true;
    current_time = 0;
    is_hit = false;
    hit_pos = [0, 0, 0];
    
    while sim_running && current_time < t_max
        % 目标智能规避 ('auto' 模式自带随机化机动，每次打靶目标机动可能不同)
        escapeAccCmd = TargetEscapeStrategy(targetState, missileStates{1}, tParams, 'auto', env);
        [targetState, ~] = TargetModel(targetState, escapeAccCmd, dt, tParams, env);
        
        all_missed_or_crashed = true;
        
        for i = 1:N_missiles
            if strcmp(missileStates{i}.status, 'CRASHED') || strcmp(missileStates{i}.status, 'MISSED')
                continue; 
            end
            all_missed_or_crashed = false;
            
            dist_to_target = norm(targetState.pos - missileStates{i}.pos);
            
            if dist_to_target <= guidanceParams{i}.handover_dist
                missileStates{i}.isSeekerActive = true;
            end
            
            % 命中判定
            if dist_to_target < mParams.warhead.killRadius
                is_hit = true;
                hit_pos = targetState.pos;
                sim_running = false;
                break;
            end
            
            % 脱靶判定
            if (targetState.pos(1) - missileStates{i}.pos(1)) < -1000 
                 missileStates{i}.status = 'MISSED';
            end
            
            % 导引与动力学
            guidanceAccel = GuidanceLaw(missileStates{i}, targetState, guidanceParams{i}, env);
            [missileStates{i}, ~] = MissileModel(missileStates{i}, guidanceAccel, dt, env, mParams);
        end
        
        if all_missed_or_crashed
            sim_running = false;
        end
        current_time = current_time + dt;
    end
    
    % --- 3.3 结果记录 ---
    if is_hit
        success_count = success_count + 1;
        intercept_points = [intercept_points; hit_pos];
        intercept_times = [intercept_times; current_time];
    else
        miss_count = miss_count + 1;
    end
end
close(wb);
total_time = toc;

%% ================= 4. 统计与可视化 =================
kill_prob = (success_count / N_runs) * 100;
fprintf('\n📊 蒙特卡洛打靶结束！总耗时: %.2f 秒\n', total_time);
fprintf('👉 总打靶次数: %d\n', N_runs);
fprintf('✅ 成功拦截: %d 次\n', success_count);
fprintf('❌ 目标逃脱: %d 次\n', miss_count);
fprintf('🏆 最终系统杀伤概率 (Kill Probability): %.2f%%\n', kill_prob);

% 绘制结果图表
figure('Name', 'Monte Carlo Results', 'Color', 'w', 'Position', [150, 150, 1000, 450]);

% 子图 1：命中率饼图
subplot(1, 2, 1);
pie([success_count, miss_count], {'Successful Interception', 'Target Escaped'});
colormap([0.2 0.8 0.2; 0.8 0.2 0.2]); % 绿代表成功，红代表逃跑
title(sprintf('System Kill Probability: %.1f%%', kill_prob), 'FontSize', 14);

% 子图 2：拦截点三维散布图
subplot(1, 2, 2);
if ~isempty(intercept_points)
    scatter3(intercept_points(:,1), intercept_points(:,2), intercept_points(:,3), 40, 'b', 'filled', 'MarkerEdgeColor', 'k');
    grid on; view(30, 20);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Altitude (m)');
    title('Spatial Distribution of Intercept Points', 'FontSize', 14);
else
    title('No successful interceptions to display.');
end

sgtitle(sprintf('Monte Carlo Simulation Results (N = %d)', N_runs), 'FontSize', 16, 'FontWeight', 'bold');