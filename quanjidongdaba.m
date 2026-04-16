clear; clc; close all;

%% ================= 1. Environment & Parameter Setup (保守最差工况) =================
X_go_handover = 27000;       % 交班距离 (m)
handover_altitude = 10000;  % 拦截高度 (m)

env.g = 9.81;
env.sound_speed = 340;
rho0 = 1.225; H_scale = 8500; 
rho = rho0 * exp(-handover_altitude / H_scale); 

% 目标机动参数：使用最大逃逸速度 2.5 Ma 计算机动潜力
env.V_target_max_escape = 2.5 * env.sound_speed; 

% 导弹物理参数 (来自你的 MissileParams.m)
m_missile = 132;  % 干质量 (kg)
S_ref = 0.025;    
V_initial = 4 * env.sound_speed; 

% --- 1.2 保守预估 t_go (目标最大机动时长) ---
% 假设目标在轴向上不动，导弹仅受阻力掉速，从而得出最长的逃逸窗口
dt_est = 0.01; curr_X = 0; curr_V = V_initial; curr_t = 0;
while curr_X < X_go_handover
    Mach = curr_V / env.sound_speed;
    % 阻力系数插值近似
    Cd0 = (Mach > 1.2) * 0.4 + (Mach <= 1.2) * 0.3; 
    q = 0.5 * rho * curr_V^2;
    D = q * S_ref * Cd0; 
    ax = -D / m_missile; 
    curr_V = curr_V + ax * dt_est;
    curr_X = curr_X + curr_V * dt_est; % 仅累加导弹位移
    curr_t = curr_t + dt_est;
    if curr_V < 250, break; end 
end
t_go = curr_t; 

fprintf('--- 保守最差工况 PDF 动力学生成 ---\n');
fprintf('预估目标最大机动时间 t_go: %.2f s\n', t_go);

N_total = 100000;           % 总打靶次数
all_endpoints = zeros(N_total, 2);
base_params = get_base_params(); % 调用你提供的参数设置
is_maneuver_triggered = (t_go <= 6);

tic;

%% ================= 2. Markov Chain State Space (已剔除平飞) =================
% 模式库：剔除 levelflight，保留 8 种高机动模式
normal_modes = {'steadyturn', 'verticalloop', 'snake', 'barrelroll', ...
                'spiralclimb', 'splits', 'immelmann', 'tailchase'};
N_modes = length(normal_modes);

% 调整后的 8x8 转移矩阵 (强化状态保持，确保机动不被抵消)
P_normal = eye(N_modes) * 0.7 + ones(N_modes) * (0.3/N_modes);
P_normal = P_normal ./ sum(P_normal, 2);

triggered_modes = {'splits', 'immelmann', 'barrelroll', 'verticalloop'};
P_triggered = eye(4) * 0.6 + ones(4) * 0.1;
P_triggered = P_triggered ./ sum(P_triggered, 2);

%% ================= 3. Monte Carlo Kinematic Integration Loop =================
for i = 1:N_total
    curr_t_int = 0; vy = 0; vz = 0; sy = 0; sz = 0;
    phi_plane = rand() * 2 * pi; % 空间全向随机机动平面
    
    % 使用你提供的随机参数生成函数
    rand_params = generate_random_target_params(base_params);
    
    current_normal_idx = randi(N_modes);
    current_trigger_idx = randi(length(triggered_modes));
    
    while curr_t_int < t_go
        % 马尔可夫状态切换
        if is_maneuver_triggered
            current_trigger_idx = find(cumsum(P_triggered(current_trigger_idx, :)) >= rand(), 1);
            mode = triggered_modes{current_trigger_idx};
        else
            current_normal_idx = find(cumsum(P_normal(current_normal_idx, :)) >= rand(), 1);
            mode = normal_modes{current_normal_idx};
        end
        
        % 机动参数受能量衰减影响
        energy_decay = exp(-curr_t_int / 2.5); 
        %energy_decay = 1.0; % 强制过载不随时间衰减
        current_maxG = rand_params.maxG_evade * energy_decay;
        

        
        dt_mode = min(1.0 + rand()*1.5, t_go - curr_t_int);
        step = 0.05;
        
        for t_sub = 0:step:dt_mode
            switch mode
                case 'steadyturn'
                    req_acc = (env.V_target_max_escape^2) / rand_params.turn_radius;
                    ay = min(req_acc, current_maxG * env.g); az = 0;
                case 'verticalloop'
                    req_acc = (env.V_target_max_escape^2) / rand_params.loop_radius;
                    ay = 0; az = min(req_acc, current_maxG * env.g);
                case 'snake'
                    ay = rand_params.snake_maxG * env.g * sin(2*pi*rand_params.snake_freq*curr_t_int); az = 0;
                case 'barrelroll'
                    % 滚筒：绕轴心旋转的向心力
                    ay = 0.7 * current_maxG * env.g * cos(2*pi*0.2*curr_t_int);
                    az = 0.7 * current_maxG * env.g * sin(2*pi*0.2*curr_t_int);
                case 'splits'
                    ay = 0; az = -current_maxG * env.g;
                case 'immelmann'
                    ay = 0; az = current_maxG * env.g;
                otherwise
                    ay = current_maxG * env.g; az = 0;
            end
            
            vy = vy + ay * step; vz = vz + az * step;
            sy = sy + vy * step; sz = sz + vz * step;
            curr_t_int = curr_t_int + step;
        end
    end
    % 旋转并存入最终碰撞点
    all_endpoints(i, :) = [sy*cos(phi_plane)-sz*sin(phi_plane), sy*sin(phi_plane)+sz*cos(phi_plane)];
end

fprintf('蒙特卡洛积分完成，耗时: %.2f 秒.\n', toc);

%% ================= 4. KDE Processing & Data Saving =================
actual_max_val = max(abs(all_endpoints(:)));
max_disp = actual_max_val;
grid_step = max_disp / 100; 
[Y, Z] = meshgrid(-max_disp:grid_step:max_disp, -max_disp:grid_step:max_disp);
Y_edges = -max_disp-grid_step/2 : grid_step : max_disp+grid_step/2;
Z_edges = -max_disp-grid_step/2 : grid_step : max_disp+grid_step/2;

counts = histcounts2(all_endpoints(:,1), all_endpoints(:,2), Y_edges, Z_edges);
% 考虑到打靶密度，使用 4.0 左右的 sigma 保证图像平滑
P_density_opt = imgaussfilt(counts, 4.0); 
P_density_opt = (P_density_opt / (sum(P_density_opt(:)) * grid_step^2))'; 

% 兼容保存
X_go = X_go_handover;
max_y_disp = max_disp; max_z_disp = max_disp;
save('Target_PDF_Data.mat', 'Y', 'Z', 'P_density_opt', 'grid_step', 'max_y_disp', 'max_z_disp', 'X_go');
%% ================= 5. Visualization (极致紧凑对齐版) =================
% 1. 调整画布比例：对于 1x3 排版，[1200, 400] 的比例最能消除左右空隙
fig = figure('Color', 'w', 'Position', [100, 100, 1200, 400]); 

% 2. 使用 tiledlayout 替代 subplot，它可以自动管理边距 (Padding)
t = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'tight');
colormap(flipud(hot));

% 统一轴限
plot_limit = max_disp; 

% --- 子图 1: 2D 散点图 ---
nexttile;
plot(all_endpoints(:,1), all_endpoints(:,2), '.', 'MarkerSize', 1, 'Color', [0.1 0.4 0.7]);
grid on; axis square;
xlim([-plot_limit plot_limit]); ylim([-plot_limit plot_limit]);
xlabel('Lateral $y$ (m)', 'Interpreter', 'latex');
ylabel('Vertical $z$ (m)', 'Interpreter', 'latex');
title('Impact Points Distribution', 'FontSize', 15);

% --- 子图 2: 2D 概率密度图 ---
nexttile;
contourf(Y, Z, P_density_opt, 40, 'LineStyle', 'none');
grid on; axis square;
xlim([-plot_limit plot_limit]); ylim([-plot_limit plot_limit]);
xlabel('Lateral $y$ (m)', 'Interpreter', 'latex');
title('2D Probability Density', 'FontSize', 15);
% 关键：让 colorbar 不占用子图空间
cb = colorbar;
cb.Label.String = 'Density';
cb.Label.Interpreter = 'latex';

% --- 子图 3: 3D 概率曲面 ---
nexttile;
surf(Y, Z, P_density_opt, 'EdgeColor', 'none'); 
grid on; axis square;
xlim([-plot_limit plot_limit]); ylim([-plot_limit plot_limit]);
zlim([0 max(P_density_opt(:))*1.1]);
view(-35, 35); 
xlabel('$y$ (m)', 'Interpreter', 'latex'); 
ylabel('$z$ (m)', 'Interpreter', 'latex');
title(sprintf('3D PDF Surface ($X_{go} = %.0f$ m)', X_go), 'Interpreter', 'latex', 'FontSize', 15);
lighting phong; camlight head;

% 3. 强制刷新：确保所有元素渲染完毕后再导出
drawnow;

% 4. 导出 (此时应该非常紧凑且对齐)
exportgraphics(gcf, 'Final_Result_Compact.png', 'Resolution', 1200);
%% ================= 6. Auxiliary Functions (按照你的要求保留) =================
function base_params = get_base_params()
    base_params.maxG_evade_mean = 7.0;
    base_params.maxG_evade_std = 1.5;
    base_params.maxG_evade_min = 5.0;
    base_params.maxG_evade_max = 9.0;
    
    base_params.snake_maxG_mean = 4.0;
    base_params.snake_maxG_std = 0.8;
    base_params.snake_maxG_min = 2.5;
    base_params.snake_maxG_max = 5.5;
    
    base_params.snake_freq_mean = 0.15;
    base_params.snake_freq_std = 0.03;
    base_params.snake_freq_min = 0.10;
    base_params.snake_freq_max = 0.20;
    
    base_params.roll_freq_mean = 0.20;
    base_params.roll_freq_std = 0.04;
    base_params.roll_freq_min = 0.12;
    base_params.roll_freq_max = 0.28;
    
    base_params.turn_radius_mean = 3000;
    base_params.turn_radius_std = 500;
    base_params.turn_radius_min = 2000;
    base_params.turn_radius_max = 4000;
    
    base_params.loop_radius_mean = 3000;
    base_params.loop_radius_std = 500;
    base_params.loop_radius_min = 2000;
    base_params.loop_radius_max = 4000;
    
    base_params.split_alt_drop_mean = 1000;
    base_params.split_alt_drop_std = 200;
    base_params.split_alt_drop_min = 700;
    base_params.split_alt_drop_max = 1300;
    
    base_params.immel_alt_rise_mean = 1000;
    base_params.immel_alt_rise_std = 200;
    base_params.immel_alt_rise_min = 700;
    base_params.immel_alt_rise_max = 1300;
end

function rand_params = generate_random_target_params(base_params)
    rand_params.maxG_evade = normrnd(base_params.maxG_evade_mean, base_params.maxG_evade_std);
    rand_params.maxG_evade = max(rand_params.maxG_evade, base_params.maxG_evade_min);
    rand_params.maxG_evade = min(rand_params.maxG_evade, base_params.maxG_evade_max);
    
    rand_params.snake_maxG = normrnd(base_params.snake_maxG_mean, base_params.snake_maxG_std);
    rand_params.snake_maxG = max(rand_params.snake_maxG, base_params.snake_maxG_min);
    rand_params.snake_maxG = min(rand_params.snake_maxG, base_params.snake_maxG_max);
    
    rand_params.snake_freq = normrnd(base_params.snake_freq_mean, base_params.snake_freq_std);
    rand_params.snake_freq = max(rand_params.snake_freq, base_params.snake_freq_min);
    rand_params.snake_freq = min(rand_params.snake_freq, base_params.snake_freq_max);
    
    rand_params.roll_freq = normrnd(base_params.roll_freq_mean, base_params.roll_freq_std);
    rand_params.roll_freq = max(rand_params.roll_freq, base_params.roll_freq_min);
    rand_params.roll_freq = min(rand_params.roll_freq, base_params.roll_freq_max);
    
    rand_params.turn_radius = normrnd(base_params.turn_radius_mean, base_params.turn_radius_std);
    rand_params.turn_radius = max(rand_params.turn_radius, base_params.turn_radius_min);
    rand_params.turn_radius = min(rand_params.turn_radius, base_params.turn_radius_max);
    
    rand_params.loop_radius = normrnd(base_params.loop_radius_mean, base_params.loop_radius_std);
    rand_params.loop_radius = max(rand_params.loop_radius, base_params.loop_radius_min);
    rand_params.loop_radius = min(rand_params.loop_radius, base_params.loop_radius_max);
    
    rand_params.split_alt_drop = normrnd(base_params.split_alt_drop_mean, base_params.split_alt_drop_std);
    rand_params.split_alt_drop = max(rand_params.split_alt_drop, base_params.split_alt_drop_min);
    rand_params.split_alt_drop = min(rand_params.split_alt_drop, base_params.split_alt_drop_max);
    
    rand_params.immel_alt_rise = normrnd(base_params.immel_alt_rise_mean, base_params.immel_alt_rise_std);
    rand_params.immel_alt_rise = max(rand_params.immel_alt_rise, base_params.immel_alt_rise_min);
    rand_params.immel_alt_rise = min(rand_params.immel_alt_rise, base_params.immel_alt_rise_max);
end