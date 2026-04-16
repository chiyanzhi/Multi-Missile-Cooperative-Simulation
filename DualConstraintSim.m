clear; clc; close all;

%% 1. 环境准备与数据加载
if ~exist('Target_PDF_Data.mat', 'file')
    error('请先运行 quanjidongdaba.m 生成目标概率数据！');
end
% 加载你生成的逃逸概率密度 2D 数据
load('Target_PDF_Data.mat'); 

X_go = 27000; 

% --- 核心参数设置 ---
N_list = 1:4; 
all_FOVs = [10.0, 8.0, 8.0, 6.0];
all_a    = [10000, 9000, 8000, 7000]; 
all_b    = [8000, 8000, 7000, 6000];   

% 公式(4) 的连续探测学参数
P_max_val = 0.95; % 峰值截获概率
alpha_val = 12.0; % 衰减系数

weight_params.w1 = 1.0; 
weight_params.w2 = 0.1;

%% 2. 创建主画布
fig = figure('Name', 'Multi-Missile Coverage', 'Color', 'w', 'Position', [50, 300, 1600, 420]);

%% 3. 循环计算并绘制
for idx = 1:length(N_list)
    n = N_list(idx);
    fprintf('正在处理 N = %d 枚导弹...\n', n);
    
    m_params.maxFOV = all_FOVs(1:n);
    m_params.a = all_a(1:n);
    m_params.b = all_b(1:n);
    m_params.Pmax  = P_max_val; 
    m_params.alpha = alpha_val; 
    
    t_params_for_opt = struct('a', max_y_disp, 'b', max_z_disp); 
    
    % 调用优化器
    [optimal_Z, final_cov, final_red, ~, ~] = DualConstraint(...
        n, t_params_for_opt, m_params, weight_params, X_go, Y, Z, P_density_opt, grid_step);
    
    subplot(1, 4, idx);
    
    % ========================================================
    % 🚨 关键修复：绘制热力图并强行锁定颜色轴 (caxis)
    % ========================================================
    max_density = max(P_density_opt(:)); % 提取热力图的真实峰值
    
    contourf(Y, Z, P_density_opt, 30, 'LineStyle', 'none'); 
    colormap(flipud(hot)); 
    clim([0, max_density]); % 🚨 锁死颜色范围！禁止后续绘图篡改背景色
    hold on;
    
    % --- 绘制目标物理逃逸极限 (黑色细线) ---
    plot(max_y_disp * cos(linspace(0, 2*pi, 100)), ...
         max_z_disp * sin(linspace(0, 2*pi, 100)), 'k-', 'LineWidth', 1.0);
    
    colors = lines(n);
    for i = 1:n
        zy = optimal_Z(2*i-1); zz = optimal_Z(2*i);
        
        a_i = m_params.a(i);
        b_i = m_params.b(i);
        R_fov_i = X_go * tan(deg2rad(m_params.maxFOV(i)));
        
        % 计算这单枚导弹的综合有效概率场
        d_kin = 1 - sqrt(((Y - zy).^2)/(a_i^2) + ((Z - zz).^2)/(b_i^2));
        chi_kin_i = 1 ./ (1 + exp(-10 * d_kin));
        
        dist_to_center = sqrt((Y - zy).^2 + (Z - zz).^2);
        chi_fov_i = P_max_val ./ (1 + exp(alpha_val .* ((dist_to_center ./ R_fov_i) - 1)));
        
        chi_eff_i = chi_kin_i .* chi_fov_i;
        
        % 提取并绘制真实覆盖边界
        threshold_val = P_max_val * 0.5;
        contour(Y, Z, chi_eff_i, [threshold_val threshold_val], '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        
        % 绘制交接班中心点
        plot(zy, zz, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'w', 'MarkerSize', 6);
    end
    
    
    title(sprintf('(%s) $N = %d$ \n $E\\left[P_{int}\\right]$: %.2f\\%%\n SOP: %.2f\\%%', ...
        char(96+idx), n, final_cov*100, final_red*100), ...
        'Interpreter', 'latex', 'FontSize', 12);
    
    axis equal; grid on; 
    xlabel(' $y$ (m)', 'Interpreter', 'latex'); 
    ylabel(' $z$ (m)', 'Interpreter', 'latex');
    axis([-14000 14000 -10000 10000]); 
    set(gca, 'TickLabelInterpreter', 'latex', 'FontName', 'Times New Roman');
end

%% 4. 全局图例
L1 = plot(nan, nan, 'k-', 'LineWidth', 1.0);
L2 = plot(nan, nan, 'k-', 'LineWidth', 2.5); 
legend([L1, L2], {'Target Evasive Limit', 'Missile Effective Boundary (Kinematic \cap FOV)'}, ...
    'Orientation', 'horizontal', 'Position', [0.20, 0.02, 0.6, 0.03], 'FontSize', 11);

exportgraphics(gcf, 'DualConstraintSim.png', 'Resolution', 600);