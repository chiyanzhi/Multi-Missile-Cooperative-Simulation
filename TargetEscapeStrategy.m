function escapeAccCmd = TargetEscapeStrategy(targetState, missileState, params, maneuverType, environment)
    % TargetEscapeStrategy 目标机动指令生成
    % 核心升级：基于威胁等级的智能机动决策，兼容手动指定机动
    % 威胁等级：无威胁→低威胁→中威胁→高威胁，自动匹配对应机动
    % 包含论文标准8种机动+置尾逃逸，完全兼容原有参数

    g = environment.g;
    vel = targetState.vel;
    speed = norm(vel);
    
    % 顶级持久化变量：新增 right_vec 用于锁定旋转平面，防止垂直死锁
    persistent split_done split_triggered split_init_alt split_right_vec immel_done immel_triggered immel_init_alt immel_right_vec;
    
    % 仿真全局重置
    if targetState.time < 0.1
        split_done = false; split_triggered = false; split_init_alt = 0; split_right_vec = [0,1,0];
        immel_done = false; immel_triggered = false; immel_init_alt = 0; immel_right_vec = [0,1,0];
    end
    % ================= 1. 核心态势与威胁等级计算 =================
    % 1.1 基础态势计算
    r_vec = targetState.pos - missileState.pos; % 目标→导弹向量
    dist = norm(r_vec); % 弹目距离
    los_dir = r_vec / (dist + 1e-6); % 视线方向
    v_rel = targetState.vel - missileState.vel; % 相对速度
    approach_speed = -dot(v_rel, los_dir); % 接近速度（正数=导弹正在靠近）
    omega = cross(r_vec, v_rel) / (dot(r_vec, r_vec) + 1e-6); % 视线角速度
    omega_mag = norm(omega); % 视线角速度大小（越大=导弹机动越剧烈，威胁越高）
    isSeekerActive = isfield(missileState, 'isSeekerActive') && missileState.isSeekerActive;

    % 1.2 威胁等级划分（完全贴合中距空空导弹攻防逻辑）
    threat_level = 0; % 0=无威胁 1=低威胁 2=中威胁 3=高威胁
    if dist > 60000
        threat_level = 0; % 60km外：无威胁
    elseif dist > 40000 && dist <= 60000
        threat_level = 1; % 40-60km：低威胁
    elseif dist > 27000 && dist <= 40000
        threat_level = 2; % 27-40km：中威胁（不可逃逸区边缘）
    else
        threat_level = 3; % 27km内：高威胁（导引头开机，末制导）
    end
    % 导引头开机强制拉满威胁等级
    if isSeekerActive
        threat_level = 3;
    end
    % 视线角速度过大，提升威胁等级（导弹在全力机动追踪）
    if omega_mag > 0.005
        threat_level = min(threat_level + 1, 3);
    end

    % ================= 2. 机动决策逻辑 =================
    auto_maneuver = 'levelflight'; % 默认机动
    switch threat_level
        case 0 % 无威胁：平飞巡航
            auto_maneuver = 'levelflight';
        case 1 % 低威胁：小幅预警机动，保持能量
            auto_maneuver = 'snake'; % 小过载蛇形机动，破坏导弹预测
        case 2 % 中威胁：中等强度机动，提前转向/占位
            auto_maneuver = 'tailchase'; % 置尾转向，拉开距离，规避中制导
        case 3 % 高威胁：极限大过载规避机动
            % 高威胁下随机选极限机动，增加不可预测性（也可以固定为某一种）
            maneuver_list = {'splits', 'barrelroll', 'verticalloop', 'immelmann'};
            %rng(targetState.time*1000); % 基于时间的随机种子，保证复现性
            auto_maneuver = maneuver_list{randi(length(maneuver_list))};
    end

    % 决策优先级：手动指定机动 > 自动威胁决策
    if ~strcmpi(maneuverType, 'auto')
        effectiveManeuver = lower(maneuverType);
    else
        effectiveManeuver = auto_maneuver;
    end

    % ================= 3. 基础向量计算 =================
    if speed > 1e-1
        vel_dir = vel / speed; % 目标速度方向单位向量
    else
        vel_dir = [1,0,0]; % 速度为0时默认沿X轴
    end
    
    up = [0, 0, 1]; % 全局天顶方向
    right_vec = cross(vel_dir, up); % 速度坐标系右侧（水平面）
    if norm(right_vec) < 1e-3, right_vec = [0, 1, 0]; end 
    right_vec = right_vec / norm(right_vec);
    
    lift_vec = cross(right_vec, vel_dir); % 速度坐标系升力方向（垂直速度向上）
    lift_vec = lift_vec / norm(lift_vec);

    % ================= 4. 机动参数默认值读取 =================
    if isfield(params, 'turn_radius'), R_turn = params.turn_radius; else, R_turn = 3000; end % 论文默认3000m
    if isfield(params, 'loop_radius'), R_loop = params.loop_radius; else, R_loop = 3000; end
    if isfield(params, 'snake_freq'), snake_omega = 2*pi*params.snake_freq; else, snake_omega = 2*pi*0.15; end % 论文0.15Hz
    if isfield(params, 'snake_maxG'), snake_maxG = params.snake_maxG; else, snake_maxG = 4.0; end % 论文4G

    % ================= 5. 核心机动逻辑（严格对齐论文机动库） =================
    switch effectiveManeuver
        % ========== 论文机动1：平飞机动 ==========
        case 'levelflight'
            accCmd = [0, 0, g];

        % ========== 论文机动2：定半径盘旋机动 ==========
        case 'steadyturn'
            req_acc = (speed^2) / R_turn;
            req_g = req_acc / g;
            final_g = min(req_g, params.maxG_evade);
            accCmd = right_vec * final_g * g + [0, 0, g];

        % ========== 论文机动3：定半径筋斗机动 ==========
        case 'verticalloop'
            req_acc = (speed^2) / R_loop;
            req_g = req_acc / g;
            final_g = min(req_g, params.maxG_evade);
            accCmd = lift_vec * max(final_g, 1.5) * g;

        % ========== 论文机动4：蛇形机动 ==========
        case 'snake'
            weave_g = snake_maxG * sin(snake_omega * targetState.time);
            accCmd = lift_vec * weave_g * g + [0, 0, g];

        % ========== 论文机动5：滚筒机动 ==========
        case 'barrelroll'
            roll_omega = 2 * pi * 0.2;
            acc_right = right_vec * params.maxG_evade * 0.7 * g * cos(roll_omega * targetState.time);
            acc_up = lift_vec * params.maxG_evade * 0.7 * g * sin(roll_omega * targetState.time);
            accCmd = acc_right + acc_up + [0, 0, g];

        % ========== 论文机动6：螺旋上升机动 ==========
        case 'spiralclimb'
            req_acc_turn = (speed^2) / R_turn;
            req_g_turn = req_acc_turn / g;
            final_g_turn = min(req_g_turn, params.maxG_evade*0.6);
            climb_g = 0.3 * params.maxG_evade;
            accCmd = right_vec * final_g_turn * g + lift_vec * climb_g * g + [0, 0, g];

        % ========== 破S机动 (Split-S) ==========
        case 'splits'
            current_alt = targetState.pos(3);
        
            if ~split_triggered && dist < 60000 % 可根据需要调整触发距离
                split_triggered = true;
                split_init_alt = current_alt;
                split_done = false;
                
                % 【核心修复】在触发瞬间，锁定当前的“水平右侧”向量作为旋转轴
                v_xy = [vel_dir(1), vel_dir(2), 0];
                if norm(v_xy) < 1e-3
                    split_right_vec = [0, 1, 0];
                else
                    split_right_vec = cross(v_xy / norm(v_xy), [0, 0, 1]);
                end
                %fprintf('破S已触发！锁定旋转平面。初始高度=%.0fm\n', split_init_alt);
            end
        
            if split_done || ~split_triggered
                accCmd = [0, 0, g];
            else
                % 使用锁定的旋转轴计算升力方向，完美避开垂直死锁
                current_lift_vec = cross(split_right_vec, vel_dir);
                current_lift_vec = current_lift_vec / (norm(current_lift_vec) + 1e-6);
                
                loop_radius = 2000;
                req_acc = speed^2 / loop_radius;
                req_g = req_acc / g;
                final_g = min(req_g, params.maxG_evade);
                
                % 破S是向下半扣，加速度方向取反
                accCmd = -current_lift_vec * final_g * g;
        
                % 完成条件：高度下降且速度重新回归水平
                alt_drop = split_init_alt - current_alt;
                if alt_drop >= 1000 && abs(vel_dir(3)) < 0.05
                    split_done = true;
                    fprintf('破S圆满完成！完美掉头。最终高度=%.0fm\n', current_alt);
                end
        
                % 安全保护 + 过载限幅
                if current_alt < 1500
                    accCmd = current_lift_vec * 0.8 * g + [0, 0, g]; 
                    split_done = true;
                end
                acc_mag = norm(accCmd);
                if acc_mag > params.maxG_evade * g
                    accCmd = accCmd / acc_mag * (params.maxG_evade * g);
                end
            end

        % ========== 殷麦曼机动 (Immelmann) ==========
        case 'immelmann'
            current_alt = targetState.pos(3);
            
            if ~immel_triggered && dist < 60000
                immel_triggered = true;
                immel_init_alt = current_alt; 
                immel_done = false;
                
                % 【核心修复】同样锁定旋转平面
                v_xy = [vel_dir(1), vel_dir(2), 0];
                if norm(v_xy) < 1e-3
                    immel_right_vec = [0, 1, 0];
                else
                    immel_right_vec = cross(v_xy / norm(v_xy), [0, 0, 1]);
                end
                %fprintf('殷麦曼已触发！锁定旋转平面。初始高度=%.0fm\n', immel_init_alt);
            end
            
            if immel_done || ~immel_triggered
                accCmd = [0, 0, g];
            else
                % 使用锁定的旋转轴计算升力方向
                current_lift_vec = cross(immel_right_vec, vel_dir);
                current_lift_vec = current_lift_vec / (norm(current_lift_vec) + 1e-6);
                
                loop_radius = 2000;
                req_acc = speed^2 / loop_radius;
                req_g = req_acc / g;
                final_g = min(req_g, params.maxG_evade);
                
                % 殷麦曼是向上拉起
                accCmd = current_lift_vec * final_g * g; 
                
                % 完成条件：高度爬升且速度重新回归水平
                alt_rise = current_alt - immel_init_alt; 
                if alt_rise >= 1000 && abs(vel_dir(3)) < 0.05
                    immel_done = true;
                    fprintf('殷麦曼圆满完成！完美掉头。最终高度=%.0fm\n', current_alt);
                end
                
                % 安全保护 + 过载限幅
                if current_alt > 15000
                    accCmd = -current_lift_vec * 0.8 * g + [0, 0, g];
                    immel_done = true;
                end
                acc_mag = norm(accCmd);
                if acc_mag > params.maxG_evade * g
                    accCmd = accCmd / acc_mag * (params.maxG_evade * g);
                end
            end
        % ========== 置尾逃逸机动 ==========
        case 'tailchase'
            accel_dir = [0,0,0];
            R_vec_m2t = missileState.pos - targetState.pos;
            los_vec_m2t = R_vec_m2t / (norm(R_vec_m2t) + 1e-6);
            vel_missile = missileState.vel;
            vel_target = targetState.vel;
            speed_missile = norm(vel_missile);
            speed_target = norm(vel_target);
            
            vel_dir_missile = vel_missile / max(speed_missile, 1e-3);
            vel_dir_target = vel_target / max(speed_target, 1e-3);
            
            angle_los_vel = acos(dot(los_vec_m2t, vel_dir_target));
            is_missile_in_front = angle_los_vel < 120/180*pi;
            
            up = [0,0,1];
            missile_vel_horiz = vel_dir_missile;
            missile_vel_horiz(3) = 0;
            missile_vel_horiz = missile_vel_horiz / max(norm(missile_vel_horiz), 1e-3);
            missile_side = cross(missile_vel_horiz, up);
            missile_side = missile_side / max(norm(missile_side), 1e-3);
            
            desired_vel_dir = vel_dir_missile + 0.8 * missile_side;
            desired_vel_dir(3) = 0;
            desired_vel_dir = desired_vel_dir / max(norm(desired_vel_dir), 1e-3);
            
            desired_turn_dir = desired_vel_dir - dot(desired_vel_dir, vel_dir_target) * vel_dir_target;
            desired_turn_dir = desired_turn_dir / max(norm(desired_turn_dir), 1e-3);
            
            if is_missile_in_front
                escape_g = 1.0 * params.maxG_evade;
            else
                escape_g = 0.4 * params.maxG_evade;
                if speed_target < params.maxSpeed
                    accel_dir = vel_dir_target * 0.2 * g;
                end
            end
            
            acc_turn = desired_turn_dir * escape_g * g;
            acc_gravity = [0, 0, g];
            accCmd = acc_turn + acc_gravity + accel_dir;
            
            max_acc = params.maxG_evade * g;
            accCmd_mag = norm(accCmd);
            if accCmd_mag > max_acc
                accCmd = accCmd / accCmd_mag * max_acc;
            end

        % ========== 俯冲机动 ==========
        case 'dive'
            accCmd = -lift_vec * params.maxG_evade * g + [0,0,g];

        % ========== 默认：平飞 ==========
        otherwise
            accCmd = [0, 0, g];
    end
    
    escapeAccCmd = accCmd;
end