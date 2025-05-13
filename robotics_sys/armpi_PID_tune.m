function armpi_PID_tune()
    % 加载机器人模型
    urdfPath = 'armpi_fpv/urdf/armpi_fpv.urdf';  % 请确保URDF文件在MATLAB路径中
    try
        robot = importrobot(urdfPath);
        robot.DataFormat = 'column';
        robot.Gravity = [0 0 0];  % 初始化不考虑重力
    catch ME
        % 如果无法加载URDF文件，尝试使用已加载的机器人模型
        fprintf('无法加载URDF文件，使用当前工作区中的机器人模型\n');
        try
            % 假设工作区中已有名为'robot'的变量
            if ~exist('robot', 'var')
                error('找不到机器人模型');
            end
        catch
            error('无法找到机器人模型');
        end
    end
    
    % 显示机器人基本信息
    fprintf('机器人对象类型: %s\n', class(robot));
    fprintf('机器人名称: %s\n', robot.BaseName);
    fprintf('关节数量: %d\n', length(robot.Bodies) + 1);  % +1 是因为基座
    
    % 获取非固定关节
    nonFixedJoints = 0;
    for i = 1:length(robot.Bodies)
        if robot.Bodies{i}.Joint.Type ~= "fixed"
            nonFixedJoints = nonFixedJoints + 1;
        end
    end
    fprintf('非固定关节数量: %d\n', nonFixedJoints);
    fprintf('自由度: %d\n', nonFixedJoints);
    
    % 显示关节列表
    fprintf('关节列表:\n');
    massMatrixToBodyIndex = zeros(nonFixedJoints, 1);
    jointIndex = 1;
    for i = 1:length(robot.Bodies)
        body = robot.Bodies{i};
        if body.Joint.Type == "fixed"
            fprintf('  - %s (关节类型: %s, 固定关节)\n', body.Name, body.Joint.Type);
        else
            fprintf('  - %s (关节类型: %s, 质量矩阵索引: %d)\n', body.Name, body.Joint.Type, jointIndex);
            massMatrixToBodyIndex(jointIndex) = i;
            jointIndex = jointIndex + 1;
        end
    end
    
    % 显示机器人结构信息
    fprintf('机器人结构信息:\n');
    disp(robot);
    
    % 计算质量矩阵
    config = homeConfiguration(robot);
    robotMassMatrix = massMatrix(robot, config);
    fprintf('质量矩阵:\n');
    disp(robotMassMatrix);
    
    % 计算逆动力学
    gravityTorques = gravityTorque(robot, config);
    fprintf('逆动力学计算的力矩:\n');
    disp(gravityTorques);
    
    % 识别机器人手臂关节 (Joint1-Joint5)
    armJointNames = {'link1', 'link2', 'link3', 'link4', 'link5'};
    armBodyIndices = [];
    armMassMatrixIndices = [];
    
    for i = 1:length(robot.Bodies)
        if ismember(robot.Bodies{i}.Name, armJointNames)
            armBodyIndices = [armBodyIndices, i];
            % 找到对应的质量矩阵索引
            for j = 1:length(massMatrixToBodyIndex)
                if massMatrixToBodyIndex(j) == i
                    armMassMatrixIndices = [armMassMatrixIndices, j];
                    break;
                end
            end
        end
    end
    
    fprintf('机器人手臂关节索引 (Bodies):\n');
    disp(armBodyIndices);
    
    fprintf('机器人手臂关节索引 (质量矩阵):\n');
    disp(armMassMatrixIndices);
    
    % 识别夹爪关节
    gripperJointNames = {'l_in_link', 'l_out_link', 'l_link', 'r_in_link', 'r_out_link', 'r_link'};
    gripperBodyIndices = [];
    gripperMassMatrixIndices = [];
    
    for i = 1:length(robot.Bodies)
        if ismember(robot.Bodies{i}.Name, gripperJointNames)
            gripperBodyIndices = [gripperBodyIndices, i];
            % 找到对应的质量矩阵索引
            for j = 1:length(massMatrixToBodyIndex)
                if massMatrixToBodyIndex(j) == i
                    gripperMassMatrixIndices = [gripperMassMatrixIndices, j];
                    break;
                end
            end
        end
    end
    
    fprintf('夹爪关节索引 (Bodies):\n');
    disp(gripperBodyIndices);
    
    fprintf('夹爪关节索引 (质量矩阵):\n');
    disp(gripperMassMatrixIndices);
    
    % 首先为机器人手臂关节设置PID参数
    fprintf('\n======== 调优机器人手臂关节 (Joint1-Joint5) ========\n');
    armPidParams = tuneArmPID(robot, robotMassMatrix, armMassMatrixIndices, massMatrixToBodyIndex);
    
    % 然后为夹爪关节设置PID参数
    fprintf('\n======== 调优夹爪关节 ========\n');
    gripperPidParams = tuneGripperPID(robot, robotMassMatrix, gripperMassMatrixIndices, massMatrixToBodyIndex);
    
    % 合并所有PID参数
    allPidParams = armPidParams;
    fn = fieldnames(gripperPidParams);
    for i = 1:length(fn)
        allPidParams.(fn{i}) = gripperPidParams.(fn{i});
    end
    
    % 导出机器人手臂PID参数到YAML文件
    exportPIDToYAML(armPidParams, armJointNames, 'arm_controller_tuned.yaml', 'arm_controller');
    fprintf('机器人手臂调优后的PID参数已导出到arm_controller_tuned.yaml文件\n');
    
    % 导出夹爪PID参数到YAML文件
    exportPIDToYAML(gripperPidParams, gripperJointNames, 'gripper_controller_tuned.yaml', 'gripper_controller');
    fprintf('夹爪调优后的PID参数已导出到gripper_controller_tuned.yaml文件\n');
    
    % 导出所有PID参数到单个YAML文件
    allJointNames = [armJointNames, gripperJointNames];
    exportPIDToYAML(allPidParams, allJointNames, 'all_controllers_tuned.yaml', 'joint_trajectory_controller');
    fprintf('所有关节调优后的PID参数已导出到all_controllers_tuned.yaml文件\n');
end

function pidParams = tuneArmPID(robot, robotMassMatrix, armMassMatrixIndices, massMatrixToBodyIndex)
    % 为机器人手臂关节调优PID参数
    pidParams = struct();
    
    % 设置合理的默认PID参数 - 手臂关节通常需要更高的增益
    defaultKp = 100.0;   % 比例增益
    defaultKi = 1.0;     % 积分增益
    defaultKd = 10.0;    % 微分增益
    
    % 对每个机器人手臂关节进行PID调优
    for i = 1:length(armMassMatrixIndices)
        massMatrixIndex = armMassMatrixIndices(i);
        bodyIndex = massMatrixToBodyIndex(massMatrixIndex);
        jointName = robot.Bodies{bodyIndex}.Name;
        
        % 获取关节的惯性值
        inertia = robotMassMatrix(massMatrixIndex, massMatrixIndex);
        fprintf('关节 %s 的原始惯性值: %.10e\n', jointName, inertia);
        
        % 尝试基于惯性值进行PID调优
        try
            % 如果惯性值太小，设置一个最小值以避免数值问题
            if inertia < 1e-3
                fprintf('警告: 惯性值 %.10e 太小，调整为 1e-3\n', inertia);
                inertia = 1e-3;
            end
            
            % 创建简化的二阶系统模型，添加适当的阻尼
            damping = 0.5;  % 手臂关节通常需要更高的阻尼
            s = tf('s');
            joint_tf = 1/(inertia*s^2 + damping*s);
            
            % 设置调优选项 - 手臂关节通常需要更快的响应
            opts = pidtuneOptions('PhaseMargin', 45);
            
            % 使用pidtune进行调优
            [C, info] = pidtune(joint_tf, 'PID', opts);
            
            % 检查PID参数是否在合理范围内
            if isReasonablePID_Arm(C.Kp, C.Ki, C.Kd)
                pidParams.(jointName).Kp = C.Kp;
                pidParams.(jointName).Ki = C.Ki;
                pidParams.(jointName).Kd = C.Kd;
                
                % 显示调优信息
                fprintf('调优信息: 相位裕度 = %.2f度, 带宽 = %.2f rad/s\n', info.PhaseMargin, info.Bandwidth);
            else
                % 如果参数不合理，使用默认值
                fprintf('警告: 计算的PID参数不合理，使用默认值\n');
                pidParams.(jointName).Kp = defaultKp;
                pidParams.(jointName).Ki = defaultKi;
                pidParams.(jointName).Kd = defaultKd;
            end
        catch ME
            % 如果调优失败，使用默认值
            fprintf('警告: PID调优失败，使用默认值。错误: %s\n', ME.message);
            pidParams.(jointName).Kp = defaultKp;
            pidParams.(jointName).Ki = defaultKi;
            pidParams.(jointName).Kd = defaultKd;
        end
        
        % 显示最终的PID参数
        fprintf('关节 %s 的PID参数: Kp = %.3f, Ki = %.3f, Kd = %.3f\n', ...
            jointName, pidParams.(jointName).Kp, pidParams.(jointName).Ki, pidParams.(jointName).Kd);
        
        % 可选：绘制阶跃响应
        try
            % 使用最终的PID参数创建控制器
            pid_controller = pid(pidParams.(jointName).Kp, pidParams.(jointName).Ki, pidParams.(jointName).Kd);
            
            % 创建闭环系统
            closedLoop = feedback(pid_controller*joint_tf, 1);
            
            % 绘制阶跃响应
            figure;
            step(closedLoop, 2);  % 模拟2秒
            title(['关节 ' jointName ' 的阶跃响应']);
            grid on;
        catch
            % 如果绘图失败，忽略错误
            fprintf('无法绘制阶跃响应图\n');
        end
    end
end

function pidParams = tuneGripperPID(robot, robotMassMatrix, gripperMassMatrixIndices, massMatrixToBodyIndex)
    % 为夹爪关节调优PID参数
    pidParams = struct();
    
    % 设置合理的默认PID参数 - 夹爪关节通常需要较小的增益
    defaultKp = 5.0;    % 比例增益
    defaultKi = 0.5;    % 积分增益
    defaultKd = 0.1;    % 微分增益
    
    % 对每个夹爪关节进行PID调优
    for i = 1:length(gripperMassMatrixIndices)
        massMatrixIndex = gripperMassMatrixIndices(i);
        bodyIndex = massMatrixToBodyIndex(massMatrixIndex);
        jointName = robot.Bodies{bodyIndex}.Name;
        
        % 获取关节的惯性值
        inertia = robotMassMatrix(massMatrixIndex, massMatrixIndex);
        fprintf('关节 %s 的原始惯性值: %.10e\n', jointName, inertia);
        
        % 尝试基于惯性值进行PID调优
        try
            % 如果惯性值太小，设置一个最小值以避免数值问题
            if inertia < 1e-5
                fprintf('警告: 惯性值 %.10e 太小，调整为 1e-5\n', inertia);
                inertia = 1e-5;
            end
            
            % 创建简化的二阶系统模型，添加适当的阻尼
            damping = 0.1;  % 夹爪关节通常阻尼较小
            s = tf('s');
            joint_tf = 1/(inertia*s^2 + damping*s);
            
            % 设置调优选项 - 夹爪关节通常需要更平稳的响应
            opts = pidtuneOptions('PhaseMargin', 60);
            
            % 使用pidtune进行调优
            [C, info] = pidtune(joint_tf, 'PID', opts);
            
            % 检查PID参数是否在合理范围内
            if isReasonablePID_Gripper(C.Kp, C.Ki, C.Kd)
                pidParams.(jointName).Kp = C.Kp;
                pidParams.(jointName).Ki = C.Ki;
                pidParams.(jointName).Kd = C.Kd;
                
                % 显示调优信息
                fprintf('调优信息: 相位裕度 = %.2f度, 带宽 = %.2f rad/s\n', info.PhaseMargin, info.Bandwidth);
            else
                % 如果参数不合理，使用默认值
                fprintf('警告: 计算的PID参数不合理，使用默认值\n');
                pidParams.(jointName).Kp = defaultKp;
                pidParams.(jointName).Ki = defaultKi;
                pidParams.(jointName).Kd = defaultKd;
            end
        catch ME
            % 如果调优失败，使用默认值
            fprintf('警告: PID调优失败，使用默认值。错误: %s\n', ME.message);
            pidParams.(jointName).Kp = defaultKp;
            pidParams.(jointName).Ki = defaultKi;
            pidParams.(jointName).Kd = defaultKd;
        end
        
        % 显示最终的PID参数
        fprintf('关节 %s 的PID参数: Kp = %.3f, Ki = %.3f, Kd = %.3f\n', ...
            jointName, pidParams.(jointName).Kp, pidParams.(jointName).Ki, pidParams.(jointName).Kd);
    end
end

function reasonable = isReasonablePID_Arm(Kp, Ki, Kd)
    % 检查机器人手臂关节PID参数是否在合理范围内
    % 手臂关节通常需要更高的增益
    reasonable = (Kp > 0 && Kp < 1000) && ...  % 比例增益可以较高
                (Ki >= 0 && Ki < 100) && ...   % 积分增益适中
                (Kd >= 0 && Kd < 100);         % 微分增益适中
end

function reasonable = isReasonablePID_Gripper(Kp, Ki, Kd)
    % 检查夹爪关节PID参数是否在合理范围内
    % 夹爪关节通常需要较小的增益
    reasonable = (Kp > 0 && Kp < 100) && ...  % 比例增益适中
                (Ki >= 0 && Ki < 50) && ...   % 积分增益不应过大
                (Kd >= 0 && Kd < 10);         % 微分增益通常较小
end

function exportPIDToYAML(pidParams, jointNames, filename, controllerName)
    % 创建YAML文件内容
    yamlContent = ['# ' controllerName ' 控制器\n' controllerName ':\n'];
    yamlContent = [yamlContent '  type: effort_controllers/JointTrajectoryController\n'];
    
    % 添加关节列表
    yamlContent = [yamlContent '  joints: ['];
    for i = 1:length(jointNames)
        if i > 1
            yamlContent = [yamlContent ', '];
        end
        yamlContent = [yamlContent '''' jointNames{i} ''''];
    end
    yamlContent = [yamlContent ']\n'];
    
    % 添加增益
    yamlContent = [yamlContent '  gains:\n'];
    
    % 添加每个关节的PID参数
    for i = 1:length(jointNames)
        jointName = jointNames{i};
        if isfield(pidParams, jointName)
            yamlContent = [yamlContent sprintf('    %s: {p: %.3f, i: %.3f, d: %.3f, i_clamp: 1.0}\n', ...
                jointName, pidParams.(jointName).Kp, pidParams.(jointName).Ki, pidParams.(jointName).Kd)];
        else
            % 如果没有找到参数，使用默认值
            if strcmp(controllerName, 'arm_controller')
                % 手臂关节默认值
                yamlContent = [yamlContent sprintf('    %s: {p: 100.000, i: 1.000, d: 10.000, i_clamp: 1.0}\n', jointName)];
            else
                % 夹爪关节默认值
                yamlContent = [yamlContent sprintf('    %s: {p: 5.000, i: 0.500, d: 0.100, i_clamp: 0.1}\n', jointName)];
            end
        end
    end
    
    % 添加其他控制器参数
    yamlContent = [yamlContent '  joint_limit_margin: 0.1\n'];
    yamlContent = [yamlContent '  constraints:\n'];
    yamlContent = [yamlContent '    goal_time: 1.0\n'];
    yamlContent = [yamlContent '    stopped_velocity_tolerance: 0.01\n'];
    
    % 添加每个关节的约束
    for i = 1:length(jointNames)
        if strcmp(controllerName, 'arm_controller') || strcmp(controllerName, 'joint_trajectory_controller')
            % 手臂关节需要更精确的轨迹跟踪
            yamlContent = [yamlContent sprintf('    %s: {trajectory: 0.1, goal: 0.1}\n', jointNames{i})];
        else
            % 夹爪关节可以有更宽松的约束
            yamlContent = [yamlContent sprintf('    %s: {trajectory: 0.2, goal: 0.2}\n', jointNames{i})];
        end
    end
    
    % 添加最后的参数
    yamlContent = [yamlContent '  stop_trajectory_duration: 0.5\n'];
    yamlContent = [yamlContent '  state_publish_rate: 25\n'];
    
    % 写入文件
    fid = fopen(filename, 'w');
    if fid == -1
        error('无法创建文件 %s', filename);
    end
    fprintf(fid, '%s', yamlContent);
    fclose(fid);
end
