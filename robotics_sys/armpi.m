clear all;

% 导入URDF文件及相关的STL模型
urdfFile = 'armpi_fpv/urdf/armpi_fpv.urdf';

% 尝试直接导入
try
    % 导入时直接指定DataFormat为'column'
    robot = importrobot(urdfFile, 'DataFormat', 'column');
    
    % 显示机器人模型
    figure;
    show(robot);
    title('机械臂模型');
    
    % 添加灯光以更好地显示3D模型
    camlight('headlight');
    material('dull');
    
    % 显示机器人的基本信息
    robotInfo = whos('robot');
    disp(['机器人对象类型: ' robotInfo.class]);
    disp(['机器人名称: ' robot.BaseName]);
    disp(['关节数量: ' num2str(robot.NumBodies)]);
    
    % 手动计算非固定关节数量
    nonFixedJointCount = 0;
    nonFixedJointIndices = [];
    for i = 1:length(robot.Bodies)
        if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
            nonFixedJointCount = nonFixedJointCount + 1;
            nonFixedJointIndices(end+1) = i;
        end
    end
    disp(['非固定关节数量: ' num2str(nonFixedJointCount)]);
    
    % 获取自由度
    disp(['自由度: ' num2str(nonFixedJointCount)]);
    
    % 列出所有关节
    disp('关节列表:');
    for i = 1:length(robot.Bodies)
        body = robot.Bodies{i};
        jointType = body.Joint.Type;
        disp(['  - ' body.Name ' (关节类型: ' jointType ')']);
    end
    
    % 显示更多关于机器人的信息
    disp('机器人结构信息:');
    disp(robot);
    
    %% 添加重力物理限制和动力学分析
    disp('===== 添加重力物理限制 =====');
    
    % 设置重力向量 [x, y, z] 方向，单位为 m/s^2
    gravityVector = [0, 0, -9.81]; % 标准地球重力，向下为负
    disp(['设置重力向量: [' num2str(gravityVector) '] m/s^2']);
    
    % 设置机器人的重力属性
    robot.Gravity = gravityVector;
    disp('已将重力设置应用到机器人模型');
    
    % 获取机器人的默认配置
    config = homeConfiguration(robot);
    
    % 计算重力载荷（重力矩）
    try
        % 直接使用rigidBodyTree的方法计算重力力矩
        gravityTorque = gravityTorque(robot, config);
        disp('重力产生的关节力矩:');
        for i = 1:length(gravityTorque)
            disp(['  关节 ' num2str(i) ': ' num2str(gravityTorque(i)) ' N·m']);
        end
    catch ME
        disp(['计算重力载荷失败: ' ME.message]);
        disp('尝试替代方法...');
        
        try
            % 尝试使用inverseDynamics计算重力效应
            % 设置零速度和零加速度
            jointVelocities = zeros(size(config));
            jointAccelerations = zeros(size(config));
            
            % 使用逆动力学计算重力力矩
            gravityTorque = inverseDynamics(robot, config, jointVelocities, jointAccelerations);
            
            disp('重力产生的关节力矩 (使用inverseDynamics):');
            for i = 1:length(gravityTorque)
                disp(['  关节 ' num2str(i) ': ' num2str(gravityTorque(i)) ' N·m']);
            end
        catch ME2
            disp(['计算重力载荷的替代方法也失败: ' ME2.message]);
        end
    end
    
    % 计算质量矩阵
    try
        massMatrix = massMatrix(robot, config);
        disp('质量矩阵的对角线元素:');
        for i = 1:size(massMatrix, 1)
            disp(['  关节 ' num2str(i) ': ' num2str(massMatrix(i,i)) ' kg·m²']);
        end
    catch ME
        disp(['计算质量矩阵失败: ' ME.message]);
    end
    
    % 模拟重力下的静态平衡位置
    disp('===== 模拟重力下的静态平衡 =====');
    
    % 创建一个新的图形窗口
    figure;
    ax = show(robot, config);
    title('初始位置');
    camlight('headlight');
    material('dull');
    
    % 尝试模拟重力效应 - 使用更简单的方法
    try
        % 创建一个简单的动画来模拟重力效应
        numFrames = 50;
        
        % 获取所有非固定关节的索引和关节限制
        jointLimits = zeros(nonFixedJointCount, 2);
        jointIdx = 1;
        
        for i = 1:length(robot.Bodies)
            if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
                if ~isempty(robot.Bodies{i}.Joint.PositionLimits)
                    jointLimits(jointIdx, :) = robot.Bodies{i}.Joint.PositionLimits;
                else
                    jointLimits(jointIdx, :) = [-pi, pi];
                end
                jointIdx = jointIdx + 1;
            end
        end
        
        % 创建一个新的配置数组用于动画
        configArray = repmat(config, 1, numFrames);
        
        % 生成动画帧
        for frame = 1:numFrames
            % 计算当前帧的配置
            if frame <= numFrames/2
                % 前半段：逐渐增加重力效应
                factor = frame / (numFrames/2) * 0.1; % 最大偏移为0.1弧度
            else
                % 后半段：恢复到初始位置
                factor = (numFrames - frame) / (numFrames/2) * 0.1;
            end
            
            % 对每个关节应用不同的重力效应
            for j = 1:nonFixedJointCount
                % 根据关节的方向和类型模拟重力效应
                % 假设重力主要影响水平轴的旋转关节
                
                % 简单模拟：第1和第3关节受重力影响最大
                if j == 2 || j == 4
                    delta = -factor * 2; % 更大的影响
                else
                    delta = -factor;
                end
                
                % 确保在关节限制范围内
                newPos = config(j) + delta;
                configArray(j, frame) = min(max(newPos, jointLimits(j, 1)), jointLimits(j, 2));
            end
        end
        
        % 播放动画
        for frame = 1:numFrames
            show(robot, configArray(:, frame), 'Parent', ax);
            title(['重力模拟: 帧 ' num2str(frame) '/' num2str(numFrames)]);
            drawnow;
            pause(0.05);
        end
        
        % 显示最终的配置
        show(robot, config, 'Parent', ax);
        title('恢复到初始位置');
        
    catch ME
        disp(['模拟重力效应失败: ' ME.message]);
        disp(['错误详情: ' getReport(ME)]);
    end
    
    % 显示关节限制信息
    disp('===== 关节限制信息 =====');
    jointIndex = 1;
    for i = 1:length(robot.Bodies)
        body = robot.Bodies{i};
        joint = body.Joint;
        
        if ~strcmp(joint.Type, 'fixed')
            disp(['关节 ' num2str(jointIndex) ' (' joint.Name '):']);
            
            % 显示位置限制
            if ~isempty(joint.PositionLimits)
                lowerLimit = joint.PositionLimits(1);
                upperLimit = joint.PositionLimits(2);
                
                % 如果是旋转关节，转换为度
                if strcmp(joint.Type, 'revolute')
                    disp(['  位置限制: [' num2str(rad2deg(lowerLimit)) '°, ' num2str(rad2deg(upperLimit)) '°]']);
                else
                    disp(['  位置限制: [' num2str(lowerLimit) ', ' num2str(upperLimit) '] ' joint.PositionUnits]);
                end
            else
                disp('  位置限制: 无限制');
            end
            
            jointIndex = jointIndex + 1;
        end
    end
    
    %% 添加一个更实用的重力效应演示
    disp('===== 更实用的重力效应演示 =====');
    
    % 创建新的图形窗口
    figure;
    ax = show(robot, config);
    title('重力效应演示');
    camlight('headlight');
    material('dull');
    
    % 设置一个特定的机器人配置，使手臂伸展
    extendedConfig = config;
    extendedConfig(2) = pi/4;  % 第二个关节抬起
    extendedConfig(3) = -pi/4; % 第三个关节下弯
    
    % 显示伸展配置
    show(robot, extendedConfig, 'Parent', ax);
    title('伸展位置 - 无重力补偿');
    drawnow;
    pause(1);
    
    % 模拟重力作用下的下垂
    numFrames = 30;
    droopConfig = extendedConfig;
    
    for frame = 1:numFrames
        % 模拟重力导致手臂下垂
        factor = frame / numFrames * 0.2; % 最大下垂0.2弧度
        
        % 主要影响第2、3关节
        droopConfig(2) = extendedConfig(2) - factor;
        droopConfig(3) = extendedConfig(3) - factor*1.5;
        
        % 确保在关节限制范围内
        for j = 1:nonFixedJointCount
            if ~isempty(robot.Bodies{nonFixedJointIndices(j)}.Joint.PositionLimits)
                limits = robot.Bodies{nonFixedJointIndices(j)}.Joint.PositionLimits;
                droopConfig(j) = min(max(droopConfig(j), limits(1)), limits(2));
            end
        end
        
        show(robot, droopConfig, 'Parent', ax);
        title(['重力导致的下垂: ' num2str(round(frame/numFrames*100)) '%']);
        drawnow;
        pause(0.05);
    end
    
    % 显示最终下垂位置
    title('重力作用下的最终位置');
    pause(1);
    
    % 模拟重力补偿
    compensatedConfig = droopConfig;
    
    for frame = 1:numFrames
        % 逐渐恢复到伸展位置
        factor = frame / numFrames;
        compensatedConfig = droopConfig + factor * (extendedConfig - droopConfig);
        
        show(robot, compensatedConfig, 'Parent', ax);
        title(['重力补偿: ' num2str(round(frame/numFrames*100)) '%']);
        drawnow;
        pause(0.05);
    end
    
    % 显示最终补偿后的位置
    title('重力补偿后的位置');
    
catch ME
    disp(['导入或处理失败: ' ME.message]);
    disp(['错误详情: ' getReport(ME)]);
end
