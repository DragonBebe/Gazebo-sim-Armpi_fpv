% 导入URDF文件及相关的STL模型
urdfFile = 'armpi_fpv/urdf/armpi_fpv.urdf';

% 尝试直接导入
try
    robot = importrobot(urdfFile);
    
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
    
    % 尝试获取机器人名称
    try
        if isprop(robot, 'BaseName')
            disp(['机器人名称: ' robot.BaseName]);
        elseif isprop(robot, 'name')
            disp(['机器人名称: ' robot.name]);
        elseif isprop(robot.Base, 'Name')
            disp(['机器人基座名称: ' robot.Base.Name]);
        else
            disp('无法获取机器人名称 - 属性不存在');
        end
    catch ME
        disp(['获取机器人名称失败: ' ME.message]);
    end
    
    % 获取关节数量
    disp(['关节数量: ' num2str(robot.NumBodies)]);
    
    % 手动计算非固定关节数量
    nonFixedJointCount = 0;
    for i = 1:length(robot.Bodies)
        if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
            nonFixedJointCount = nonFixedJointCount + 1;
        end
    end
    disp(['非固定关节数量: ' num2str(nonFixedJointCount)]);
    
    % 获取自由度
    if isprop(robot, 'NumDoF')
        disp(['自由度: ' num2str(robot.NumDoF)]);
    else
        disp(['自由度: ' num2str(nonFixedJointCount)]);
    end
    
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
    
catch ME
    disp(['导入失败: ' ME.message]);
end
