% 使用importdata读取STL
filename = 'armpi_fpv/meshes/r_out_link.STL';
try
    stlData = importdata(filename);
    
    % 检查导入的数据结构
    if isstruct(stlData)
        disp('成功导入STL文件为结构体');
        disp(fieldnames(stlData));
        
        % 根据结构体字段提取顶点和面
        if isfield(stlData, 'vertices') && isfield(stlData, 'faces')
            vertices = stlData.vertices;
            faces = stlData.faces;
        elseif isfield(stlData, 'Points') && isfield(stlData, 'ConnectivityList')
            vertices = stlData.Points;
            faces = stlData.ConnectivityList;
        else
            error('无法识别的STL数据结构');
        end
    else
        % 可能是直接返回了triangulation对象
        vertices = stlData.Points;
        faces = stlData.ConnectivityList;
    end
    
    % 显示3D模型
    figure;
    p = patch('Vertices', vertices, 'Faces', faces);
    set(p, 'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    camlight('headlight');
    material('dull');
    axis equal;
    view(3);
    title('STL模型可视化');
    
catch ME
    disp(['错误: ' ME.message]);
end
