%%此示例说明如何使用标记的相机图像和 GPS 航点生成包含添加或删除带有交叉路口的车道的 RoadRunner 场景。
%链接：https://ww2.mathworks.cn/help/driving/ug/generate-scene-from-labeled-camera-gps-data.html
% 通过结合相机图像和 GPS 数据，您可以准确地重建包含车道信息和交叉路口的道路场景。相机图像使您能够轻松识别场景元素，例如车道标记和道路边界，而 GPS 航点使您能够将这些元素放置在世界坐标系中。此外，您还可以通过使用各种车道检测器来改进图像中车道边界的标记，无论使用何种摄像头，这些检测器都能可靠地运行。此示例将图像中的车道边界标签与 GPS 航点融合在一起，以生成准确的场景。
% 如下步骤：
% 加载一系列车道边界标记的图像和 GPS 航点。
% 将场景划分为多个段，每个段中具有固定数量的车道边界。
% 在整个序列中映射车道边界标签。
% 从图像中提取车道边界点并将其转换为世界坐标系。
% 从提取的车道边界点生成 RoadRunner 高清地图。


% 从 PandaSet 数据集下载包含传感器数据子集的 ZIP 文件，然后解压缩该文件
dataFolder = tempdir;
dataFilename = "PandasetSceneGeneration.zip";
url = "https://ssd.mathworks.com/supportfiles/driving/data/" + dataFilename;
filePath = fullfile(dataFolder,dataFilename);
if ~isfile(filePath)
    websave(filePath,url)
end
unzip(filePath,dataFolder)
% fullfile用作拼接数据集的完整路径
dataset = fullfile(dataFolder,"PandasetSceneGeneration");

%使用helperLoadExampleData函数将下载的数据集加载到工作区中
% imds— 图像的数据存储，指定为 imageDatastore 对象。
% sensor— 相机参数，指定为 monoCamera 对象。
% gTruth— 像素坐标中的车道边界标签，指定为 groundTruthMultisignal 对象。
% gps— 自我车辆的 GPS 坐标，指定为结构。
% timestamps— 激光雷达、摄像头和 GPS 数据的捕获时间（以秒为单位）。
[imds,~,sensor,gTruth,~,gps,timestamps] = helperLoadExampleData(dataset);

% 读取第一个点云、第一个图像及其相应的车道边界标签。
img = read(imds);
boundaries = gTruth.ROILabelData.Camera{1,:}{1};  %读取第一个数据

% 将车道边界标签叠加到图像上并可视化。
figure
imshow(img)
hold on

for i = 1:length(boundaries)
    boundary = boundaries(i);
    points = boundary.Position;
    plot(points(:,1),points(:,2),LineWidth=3,Color="r")
end

hold off


% 使用helperPlotGPS功能，将地理地图数据与自我车辆的GPS轨迹可视化。请注意，在图中，自我车辆在轨迹上从右下角移动到左上角。
helperPlotGPS(gps);

%%将场景划分为具有固定车道边界数的段
% 使用helperCreateSegments函数将场景划分为更小的段，每个段具有固定数量的车道边界。通过将场景划分为多个段，可以更轻松地提取每个车道边界的几何图形。
% 创建一个参考点geoRef数组，由第一个 GPS 数据点的纬度、经度和高度组成。
lat = [gps.lat]';  %转为列向量
lon = [gps.long]';
alt = [gps.height]';
geoRef = [lat(1),lon(1),alt(1)];
%latlon2local 函数将 GPS 坐标转换为基于参考点的本地坐标系下的笛卡尔坐标
%waypoints 里面存储场景中的路点或路径点。
[x,y,z] = latlon2local(lat,lon,alt,geoRef);
waypoints = [x,y,z];

%  helperCreateSegments 的辅助函数将场景划分为更小的段，每个段具有固定数量的车道边界。该函数接受车道边界标签数据 gTruth、路点数据 waypoints 和阈值参数 Threshold（设定为 18）作为输入，
% 划分后的段segments
segments = helperCreateSegments(gTruth,waypoints,Threshold=18);

% 使用helperPlotGPS函数可视化获得的五个片段
helperPlotGPS(gps,segments);

% 在路段中，由于车道边界被遮挡或移出摄像机视野，车道边界标签可能会在几帧内丢失。
% 要提取整个序列中每个车道边界的点，必须将右侧图像中的两个可见车道边界与左侧图像中最右侧的两个车道边界进行映射。


%% 使用helperMapLaneLabels函数将每帧中的车道边界与前一帧中的车道边界进行映射。
%创建一个与划分的段数量相同大小的单元数组mappedBoundaries，用于存储映射后的车道边界标签。
mappedBoundaries = cell(size(segments,1),1);

% 传入车道边界标签数据 gTruth、相机参数 sensor 以及当前划分的段 segments(i,:) 作为参数
for i = 1:size(segments,1)
    mappedBoundaries{i} = helperMapLaneBoundaryLabels(gTruth,sensor,segments(i,:));
end

% 使用helperPlotLaneBoundaries函数绘制特定帧的映射车道边界。此功能显示从左到右编号的车道边界
segmentNumber = 2; %第2段
frameNumber = 45;  %绘制第45帧
helperPlotLaneBoundaries(imds,mappedBoundaries,segments,segmentNumber,frameNumber);
title("All Lane Boundaries Visible")

% 绘制后续帧的车道边界。在此图中，您观察到最右边的两个车道边界编号为 3 和 4，即使在其他边界移出摄像机视野之后也是如此。
segmentNumber = 2;
frameNumber = 52;
helperPlotLaneBoundaries(imds,mappedBoundaries,segments,segmentNumber,frameNumber);
title("Only Right Lane Boundaries Visible")




%%从车道边界标签和 GPS 轨迹中提取车道几何
%使用helperConvertImageToVehicle函数通过投影变换将车道边界点从影像像素位置转换为车辆坐标系。
%注意以下情况可能会导致车道边界点不准确：
% 相机镜头失真 — 这会影响图像像素坐标的准确性，因此获得的通道边界可能会出现扭曲或弯曲。如果图像数据包含失真，则可以使用 undistortPoints 函数进行校正。
% 路面状况 — 这会影响车道边界点在像素坐标中的位置精度。例如，如果道路有凹陷或颠簸，摄像机的视点可能会发生变化，从而导致车道边界点不准确。如果测试路面不平坦，则可以将激光雷达数据与摄像头数据相结合，以获得准确的车道边界点。
mappedBoundaries = helperConvertImageToVehicle(mappedBoundaries,sensor);

% 使用 GPS 或 IMU 数据计算每个时间戳的自我车辆的偏航角。在此示例中，使用 GPS 航点计算偏航角。请注意，此示例中使用的 GPS 航点来自高精度传感器。对于嘈杂的 GPS 数据，使用此方法获得的方向可能不准确。如果 IMU 数据可用，您可以与 GPS 数据融合，以准确估计自负车辆的位置和方向。
yaw = atan2(y(2:end)-y(1:end-1),x(2:end)-x(1:end-1));
yaw = [yaw;yaw(end)];

% 参考自我位置和偏航信息创建自我车辆到世界坐标系刚性变换的数组。
tforms = [];
for i = 1:size(waypoints,1)
    tform = rigidtform2d(rad2deg(yaw(i)),[x(i) y(i)]);
    tforms = [tforms;tform];
end
% 使用计算变换将映射的车道边界从车辆坐标系变换为世界坐标系
laneBoundaryPoints = cell(size(segments,1),1);
for j = 1:size(segments,1)
    % Create a container for lane boundary points of current segment
    segmentPoints = cell(1,size(mappedBoundaries{j},2));
    % Get lane boundary points for current segment
    boundaries = mappedBoundaries{j};
    for i = segments(j,1):segments(j,2)
        
        % Create a container for lane boundary points of current frame
        lanes = mappedBoundaries{j}(i-segments(j,1)+1,:);
        % Loop over all the lane boundaries and transform the points
        for k = 1:length(lanes)
            points = lanes{k};
            if ~isempty(points)
                % Apply forward geometric transformation to the lane boundarie points
                [xT,yT] = transformPointsForward(tforms(i),points(:,1),points(:,2));
                segmentPoints{k} = [segmentPoints{k};[xT,yT]];
            end
        end
    end
    % Update the transformed lane boundary points
    laneBoundaryPoints{j} = segmentPoints;
end


% 在世界坐标系中可视化变换后的车道边界点。
figure
hold on
for i = 1:length(laneBoundaryPoints)
    for j = 1:length(laneBoundaryPoints{i})
        if ~isempty(laneBoundaryPoints{i}{1})
            plot(laneBoundaryPoints{i}{j}(:,1),laneBoundaryPoints{i}{j}(:,2),"b.")
        end
    end
end
title("Lane Boundary Points in World Coordinate System")
hold off

% 用该函数沿行驶方向对提取的车道边界点进行排序。这对于确保正确连接路段的几何形状是必要的。该函数通过比较自我车辆航点的起点和终点 x 坐标来计算自我车辆的行驶方向。
laneBoundaryPoints = helperSortPointsInTravelDirection(laneBoundaryPoints,waypoints,segments);




%%创建RoadRunner HD地图
% Delete the empty segments
empty_idx = cellfun(@(x) isempty(x{1}),laneBoundaryPoints);
laneBoundaryPoints(empty_idx) = [];

% Create a RoadRunner HD Map
rrMap = roadrunnerHDMap;
[laneInfo,geographicBoundary] = roadrunnerLaneInfo(laneBoundaryPoints,SmoothingDegree=1,SmoothingFactor=0.25,MinJunctionLength=10);
rrMap.Lanes = laneInfo.Lanes;
rrMap.LaneBoundaries = laneInfo.LaneBoundaries;
rrMap.GeographicBoundary = geographicBoundary;
write(rrMap,"rrMap.rrhd")

% 可视化生成的RoadRunner HD地图以及自我车辆航点。
plot(rrMap)
hold on
plot(waypoints(:,1),waypoints(:,2),"bo")
legend("Lane Boundaries","Lane Centers","Ego-Vehicle Waypoints")

%将 RoadRunner 高清地图写入二进制文件，并将其导入到 RoadRunner 中。
write(rrMap,"rrMap.rrhd")


% %%导入到 RoadRunner，目前缺少相应的软件许可证，无法使用importcence导入，可手动将该文件导进去。
% rrProjectPath = "D:\roadrunner\test";
% rrAppPath = "C:\Program Files\RoadRunner R2022b\bin\win64";
% rrApp = roadrunner(rrProjectPath,InstallationFolder=rrAppPath);
% 
% %指定 RoadRunner HD Map 导入选项。将 enableOverlapGroupsOptions设置为 false 以确保在两条相交的道路上构建交汇点
% overlapGroupsOptions = enableOverlapGroupsOptions(IsEnabled=false);
% buildOptions = roadrunnerHDMapBuildOptions(EnableOverlapGroupsOptions=overlapGroupsOptions);
% importOptions = roadrunnerHDMapImportOptions(BuildOptions=buildOptions);
% %将场景导入 RoadRunner，目前采用importScene导出到RoadRunner R2022b无法成功，采用手动导入.rrhd文件验证是否成功识别
% importScene(rrApp,fullfile(pwd,"rrMap.rrhd"),"RoadRunner HD Map",importOptions);
