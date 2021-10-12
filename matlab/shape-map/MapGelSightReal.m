%% 3-D shape mapping with GelSight and Depth camera, real

% Copyright (c) 2021 Sudharshan Suresh <suddhu@cmu.edu>
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:

% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer.

% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.

% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function MapGelSightReal(varargin)
    close all;
    gpisRootPath = getenv('GPIS_PATH');
    addpath('/usr/local/lib'); 
    addpath(genpath(fullfile(gpisRootPath, 'matlab')));
    shapeMappingRootPath = getenv('SHAPE_MAPPING_ROOT');
    addpath([shapeMappingRootPath,'/scripts', '/matlab']) 
    
    %% hyperparameters   
    params = inputParser;
    addParameter(params,'debugFlag',false);
    addParameter(params,'MeshN',16);
    addParameter(params,'dim',3);
    addParameter(params,'kernelType',"thinplate");
    addParameter(params,'poseNoise',[1e-3*ones(1, 3), 1e-1*ones(1, 3)],);
    addParameter(params,'measNoise',[6e-4, 1e-3, 1e-3, 1e-3]);
    addParameter(params,'depthNoise',[1e-3, 1e-3, 1e-3, 1e-3]);
    addParameter(params,'inputNoise',[0, 0, 0]);
    addParameter(params,'compact',true);
    addParameter(params,'depthRes',3e-3);
    addParameter(params,'measRes',1e-3);
    addParameter(params,'training','incremental');
    addParameter(params,'shape','010_potted_meat_can');
    addParameter(params,'genType','fcrn');
    addParameter(params,'elev',30);
    parse(params, varargin{:});
    params = params.Results;

    %% load pointcloud data: x, y, z, nx, ny, nz
    datasetRoot = "/media/suddhu/Backup Plus/suddhu/rpl/datasets/tactile_mapping/";
    datasetName = 'textured_60sampled';
    
    gelsightFile = fullfile(datasetRoot, 'ycbSight2', params.shape, 'gelsight_data.mat'); load(gelsightFile);
    
    %% get generated data
    if strcmp(params.genType, 'fcrn')
        genFile = fullfile(datasetRoot, 'generated_data', 'fcrn', params.shape, 'fcrn_data.mat'); 
        genData = load(genFile); gen_heightmaps = genData.fcrn_heightmaps; gen_normalmaps = genData.fcrn_normalmaps; 
    elseif strcmp(dataSource, 'lookup')
        genFile = fullfile(datasetRoot, 'generated_data', 'lookup', params.shape, 'lookup_data.mat'); 
        genData = load(genFile); gen_heightmaps = genData.lookup_heightmaps; gen_normalmaps = genData.lookup_normalmaps; 
    end
           
    modelFile = fullfile('/home/suddhu/projects/Shape_Mapping_Tactile/', 'aligned_models', strcat(params.shape, '.stl')); 
    [TriObj, XObj] = stlread(modelFile);
    numMeas = length(poses)/2; 
    
    %% transforms
    world2azure = toPose(world2azure);     
    world2object = toPose(world2object).inv; 
    gripper2gelsight = toPose(gripper2gelsight);
    gripper2gelsight.t(2) = gripper2gelsight.t(2) + 5e-3; % 5mm offset compensation 

    sensorPoses = {}; sensorPositions = zeros(numMeas, 3);
    
    %% extract patches from point cloud 
    fprintf('Extracting tactile + vision data\n');
    patches = []; sensorPoses = {};
    
    count = 1; 
    for i = 2:2:2*numMeas
        poses(i, [4, 5, 6, 7]) = poses(i, [7, 4, 5, 6]);        % x y z w -> w x y z
        
        %% get sensorpose
        sensorPose = SE3(quat2rotm(poses(i, 4:end)), poses(i, 1:3));
        gelsightTrans = SE3(eye(3), gripper2gelsight.t); 
        sensorPose = sensorPose * gelsightTrans;
        gelsightRot = SE3(gripper2gelsight.R, zeros(1, 3)); 
        sensorPose = sensorPose * gelsightRot;    
        
        sensorPose = world2object * sensorPose;
        sensorPoses{count} = sensorPose; 
        sensorPositions(count, :) = sensorPose.t';
        count = count + 1;
        
        %% get 2-D image information
        heightmap = reshape(gen_heightmaps(:, i),[480,640]);
        normalmap = reshape(gen_normalmaps(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480*640, 3]);  
        contactmask = reshape(est_contactmasks(:, i),[480,640]);
        tactileimage = reshape(tactileimages(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480,640, 3]);
        tactileimage = rescale(tactileimage); 
        
        %% get world point cloud      
        world_point_cloud = heightMap2Cloud(heightmap, normalmap, contactmask, sensorPose);
        world_point_cloud = pcdownsample(world_point_cloud,'gridAverage', params.measRes);

        patches(end + 1).X = world_point_cloud.Location;
        patches(end).Y = world_point_cloud.Normal;
        patches(end).Y = normr(patches(end).Y);
    end
    
    %% get object depth in bounding box
    world_depth_map = pointCloud((world2azure * depth_map')');
    object_depth_map = pointCloud((world2object * world_depth_map.Location')');
    
    xmin = min(sensorPositions(:, 1)); xmax = max(sensorPositions(:, 1)); 
    ymin = min(sensorPositions(:, 2));  ymax = max(sensorPositions(:, 2)); 
    zmin = min(sensorPositions(:, 3));   zmax = max(sensorPositions(:, 3)); 
    roi = [xmin xmax ymin ymax zmin zmax] + [-1 1 -1 1 -1 3]*1e-2;
    indices = findPointsInROI(object_depth_map,roi);
    object_depth_map = select(object_depth_map,indices);

    origin = mean(sensorPositions);    
    object_depth_map.Normal = object_depth_map.Location - origin;
    object_depth_map.Normal = (object_depth_map.Normal)./vecnorm(object_depth_map.Normal, 2, 2);
    object_depth_map = pcdownsample(object_depth_map,'gridAverage',params.depthRes);
    
    %% Hallucinate base measurements     
    x = sensorPositions(5:5:end, 1)'; x(end + 1) = x(1);
    y = sensorPositions(5:5:end, 2)'; y(end + 1) = y(1);
    meshx = (min(x)):3e-3:(max(x));  meshy = (min(y)):3e-3:(max(y)); 
    [xm,ym] = meshgrid(meshx,meshy);
    xm = xm(:)'; ym = ym(:)'; 
    
    in1 = inpolygon(xm,ym,x, y);     % get the contour 
    bottomPts = [xm(in1)',ym(in1)', abs(world2object.t(3))*ones(size(xm(in1)'))]; 
    in2 = inpolygon(xm,ym,x - sign(x)*5e-3, y - sign(y)*5e-3);
    sidePtsXY = [xm(~in2)',ym(~in2)'];     % get the side points 
    
    z = mean(sensorPositions(5:5:end, 3));
    hs = linspace(abs(world2object.t(3)), z, 6); 
    sidePtsZ = [repmat(hs(1), size(sidePtsXY, 1), 1); repmat(hs(2), size(sidePtsXY, 1), 1);...
                repmat(hs(3), size(sidePtsXY, 1), 1); repmat(hs(4), size(sidePtsXY, 1), 1);...
                repmat(hs(5), size(sidePtsXY, 1), 1); repmat(hs(6), size(sidePtsXY, 1), 1)];
    
    sidePts = [repmat(sidePtsXY, 6, 1), sidePtsZ];
    sideNormals = sidePts - [origin(1), origin(2), mean(sidePtsZ)]; 
    sideNormals = (sideNormals)./vecnorm(sideNormals, 2, 2);

    bottomPts = [bottomPts; sidePts];
    bottomPts = pointCloud(bottomPts);
    bottomPts.Normal = [repmat([0 0 -1],bottomPts.Count - size(sidePts, 1),1);  sideNormals];

    %% kernel parameters
    if strcmp(params.kernelType, "thinplate")
        params.theta = [0, 0.25, 1]; 
    elseif strcmp(params.kernelType, "rbf")
        params.theta = [1, 0.25, 0.01]; 
    end
    
    fprintf('Training data points: %d, Query points: %d\n', numMeas, params.MeshN^3);        
    %% generate 3D mesh query points
    params.meshLim = [min(sensorPositions(:, 1)) - 2e-2 max(sensorPositions(:, 1)) + 1.5e-2;
                      min(sensorPositions(:, 2)) - 2e-2 max(sensorPositions(:, 2)) + 1.5e-2;
                      abs(world2object.t(3)) - 2e-2 max(sensorPositions(:, 3)) + 3e-2;];
    
    % assign compact radius: tuning for each object (can work well with fixed value too)
    if (strcmp(params.shape, "010_potted_meat_can"))
        params.compRadius = 0.15*max(params.meshLim(:, 2) - params.meshLim(:,1));    
    elseif (strcmp(params.shape, "021_bleach_cleanser"))
        params.compRadius = 0.16*mean(params.meshLim(:, 2) - params.meshLim(:,1));    
    else
        params.compRadius = 0.15*mean(params.meshLim(:, 2) - params.meshLim(:,1));
    end
    params.cleanRadius =  params.compRadius;
    
    Grid = {linspace(params.meshLim(1, 1), params.meshLim(1, 2), params.MeshN),...
            linspace(params.meshLim(2, 1), params.meshLim(2, 2), params.MeshN)...
            linspace(params.meshLim(3, 1), params.meshLim(3, 2), params.MeshN)};
    [xt, yt, zt] = meshgrid(Grid{1}, Grid{2}, Grid{3}); 
    
    %% visualize measurements   
    viz = Viz3DGPGraphReal(params);
    viz.groundTruthModel(TriObj, XObj); 
    viz.measurements();
    drawnow;
    
    params
    
    if strcmp(params.training, 'batch')
        opt = optimizeGraphBatch(params, object_depth_map);
    elseif strcmp(params.training, 'incremental')
        opt = optimizeGraphIncremental(params);
    end
    
    if ~params.debugFlag
        shapeFolder = fullfile(gpisRootPath, 'results', params.shape);
        exportString = sprintf('real_k=%s_t=%s_g=%s', params.kernelType,params.training, params.genType); 
        saveFolder = fullfile(shapeFolder, exportString); 
        if ~exist(saveFolder, 'dir')
            mkdir(saveFolder);
        end
    end
    
    updateTimes = zeros(1, numMeas + 1); queryTimes = zeros(1, numMeas + 1);
    tic;
    opt.UpdateDepth(object_depth_map);
    updateTimes(1) = toc;
    fprintf('Depth prior update time: %.04f secs\n', updateTimes(1));

    %% query grid
    tic;
    [fMean, fVar] = opt.Query();   
    queryTimes(1) = toc;
    fprintf('Depth prior query time: %.04f secs\n', queryTimes(1));
    
    %% Bottom points 
    opt.UpdateDepth(bottomPts);
    %% query grid
    [fMean, fVar] = opt.Query();   
    
    %% get isosurface
    iso = isosurface(xt,yt,zt,fMean, 0);
    iso = cleanUp([object_depth_map.Location; bottomPts.Location], iso, params.cleanRadius);
    [iso, ~] = splitFV(iso);

    viz.addDepthMeasurements(object_depth_map.Location);
    viz.plotMesh(iso, {xt, yt, zt}, fMean, fVar);
    viz.plotSDF(iso, Grid, {xt, yt, zt}, fMean);
    drawnow;

     if ~params.debugFlag
        fprintf('Saving images...\n');
        viz.exportAll(saveFolder, 0);
        exportIso(saveFolder, iso, 0); 
    end

    if strcmp(params.training, 'batch')     % run batch       
        %% update with all measurements
        X = []; Y = [];
        for i = 1:length(patches)
            X = [X; patches(i).X];
            Y = [Y; patches(i).Y];
        end

        viz.addSensorMeasurements(X, Y); drawnow;
        
        %% update step
        tic;
        opt.Update(X, Y);
        updateTime = toc;  
            
        %% query grid
        tic;
        [fMean, fVar] = opt.Query();   
        queryTime = toc;
        fprintf('Update time: %.04f secs, Query time: %.04f secs\n', updateTime, queryTime);
        
        %% get isosurface
        iso = isosurface(xt,yt,zt,fMean, 0);
        iso = cleanUp([opt.X; object_depth_map.Location; bottomPts.Location], iso, params.cleanRadius);
        [iso, ~] = splitFV(iso);
       
        viz.plotMesh(iso, {xt, yt, zt}, fMean, fVar);
        viz.plotSDF(iso, Grid, {xt, yt, zt}, fMean);
        drawnow;
    else  % run incremental        
        for i = 1:length(patches)
            x = patches(i).X; y = patches(i).Y; 
            fprintf("Update size: %d\n", length(x));

            %% update step
            tic;
            opt.Update(x, y);
            updateTimes(i + 1) = toc;

            %% query step
            tic;
            [fMean, fVar] = opt.Query();   
            queryTimes(i + 1) = toc;
            fprintf('#%.d Update time: %.04f secs, Query time: %.04f secs\n', i, updateTimes(i + 1), queryTimes(i + 1));

            %% get isosurface
            iso = isosurface(xt,yt,zt,fMean, 0);
            iso = cleanUp([opt.X; object_depth_map.Location; bottomPts.Location], iso, params.cleanRadius);
            [iso, ~] = splitFV(iso);
                        
            %% plot results 
            viz.addSensorMeasurements(x, y, sensorPoses{i});
            viz.plotMesh(iso, {xt, yt, zt}, fMean, fVar);
            viz.plotSDF(iso, Grid, {xt, yt, zt}, fMean);
            drawnow;
            
            if (~params.debugFlag)
                fprintf('Saving images...\n');
                viz.exportAll(saveFolder, i); 
                exportIso(saveFolder, iso, i); 
            end
        end
        
        if ~params.debugFlag
            save(fullfile(saveFolder,'timing.mat'), 'updateTimes', 'queryTimes');
        end
    end 
end

function exportIso(saveFolder, iso, idx)
    stlFolder = fullfile(saveFolder, 'stl'); 
    if ~exist(stlFolder, 'dir')
        mkdir(stlFolder);
    end
    stlwrite(fullfile(stlFolder,sprintf('%s.stl', num2str(idx))), iso);
end

%% convert heightmap to 3-D pointcloud in world frame
function world_point_cloud = heightMap2Cloud(heightmap, normalmap, contactmask, sensorPose)
    %% gelsight parameters
    pixmm = 0.0295; % heightmap (pix) -> 3D conversion (mm)
    max_depth = 1.0; %mm
    max_z = max_depth/pixmm; % pix
    
    % ground truth heightmap
    heightmap = -1 * (heightmap - max_z); % invert gelsight heightmap
    heightmapValid = heightmap .* contactmask; % apply contact mask
    [x,y] = meshgrid((1:size(heightmapValid,2)) - size(heightmapValid,2)/2,...
                     (1:size(heightmapValid,1)) - size(heightmapValid,1)/2);
    heightmap_3d = [x(:), y(:), heightmapValid(:)];
    
    normalmap = ((sensorPose.SO3) * normalmap')';

    normalmap(heightmap_3d(:,3) == 0, :) = [];
    heightmap_3d(heightmap_3d(:,3) == 0, :) = []; 
    local_point_cloud = pointCloud(heightmap_3d*pixmm/1000); % heightmap (pix) -> 3D conversion (m)

    world_point_cloud = pointCloud((sensorPose * local_point_cloud.Location')');     % convert to global cooridnates via sensorPose
    world_point_cloud.Normal = normalmap; 
end

%% quat to SE3 conversion
function p = toPose(xyzquat)
    R = quat2rotm(xyzquat([7, 4, 5, 6])); 
    t = xyzquat(1:3);
    p = SE3(R, t);
end
