%% 3-D shape mapping with GelSight and Depth camera, simulation

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

function MapGelSight(varargin)
    close all;
    gpisRootPath = getenv('GPIS_PATH');
    addpath('/usr/local/lib'); 
    addpath(genpath(fullfile(gpisRootPath, 'matlab')));
   
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
    addParameter(params,'depthRes',5e-3);
    addParameter(params,'measRes',2e-3);
    addParameter(params,'training','incremental');
    addParameter(params,'shape','010_potted_meat_can');
    addParameter(params,'genType','fcrn');
    addParameter(params,'elev',30);
    parse(params, varargin{:});
    params = params.Results;
    
    %% load pointcloud data: x, y, z, nx, ny, nz
    datasetRoot = '/home/suddhu/projects/shape-map/Shape_Mapping_Tactile/';
    datasetName = 'textured_60sampled';
    
    gelsightFile = fullfile(datasetRoot, 'gelsight_data', datasetName, params.shape, 'gelsight_data.mat'); load(gelsightFile);
    if ~strcmp(params.genType, 'gt')
        genFile = fullfile(datasetRoot, 'generated_data', datasetName, params.genType, params.shape, strcat(params.genType, '_data.mat')); load(genFile);
    end
    modelFile = fullfile(datasetRoot, 'models', params.shape, 'textured.mat'); load(modelFile);
    XObj = vertices; YObj = normals; YObj = normr(YObj); TriObj = faces;
    numMeas = length(poses); 
    
    %% extract patches from point cloud 
    fprintf('Extracting tactile + vision data\n');
    patches = [];
    sensorPoses = {};
    for i = 1:numMeas
        poses(i, [4, 5, 6, 7]) = poses(i, [7, 4, 5, 6]);        % x y z w -> w x y z
        sensorPose = SE3(quat2rotm(poses(i, 4:end)), poses(i, 1:3));
        
        % add noise to sensor poses 
        if ~strcmp(params.genType, 'gt')
            noise = params.poseNoise.*randn(size(params.poseNoise));
            noiseSE3 = SE3(eul2rotm(noise(4:end)), noise(1:3));
            sensorPose = sensorPose * noiseSE3;
        end
        sensorPoses{i} = sensorPose; 
        
        % get heightmaps and contactmasks
        if strcmp(params.genType, 'gt')
            heightmap = reshape(gt_heightmaps(:, i),[480,640]);
            normalmap = reshape(gt_normalmaps(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480*640, 3]);
            contactmask = reshape(gt_contactmasks(:, i),[480,640]);
        elseif strcmp(params.genType, 'lookup')
            heightmap = reshape(lookup_heightmaps(:, i),[480,640]);
            normalmap = reshape(lookup_normalmaps(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480*640, 3]);  
            contactmask = reshape(est_contactmasks(:, i),[480,640]);
        elseif strcmp(params.genType, 'fcrn')
            heightmap = reshape(fcrn_heightmaps(:, i),[480,640]);
            normalmap = reshape(fcrn_normalmaps(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480*640, 3]);  
            contactmask = reshape(est_contactmasks(:, i),[480,640]);
        end
        
        %% get tactile images
        tactileimage = reshape(tactileimages(:, 3*(i-1) + 1 : 3*(i-1) + 3),[480,640, 3]);
        tactileimage = rescale(tactileimage); 
        normalmap = ((sensorPose.SO3) * normalmap')';
       
        world_point_cloud = heightMap2Cloud(heightmap, normalmap, contactmask, sensorPose);
        world_point_cloud = pcdownsample(world_point_cloud,'gridAverage', params.measRes);

        patches(end + 1).X = world_point_cloud.Location;
        patches(end).Y = world_point_cloud.Normal;
        
        % add noise after offset and normalize
        [patches(i).X, patches(i).Y] = addNoise(patches(i).X, patches(i).Y, params.measNoise(2:end), params.inputNoise); 
    end
    
    %% vision measurements
    depth_map = pointCloud(depth_map);
    origin = mean(XObj);    
    depth_map.Normal = depth_map.Location - origin;
    depth_map.Normal = (depth_map.Normal)./vecnorm(depth_map.Normal, 2, 2);
    depth_map = pcdownsample(depth_map,'gridAverage',params.depthRes);
    
    %% kernel parameters
    if strcmp(params.kernelType, "thinplate")
        params('theta') = [0, 0.25 , 1]; 
    elseif strcmp(params.kernelType, "rbf")
        params('theta') = [1, 0.25, 0.01]; 
    end
    
    fprintf('Training data points: %d, Query points: %d\n', numMeas, params.MeshN^3);        
    %% generate 3D mesh query points
    ml = [      min(XObj(:, 1)) max(XObj(:, 1));
                min(XObj(:, 2)) max(XObj(:, 2));
                min(XObj(:, 3)) max(XObj(:, 3))] + [-2e-2 2e-2];
                  
    params.compRadius = 0.15*mean(ml(:, 2) - ml(:,1));
    params.cleanRadius =  params.compRadius;

    Grid = {linspace(ml(1, 1), ml(1, 2), params.MeshN),...
            linspace(ml(2, 1), ml(2, 2), params.MeshN)...
            linspace(ml(3, 1), ml(3, 2), params.MeshN)};
    [xt, yt, zt] = meshgrid(Grid{1}, Grid{2}, Grid{3}); 
    
    params('meshLim') = ml; 
    %% visualize measurements   
    viz = Viz3DGPGraph(params);
    viz.groundTruthModel(TriObj, XObj); 
    viz.measurements(TriObj, XObj);
    drawnow;
    
    params
    
    if strcmp(params.training, 'batch')
        opt = optimizeGraphBatch(params, depth_map);
    elseif strcmp(params.training, 'incremental')
        opt = optimizeGraphIncremental(params);
    end
    
    if ~params.debugFlag
        shapeFolder = fullfile(gpisRootPath, 'results', params.shape);
        exportString = sprintf('k=%s_t=%s_g=%s', params.kernelType,params.training, params.genType); 
        saveFolder = fullfile(shapeFolder, exportString); 
        if ~exist(saveFolder, 'dir')
            mkdir(saveFolder);
        end
    end
    
    updateTimes = zeros(1, numMeas + 1); queryTimes = zeros(1, numMeas + 1);
    tic;
    opt.UpdateDepth(depth_map);
    updateTimes(1) = toc;
    fprintf('Depth prior update time: %.04f secs\n', updateTimes(1));

    %% query grid
    tic;
    [fMean, fVar] = opt.Query();   
    queryTimes(1) = toc;
    fprintf('Depth prior query time: %.04f secs\n', queryTimes(1));

    %% get isosurface
    iso = isosurface(xt,yt,zt,fMean, 0);
    iso = cleanUp(depth_map.Location, iso, params.cleanRadius);
    [iso, ~] = splitFV(iso);

    viz.addDepthMeasurements(depth_map.Location);
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
        iso = cleanUp([opt.X; depth_map.Location], iso, params.cleanRadius);
        [iso, ~] = splitFV(iso);
       
        viz.plotMesh(iso, {xt, yt, zt}, fMean, fVar);
        viz.plotSDF(iso, Grid, {xt, yt, zt}, fMean);
        drawnow;
    else  % run incremental        
        for i = 1:length(patches)
            x = patches(i).X; y = patches(i).Y; 
            fprintf("Update size: %d\n", length(x));

            %% update with new measurement
            tic;
            opt.Update(x, y);
            updateTimes(i + 1) = toc;

            %% query grid
            tic;
            [fMean, fVar] = opt.Query();   
            queryTimes(i + 1) = toc;
            fprintf('#%.d Update time: %.04f secs, Query time: %.04f secs\n', i, updateTimes(i + 1), queryTimes(i + 1));

            %% get isosurface
            iso = isosurface(xt,yt,zt,fMean, 0);
            iso = cleanUp([opt.X; depth_map.Location], iso, params.cleanRadius);
            [iso, ~] = splitFV(iso);
                        
            %% plot results 
            viz.addSensorMeasurements(x, y, sensorPoses{i});
            viz.plotMesh(iso, {xt, yt, zt}, fMean, fVar);
            viz.plotSDF(iso, Grid, {xt, yt, zt}, fMean);
            drawnow;
            
            if ~params.debugFlag
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
    normalmap(heightmap_3d(:,3) == 0, :) = [];
    heightmap_3d(heightmap_3d(:,3) == 0, :) = []; 
    local_point_cloud = pointCloud(heightmap_3d*pixmm/1000); % heightmap (pix) -> 3D conversion (m)

    world_point_cloud = pointCloud((sensorPose * local_point_cloud.Location')');     % convert to global cooridnates via sensorPose
    world_point_cloud.Normal = normalmap; 
end
