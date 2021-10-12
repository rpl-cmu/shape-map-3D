%% 2D GPIS via GP-SG with 2-D surface and normal measurements

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

function Map2DGPGraph(varargin)
    close all;
    gpisRootPath = getenv('GPIS_PATH');
    addpath('/usr/local/lib'); 
    addpath(fullfile(gpisRootPath, 'matlab', 'standalone','utils')) 

    %% hyperparameters   
    params = inputParser;
    addParameter(params,'debugFlag',true);
    addParameter(params,'MeshN',30);
    addParameter(params,'dim',2);
    addParameter(params,'kernelType','thinplate');
    addParameter(params,'measNoise',[1e-2, 1e-2]);
    addParameter(params,'inputNoise',[0, 0]);
    addParameter(params,'depthNoise',[5e-4, 1e-3, 1e-3]);
    addParameter(params,'compact',true);
    addParameter(params,'compRadius',0.3);
    addParameter(params,'training','batch');
    addParameter(params,'shape','bunny');
    addParameter(params,'fileID','20210331-2217');
    parse(params, varargin{:});
    params = params.Results;
    
    % load data and ground truth (x, y, nx, ny)
    dataPath = strcat(fullfile(gpisRootPath, 'data', 'contacts'), '/contacts-', params.shape,'-', params.fileID, '.txt');
    shapePath = strcat(fullfile(gpisRootPath, 'data', 'shapes', params.shape, params.shape), '.mat');
    params.shape_data = struct2array(load(shapePath));

    data = importdata(dataPath);
    data = data(randperm(size(data, 1)), :);     % shuffle data
    X = data(:,1:2); Y = normr(data(:,3:4));
    
    % scale [0, 1] and normalize data
    params.shape_data = ScaleAndOffset(params.shape_data);
    X = ScaleAndOffset(X);
    Y = normr(Y);
    
    % add noise after offset and normalize
    [X, Y] = addNoise(X, Y, params.measNoise, params.inputNoise); 
    
    % load ground-truth
    if strcmp(params.kernelType, "thinplate")
        params.theta = [0, max(pdist(X,'euclidean')), 1]; 
    elseif strcmp(params.kernelType, "rbf")
        params.theta = [1, 0.25, 0.01]; 
    end

    fprintf('Training data points: %d, Query points: %d\n', size(X, 1), params.MeshN^2);
        
    % generate 2D mesh query points
    params.meshLim = [-0.1 1.1; -0.1 1.1];
    Grid = {linspace(params.meshLim(1, 1), params.meshLim(1, 2), params.MeshN),...
            linspace(params.meshLim(2, 1), params.meshLim(2, 2), params.MeshN)};
    [xt, yt] = meshgrid(Grid{1}, Grid{2}); 
    
    % visualize measurements
    viz = Viz2DGPGraph(params);
    viz.groundTruthModel(); 
    drawnow;
    
    params
    
    if strcmp(params.training, 'batch')
        opt = optimizeGraphBatch(params);
    else
        opt = optimizeGraphIncremental(params);
    end
            
    allNodeEdges = [];
    if strcmp(params.training, 'batch')     % run batch
        %% update with all measurements
        tic;
        opt.Update(X, Y);
        updateTime = toc;  

        %% query grid
        tic;
        [fMean, fVar] = opt.Query();   
        queryTime = toc;
        fprintf('Update time: %.04f secs, Query time: %.04f secs\n', updateTime, queryTime);

        %% get level-set and delete outliers
        C = contourc(Grid{1},Grid{2},fMean, [0 0]); 
        thresh = 1; idx = any(C > thresh,1); C(:,idx) = [];
    else  % run incremental 
        for i = 1:size(X, 1)
            x = X(i, :); y = Y(i, :); 
            %% update with new measurement
            tic;
            nodeEdges = opt.Update(x, y);
            allNodeEdges = [allNodeEdges; nodeEdges];
            updateTime = toc;

            %% query grid
            tic;
            [fMean, fVar] = opt.Query();   
            queryTime = toc;
            fprintf('#%.d Update time: %.04f secs, Query time: %.04f secs\n', i, updateTime, queryTime);

            %% get level-set and delete outliers
            C = contourc(Grid{1},Grid{2},fMean, [0 0]); 
            thresh = 1; idx = any(C > thresh,1); C(:,idx) = [];

            %% plot results 
            % viz.measurements(X(1:i, :), Y(1:i, :));
            % viz.levelSet(C, {xt, yt}, fMean, fVar, false);
            % viz.SDFMean(C, {xt, yt}, fMean, false);
            % viz.SDFVar(C, {xt, yt}, fVar, false);
            % drawnow;
        end
    end 

    viz.measurements(X, Y);
    viz.levelSet(C, {xt, yt}, fMean, fVar, true);
    viz.SDFMean(C, {xt, yt}, fMean, true);
    viz.SDFVar(C, {xt, yt}, fVar, true);
    drawnow;

    if ~params.debugFlag
        saveFolder = fullfile(gpisRootPath, 'results', [params.shape, '2D']);
        if ~exist(saveFolder, 'dir')
            mkdir(saveFolder);
        end
        viz.exportAll(saveFolder, params)
    end
end

