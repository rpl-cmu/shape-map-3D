%% GP-SG incremental optimizer

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

classdef optimizeGraphIncremental < handle
    properties 
        %% query points
        queryPoints;
        numNodes;  meshLength;
        numMeasurements;
        numDepthPrior;

        inputDim; outputDim;

        %% parameters
        noiseModels = struct('meas', [], 'prior', []);
        compact = struct('isCompact', [], 'compRadius', []);
        
        theta;
        compactSet;
        
        %% solver 
        solver = struct('useGaussNewton', [], 'optParams', [], 'optimizeStopRelErr', [], 'isam', []);    
        Graph; initValues; allValues;
                
        mu; sigma;
        X; Y;
    end
    
    methods
        %% constructor
        function obj = optimizeGraphIncremental(params)
            obj.inputDim = params.dim;
            obj.outputDim = obj.inputDim + 1;
            %% get query grid            
            if (obj.inputDim == 2)
                Grid = {linspace(params.meshLim(1, 1), params.meshLim(1, 2), params.MeshN),...
                linspace(params.meshLim(2, 1), params.meshLim(2, 2), params.MeshN)};
                [xt, yt] = meshgrid(Grid{1}, Grid{2});     
                obj.queryPoints = [xt(:), yt(:)];
                obj.numNodes = params.MeshN^2;      
            elseif (obj.inputDim == 3)
                Grid = {linspace(params.meshLim(1, 1), params.meshLim(1, 2), params.MeshN),...
                linspace(params.meshLim(2, 1), params.meshLim(2, 2), params.MeshN)...
                linspace(params.meshLim(3, 1), params.meshLim(3, 2), params.MeshN)};
                [xt, yt, zt] = meshgrid(Grid{1}, Grid{2}, Grid{3});     
                obj.queryPoints = [xt(:), yt(:), zt(:)];
                obj.numNodes = params.MeshN^3; 
            end
            
            obj.meshLength = params.MeshN; 
            obj.numMeasurements = 0;
            obj.theta = params.theta';
                           
            %% isam2 solver object
            isamParams = gtsam.ISAM2Params;
            obj.solver.isam = gtsam.ISAM2(isamParams);
            obj.Graph = gtsam.NonlinearFactorGraph;
            obj.initValues = gtsam.Values;
            obj.allValues = gtsam.Values;
            
            %% noiseModels
            obj.noiseModels.meas = params.measNoise';
            obj.noiseModels.depth = params.depthNoise';
            obj.noiseModels.prior = gtsam.noiseModel.Diagonal.Sigmas([1e-1; ones(obj.inputDim, 1) * 1e-1]);
            
            %% compact stuff 
            obj.compact.isCompact = params.compact;
            obj.compact.compRadius = params.compRadius;    
                                    
            %% build prior graph 
            for idx = 1:obj.numNodes
                obj.initValues.insert(gtsam.symbol('y', idx), [1e-2, zeros(obj.inputDim, 1)']'); % init variables
                obj.Graph.add(gtsam.PriorFactorVector(gtsam.symbol('y', idx), [1e-2, zeros(obj.inputDim, 1)']', obj.noiseModels.prior))
            end
            
            fprintf("Adding %d factors\n", obj.Graph.size);
            obj.mu = zeros(obj.numNodes, obj.outputDim);
            obj.sigma = 1e-1 * ones(obj.numNodes, 1);
            obj.X = [];
        end
        
        function UpdateDepth(obj, depth_map)
            depthPrior = depth_map.Location;
            normalPrior = depth_map.Normal;

            obj.numDepthPrior = size(depthPrior, 1);
            fprintf("Intializing with %d depth points\n", obj.numDepthPrior);  

            for idx = 1:obj.numNodes
                dist2Measurements = pdist2(obj.queryPoints(idx, :), depthPrior);
                % get elements of compact set (all if params.compact = false)
                obj.compactSet = find((dist2Measurements < obj.compact.compRadius) | repmat(~obj.compact.isCompact, 1, obj.numDepthPrior)); 
                for cs = obj.compactSet
                    % add GP spatial factors 
                    y_aug = reshape([zeros(size(depthPrior(cs, :),1), 1)' ; normalPrior(cs, :)'], [], 1);
                    if (obj.inputDim == 2)
                        obj.Graph.add(gpgraph.GaussianProcessSpatialPrior2(gtsam.symbol('y', idx), obj.queryPoints(idx, :),...
                                      depthPrior(cs, :), y_aug, obj.theta, obj.noiseModels.depth));
                    else
                        obj.Graph.add(gpgraph.GaussianProcessSpatialPrior3(gtsam.symbol('y', idx), obj.queryPoints(idx, :),...
                                      depthPrior(cs, :), y_aug, obj.theta, obj.noiseModels.depth));
                    end
                end
            end
        end

        function [nodeEdges] = Update(obj, x, y)   
            numUpdate = size(x, 1);
            obj.numMeasurements = obj.numMeasurements + 1;

            obj.X = [obj.X; x];
            
            dist2Measurements = pdist2(obj.queryPoints, x);
            % get elements of compact set (all if params.compact = false)
            [r, c] = find((dist2Measurements < obj.compact.compRadius) | repmat(~obj.compact.isCompact, obj.numNodes, numUpdate)); 
            obj.compactSet = [ r, c];
            nodeEdges = zeros(size(obj.compactSet, 1), 2);
            
            count = 1;
            tic; 
            for cs = obj.compactSet'
                % add GP spatial factors 
                y_aug = reshape([0 ; y(cs(2), :)'], [], 1); 
                if (obj.inputDim == 2)
                    obj.Graph.add(gpgraph.GaussianProcessSpatialPrior2(gtsam.symbol('y', cs(1)),...
                        obj.queryPoints(cs(1), :), x(cs(2), :), y_aug, obj.theta, obj.noiseModels.meas));
                else
                    obj.Graph.add(gpgraph.GaussianProcessSpatialPrior3(gtsam.symbol('y', cs(1)),...
                        obj.queryPoints(cs(1), :), x(cs(2), :), y_aug, obj.theta, obj.noiseModels.meas));
                end
                nodeEdges(count, :) = [cs(1), cs(2)];
                count = count + 1;
            end
             
            fprintf("%f secs, Adding %d factors\n", toc, obj.Graph.size);
            obj.solver.isam.update(obj.Graph, obj.initValues);
            obj.Graph = gtsam.NonlinearFactorGraph;
            obj.initValues = gtsam.Values;      
        end

        function [fMean, fVar] = Query(obj)
            if obj.numMeasurements > 0
                set = obj.compactSet';
            else 
                obj.solver.isam.update(obj.Graph, obj.initValues);
                obj.Graph = gtsam.NonlinearFactorGraph;
                obj.initValues = gtsam.Values; 
                set = 1:obj.numNodes;
            end

            %% retrieve only modified nodes 
            for idx = set
                temp = obj.solver.isam.calculateEstimateVector(gtsam.symbol('y', idx(1))); % mean 
                obj.mu(idx(1), :) = temp';
                mc = obj.solver.isam.marginalCovariance(gtsam.symbol('y', idx(1))); % cov
                obj.sigma(idx(1), 1) = mc(1, 1);
            end
            
            fMean = reshape(obj.mu(:, 1), repelem(obj.meshLength, obj.inputDim)); 
            fVar = reshape(obj.sigma(:, 1), repelem(obj.meshLength, obj.inputDim)); 
        end
    end
end