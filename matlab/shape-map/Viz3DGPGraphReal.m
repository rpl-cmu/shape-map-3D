%% Visualizer, 3-D GPIS for real data

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

classdef Viz3DGPGraphReal < handle   
    properties
        f; % figure handle
        s1; s2; s3; s4; % subplot handles
        limits;
        meshLim; 
        elev;
        count;
        xMesh; yMesh; zMesh;
        model_name;
    end
    
    methods
        function obj = Viz3DGPGraphReal(params)
            % define figure handle
            obj.f = figure('units','normalized','outerposition',[0 0 1 1]);
            set(gcf,'color','w');
            set(gca, 'FontName', 'Times New Roman');      
            obj.meshLim = params.meshLim; 
            Grid = {linspace(params.meshLim(1), params.meshLim(2), params.MeshN),...
                    linspace(params.meshLim(1), params.meshLim(2), params.MeshN),...
                    linspace(params.meshLim(1), params.meshLim(2), params.MeshN)};
            [obj.xMesh, obj.yMesh, obj.zMesh] = meshgrid(Grid{1}, Grid{2}, Grid{3}); 
            obj.count = 1;
            obj.elev = params.elev;
            obj.model_name = params.shape;
        end
        
        %% plot original model 
        function groundTruthModel(obj, TriObj, XObj)
            set(0,'CurrentFigure',obj.f); 
            obj.s1 = subplot(2, 2, 1);
            
            hold on;
            trisurf(TriObj, XObj(:,1), XObj(:,2), XObj(:,3), 'EdgeColor', 'none');
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)])
            axis equal off
            shading interp 
            camlight; lighting phong 
%             colormap(gca,gray)
            view(130, obj.elev);
            hold off; 
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
        end
        
        %% plot measurements and isosurface
        function measurements(obj)
            set(0,'CurrentFigure',obj.f); 
            obj.s2 = subplot(2, 2, 2);  

            axis equal off
            shading interp 
            view(130, obj.elev);
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
            hold off;
        end
        
        function addDepthMeasurements(obj, x)
            set(0,'CurrentFigure',obj.f); 
            obj.s2 = subplot(2, 2, 2);  
            hold on;
            plotPointsAndNormals3D(x, [], 1e-2, 'b'); 
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
            hold off;
        end

        function addSensorMeasurements(obj, x, y, sensorPose)
            set(0,'CurrentFigure',obj.f); 
            obj.s2 = subplot(2, 2, 2);  
            hold on;
            plotPointsAndNormals3D(x, y, 1e-2, 'g'); 
            
            if nargin > 3
                sensorPose.plot('rgb', 'length', 0.005, 'labels', '   ', 'thick', 0.5);
            end
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
            hold off;
        end
        
        function plotMesh(obj, iso, xyz, M, V)
            set(0,'CurrentFigure',obj.f); 
            obj.s3 = subplot(2, 2, 3);
            cla();
                   
            psurf1=patch(iso, 'EdgeColor','none');
            hold on;
            isonormals(xyz{1},xyz{2},xyz{3},M,psurf1);
            isocolors(xyz{1},xyz{2},xyz{3},V,psurf1);
            % trisurf(iso.faces, iso.vertices(:,1), iso.vertices(:,2), iso.vertices(:,3), 'EdgeColor', 'none');
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)])
            axis equal off
            camlight;    lighting gouraud;
            shading interp;
%             customMap = jet;
%             customMap(end, :) = ones(1, 3);
%             colormap(gca,customMap);
            colormap(gca,jet);

            set(psurf1,'linestyle','none');
            
            %% hardcoded for Sep 10th
            if strcmp(obj.model_name, "002_master_chef_can")
                caxis(obj.s3, [1e-7 3e-5]); % 002_master_chef_can
            elseif strcmp(obj.model_name, "004_sugar_box")
                caxis(obj.s3, [1e-7 3e-5]); % 004_sugar_box
            elseif strcmp(obj.model_name, "005_tomato_soup_can")
                caxis(obj.s3, [1e-7 3e-5]); % 005_tomato_soup_can
            elseif strcmp(obj.model_name, "010_potted_meat_can")
                caxis(obj.s3, [1e-7 3e-5]); % 010_potted_meat_can
            elseif strcmp(obj.model_name, "021_bleach_cleanser")
                caxis(obj.s3, [1e-7 3e-5]); % 021_bleach_cleanser
            elseif strcmp(obj.model_name, "036_wood_block")
                caxis(obj.s3, [1e-7 3e-5]); % 036_wood_block
            else
                v = V(:); v(v == max(v)) = [];
                q = quantile(v,[0 0.8]);
                caxis(obj.s3, [q(1), q(2)]);
            end
            
            view(130, obj.elev);
            psurf1.FaceAlpha = 0.85;
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
            hold off;
        end
        
        %% plot isosurface, measurements and SDF
        function plotSDF(obj, iso, grid, xyz, M)
            set(0,'CurrentFigure',obj.f); 
            obj.s4 = subplot(2, 2, 4);
            cla();
            
            slice = [mean(grid{1}), mean(grid{2}), mean(grid{3})];
            slice3D(xyz{1}, xyz{2}, xyz{3}, M, slice);
            hold on;
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)])
            axis equal off
            camlight;    lighting gouraud;
            colormap(gca,flipud(jet));
%             q = quantile(M(:),[0.01 0.25]);
            caxis([0, 1e-2]);
%             caxis(obj.s4, [q(1), q(2)]);
            
            trisurf(iso.faces, iso.vertices(:,1), iso.vertices(:,2), iso.vertices(:,3),...
                    'EdgeColor', 'None', 'LineStyle', 'None',  'FaceColor', 'white');
            shading interp
            
            view(130, obj.elev);
            axis([obj.meshLim(1,:), obj.meshLim(2,:), obj.meshLim(3,:)]);
            hold off;
        end

        function save(obj)
            saveas(obj.f,sprintf('images/Fig_%d.png',obj.count));
            obj.count = obj.count + 1;
        end
        
    
        function exportAll(obj, saveFolder, idx)
            
            % ground truth model
            modelFolder = fullfile(saveFolder, 'model'); 
            if ~exist(modelFolder, 'dir')
                mkdir(modelFolder);
                fig = figure('visible', 'off');
                h = copyobj(obj.s1,fig);
                set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
                exportgraphics(h,fullfile(modelFolder,'model.png'), 'Resolution',500);
                close(fig); 
            end

            % measurements 
            measFolder = fullfile(saveFolder, 'meas'); 
            if ~exist(measFolder, 'dir')
                mkdir(measFolder);
            end
            fig = figure('visible', 'off');
            h = copyobj(obj.s2,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            exportgraphics(h,fullfile(measFolder,sprintf('%s.png', num2str(idx))), 'Resolution',500);
            close(fig); 

            % isosurface
            isoFolder = fullfile(saveFolder, 'iso'); 
            if ~exist(isoFolder, 'dir')
                mkdir(isoFolder);
            end
            fig = figure('visible', 'off');
            h = copyobj(obj.s3,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            exportgraphics(h,fullfile(isoFolder,sprintf('%s.png', num2str(idx))), 'Resolution',500);
            close(fig); 
 
            % SDF
            sdfFolder = fullfile(saveFolder, 'sdf'); 
            if ~exist(sdfFolder, 'dir')
                mkdir(sdfFolder);
            end
            fig = figure('visible', 'off');
            h = copyobj(obj.s4,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            exportgraphics(h,fullfile(sdfFolder,sprintf('%s.png', num2str(idx))), 'Resolution',500);
            close(fig); 
        end
    end
end

