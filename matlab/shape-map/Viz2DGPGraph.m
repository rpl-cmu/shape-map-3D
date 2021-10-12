%% Visualizer, 2-D GPIS

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

classdef Viz2DGPGraph < handle
    properties
        f; % figure handle
        s1; s2; s3; s4, s5, s6; % subplot handles
        meshLim; 
        elev;
        count;
        gt;
        xMesh; 
        yMesh;
    end
    
    methods
        function obj = Viz2DGPGraph(params)
            % define figure handle
            obj.f = figure('units','normalized','outerposition',[0 0 1 1]);
            set(gca, 'FontName', 'Times New Roman');           
            obj.meshLim = params.meshLim; 
            obj.gt = params.shape_data;
            Grid = {linspace(params.meshLim(1), params.meshLim(2), params.MeshN),...
            linspace(params.meshLim(1), params.meshLim(2), params.MeshN)};
            [obj.xMesh, obj.yMesh] = meshgrid(Grid{1}, Grid{2}); 
            obj.count = 1;
        end
        
        %% plot original model 
        function groundTruthModel(obj)
            set(0,'CurrentFigure',obj.f); 
            obj.s1 = subplot(3, 2, 1);
            plot(obj.gt(:,1), obj.gt(:,2), 'k-.', 'LineWidth', 3);
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            axis equal off
        end
        
        %% plot measurements and isosurface
        function measurements(obj, X, Y)
            set(0,'CurrentFigure',obj.f); 
            obj.s2 = subplot(3, 2, 2);  
            cla();
            plot(obj.gt(:,1), obj.gt(:,2), 'k-.', 'LineWidth', 1);
            hold on; 
            plot(X(:,1),X(:,2),'o','color',[0 0.392157 0],'MarkerSize',4, 'MarkerFaceColor',[0 0.392157 0]);
            quiver(X(:,1),X(:,2),Y(:,1),Y(:,2), 0.3, 'color',[0 0.392157 0]);
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            axis equal off
            hold off; 
        end
        
        %% plot isosurface, measurements and SDF with mean 
        function SDFMean(obj, C, xy, M, drawC)
            set(0,'CurrentFigure',obj.f); 
            obj.s3 = subplot(3, 2, 3);
            cla();
            if drawC
                plot(C(1, 1:end), C(2, 1:end), 'k-', 'linewidth', 2); % object contour
            end
            hold on;
            axis equal off;
            s = surf(xy{1},xy{2},M - 0.5, 'FaceColor', 'interp'); % plot GP
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            colormap(gca,jet);
            q = quantile(M(:),[0.01 0.25]);
            shading interp         
            view(2);
            hold off;
        end
        
        %% plot isosurface, measurements and SDF with variance
        function SDFVar(obj, C, xy, V, drawC)
            set(0,'CurrentFigure',obj.f); 
            obj.s4 = subplot(3, 2, 4);
            cla();
            
            if drawC
                plot(C(1, 1:end), C(2, 1:end), 'k-', 'linewidth', 2); % object contour
            end
            hold on;
            axis equal off;
            s = surf(xy{1},xy{2},V - 0.5, 'FaceColor', 'interp'); % plot GP
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            colormap(gca,parula);
%             q = quantile(V(:),[0.01 0.1]);
%             caxis([q(1), q(2)]);
            shading interp         
            view(2);
            hold off;
        end
        
        function levelSet(obj, C, xy, M, V, drawC)
            set(0,'CurrentFigure',obj.f); 
            obj.s5 = subplot(3, 2, 5);
            cla();
            if drawC
                plot(C(1, 2:end), C(2, 2:end), 'g-', 'linewidth', 5); % object contour
            end
            hold on;
            axis equal off;
            s = surf(xy{1},xy{2},M, 'FaceColor', 'interp'); % plot GP
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            colormap(gca,parula);
            s.CData = V;
%             q = quantile(V(:),[0.01 0.5]);
%             caxis([q(1), q(2)]);
            set(gca, 'ZDir','reverse');
            zlim([min(M(:)) max(M(:))]);

            % plot 0 plane
            Z = zeros(size(xy{1}));
            CO(:,:,1) = zeros(size(xy{1},1));
            CO(:,:,2) = zeros(size(xy{1},1));
            CO(:,:,3) = zeros(size(xy{1},1)); 
            s = surf(xy{1},xy{2},Z, CO); 
            alpha(s,.7);
            view(3);
            hold off;
        end
        
        function graph(obj, X, xy, fs, M)
            set(0,'CurrentFigure',obj.f); 
            obj.s6 = subplot(3, 2, 6);   
            
            xt = xy{1}; yt = xy{2};
            QueryPts = [xt(:), yt(:)];
            % connectivity 
         
            %% plot variance
            z = -2*ones(size(xy{1}));
            s = surf(xy{1},xy{2}, z, M, 'FaceColor', 'interp', 'FaceAlpha',0.1); % plot GP
            s.EdgeColor = 'none';
            axis([obj.meshLim(1, :), obj.meshLim(2, :)]);
            axis equal off
            colormap(gca,jet);
            view(2);
            hold on; 
            
            %% plot graph connections 
            linesX = []; linesY = [];
            for i=1:size(fs, 1)
                qp = [QueryPts(fs(i, 1), 1), QueryPts(fs(i, 1), 2)];
                mp = X(fs(i, 2),:);
                linesX(end + 1, :) = [qp(1),mp(1)]; 
                linesY(end + 1, :) = [qp(2),mp(2)]; 
            end

            z = -1*ones(size(linesY));
            connPlot = plot3(linesX', linesY', z', 'b-', 'LineWidth', 0.5);

            %% plot ground truth contour
            gtPlot = plot(obj.gt(:,1), obj.gt(:,2), 'k--', 'LineWidth', 0.5);
            %% plot query points
            queryPlot = plot(QueryPts(:, 1), QueryPts(:, 2), 'o', 'color','k','MarkerSize',2, 'MarkerFaceColor','r');
            %% plot measurement points 
            measurementPlot = plot(X(:,1),X(:,2),'s','color','k','MarkerSize',4, 'MarkerFaceColor',[0 0.392157 0]);
        end
    
        function save(obj)
            saveas(obj.f,sprintf('images/Fig_%d.png',obj.count));
            obj.count = obj.count + 1;
        end

        function exportAll(obj, saveFolder, params)
            %% export           
            if params.compact
                exportString = sprintf('%s_c=%s_%s_%s', params.kernelType, string(params.compact),...
                                string(params.compactRadius), params.training); 
            else
                exportString = sprintf('%s_c=%s_%s', params.kernelType, string(params.compact),...
                                params.training);
            end
            
            % ground truth model 
            fig = figure('visible', 'off');
            h = copyobj(obj.s1,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString, 'model')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString, 'model'), 'Resolution',1000);
            close(fig); 

            % measurements 
            fig = figure('visible', 'off');
            h = copyobj(obj.s2,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString, 'measurements')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString,  'measurements'), 'Resolution',1000);
            close(fig); 

            % SDF mean 
            fig = figure('visible', 'off');
            h = copyobj(obj.s3,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString,  'sdfmean')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString, 'sdfmean'), 'Resolution',1000);
            close(fig); 
 
            % SDF var 
            fig = figure('visible', 'off');
            h = copyobj(obj.s4,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString,  'sdfvar')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString, 'sdfvar'), 'Resolution',1000);
            close(fig); 
            
            % 3d level-set
            fig = figure('visible', 'off');
            h = copyobj(obj.s5,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString, 'levelset')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString, 'levelset'), 'Resolution',1000);
            close(fig); 

            % graph
            fig = figure('visible', 'off');
            h = copyobj(obj.s6,fig);
            set(h, 'pos', [0.23162 0.2233 0.72058 0.63107]);
            disp(['Saving ', sprintf('%s/%s_%s.png', saveFolder, exportString,  'graph')]);
            exportgraphics(h,sprintf('%s/%s_%s.png', saveFolder, exportString, 'graph'), 'Resolution',1000);
            close(fig); 
        end
    end
end

