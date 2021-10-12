%% Test script, visualize heightmaps and contact masks

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

clc; clear; close all;

% load gelsight data 
load('/home/suddhu/projects/Shape_Mapping_Tactile/processed_data/mustard_bottle/mustard_bottle_gelsight.mat');
% load the model and sample points 
load('/home/suddhu/projects/Shape_Mapping_Tactile/data/mustard_bottle/mustard_bottle_50sampled.mat');
elev = 30;

N = size(heightmaps,2); 

figure('Name', 'Gelsight output'); 

s1 = subplot(2, 2, [1 2]);
hold on;
trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), 'EdgeColor', 'none', 'FaceAlpha', 0.7);
axis equal off
shading interp
camlight; lighting phong
colormap(s1,gray);
view(-37.5, elev);
xlim manual; ylim manual; zlim manual;

for i = 1:N    
    subplot(2, 2, [1 2]);
    poses(i, [4, 5, 6, 7]) = poses(i, [7, 4, 5, 6]);        % x y z w -> w x y z
    sensorPose = SE3(quat2rotm(poses(i, 4:end))', poses(i, 1:3)); 
    sensorPose.plot('rgb', 'length', 0.03, 'labels', '   ', 'thick', 1);
    plot3(samplePoints(i, 1), samplePoints(i, 2), samplePoints(i, 3), 'yo');
    
    heightmap = reshape(heightmaps(:, i),[480,640]);
    s2 = subplot(2,2,3);
    imshow(heightmap, []); 
    title(strcat('Heightmap #', num2str(i)))
    colormap(s2, jet);

    contactmask = reshape(contactmasks(:, i),[480,640]);
    s3 = subplot(2,2,4);
    imshow(contactmask, []); 
    title(strcat('Contactmask #', num2str(i)))
    pause(1);
end