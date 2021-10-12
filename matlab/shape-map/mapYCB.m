%% batch script to map YCB objects (both sim and real)

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

objectList = ["002_master_chef_can","004_sugar_box", "005_tomato_soup_can",...
              "010_potted_meat_can", "021_bleach_cleanser", "036_wood_block"];

flag = "sim"; 
debug = true;

for obj = objectList
    fprintf('\n');
    if (strcmp(flag, "sim"))
        MapGelSight('shape', obj, 'debugFlag', debug, 'MeshN', 16, 'dim', 3, 'kernelType', 'thinplate',...
            'poseNoise', [1e-3*ones(1, 3), 1e-1*ones(1, 3)], 'measNoise', [6e-4, 1e-3, 1e-3, 1e-3], 'depthNoise', [1e-3, 1e-3, 1e-3, 1e-3],...
            'compact', true, 'depthRes', 5e-3, 'measRes', 2e-3,...
            'training', 'incremental', 'genType', 'fcrn', 'elev', 30);
    else
        MapGelSightReal('shape', obj, 'debugFlag', debug, 'MeshN', 16, 'dim', 3, 'kernelType', 'thinplate',...
            'poseNoise', [1e-3*ones(1, 3), 1e-1*ones(1, 3)], 'measNoise', [6e-4, 1e-3, 1e-3, 1e-3], 'depthNoise', [1e-3, 1e-3, 1e-3, 1e-3],...
            'compact', true, 'depthRes', 3e-3, 'measRes', 1e-3,...
            'training', 'incremental', 'genType', 'fcrn', 'elev', 30);       
    end
    pause(1); 
end
            