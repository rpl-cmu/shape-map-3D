%% ScaleAndOffset.m: scale and offset measurements
function [X] = ScaleAndOffset(X)
    Xmin = min(X); Xmax = max(X);
    scale = Xmax - Xmin;
    X = (X - Xmin)./scale;
end

