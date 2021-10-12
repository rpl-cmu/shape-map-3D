%% addNoise.m: adds noise to X and Y data 
function [X,Y] = addNoise(X,Y, mn, in)
    X = X + in.*randn(size(X));
    Y = Y + mn.*randn(size(Y));
    Y = normr(Y);
end

