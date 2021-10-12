%% slice3D.m:  slice across the SDF volume
function slice3D(x, y, z, val, xyzslice)
    % cross-sections
    xslice = xyzslice(1); yslice = xyzslice(2); zslice = xyzslice(3);
    s = slice(x, y, z, val, xslice, yslice, zslice);    % slice it!
    alpha(s, 0.5);
    set(s,'linewidth',1);
    caxis([0, 0.1]);
end

