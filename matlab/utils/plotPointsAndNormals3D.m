%% plotPointsAndNormals3D.m: visualizes surface measurement and normal
function plotPointsAndNormals3D(X, Y, sc, c)  
    rc = [rand,rand,rand];
    plot3(X(:,1),X(:,2),X(:,3),'.','MarkerSize',1, 'Color', c, 'MarkerFaceColor', c);
    if ~isempty(Y)
        quiver3(X(:,1),X(:,2),X(:,3),sc*Y(:,1),sc*Y(:,2),sc*Y(:,3), 'Color', c);
    end
end

