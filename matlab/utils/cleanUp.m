%% cleanUp.m: prune mesh away from points
function iso = cleanUp(X, iso, thresh)
    D = pdist2(X, iso.vertices,  'euclidean', 'Smallest', 1)';
    pruneVertices = find(mean(D,2) > thresh)'; 
    vertices = iso.vertices;
    vertices(pruneVertices,:) = []; % remove 

    [~, newIdx] = ismember(iso.vertices, vertices, 'rows');

    newFaces = iso.faces(all([iso.faces(:,1) ~= pruneVertices, iso.faces(:,2) ~= pruneVertices, iso.faces(:,3) ~= pruneVertices], 2),:);
    newFaces = newIdx(newFaces);
    iso.vertices = vertices; iso.faces = newFaces;
end

