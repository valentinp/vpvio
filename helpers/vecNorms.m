function [norms] = vecNorms(vectors)
%VECNORMS Returns the Euclidian norms of a matrix of column vectors

norms = sqrt(sum(vectors.^2, 1));

end

