function [ idx ] = findClosestTimestamp(t, list_t)
%FINDCLOSESTTIMESTAMP Find the index in list_t that most closely matches t
[~, idx] = min(abs(list_t - t));
end

