function matchIdx =  simMatchFeatures(features1, features2)
    %Match two arrays of feature ids (the match is simply an equality)
    %Output: matchIdx, Nx2 matrix where each row contains a match of features indicated by indeces
    %from features1 and features 2 (in the respective columns).
    
  features1 = features1(:);
  features2 = features2(:);
  features1 = features1';
  features2 = features2';
    
    %Find all 1 features that are in 2 and return an array of indeces
    [~, matched2idx] = ismember(features1,features2);
    
    %Extract non zero entries
    [~,matched1idx] = find(matched2idx);
    matched2idx = matched2idx(matched2idx>0);
    
    %Ensure column vectors
    matched1idx = matched1idx(:);
    matched2idx = matched2idx(:);
    
    matchIdx = [matched1idx matched2idx];
end