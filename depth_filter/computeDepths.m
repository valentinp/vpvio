function [ depths ] = computeDepths( T_curr_ref, featVec_ref, featVec_curr)
%COMPUTEDEPTHS Compute depths in the reference frame based on new feature
%vectors and a transformation
    R_curr_ref = T_curr_ref(1:3, 1:3);
    p_refcurr_curr = T_curr_ref(1:3, 4);
    featNum = size(featVec_ref, 2);
    depths = zeros(1, featNum);
    for f_i = 1:featNum
        A = [R_curr_ref*featVec_ref(:,f_i) -1*featVec_curr(:,f_i)];
        temp = -inv(A'*A)*A'*p_refcurr_curr;
        depths(f_i) = abs(temp(1));
    end
end

