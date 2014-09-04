function [ RMSE_RPE_est ] = calcRelativePoseError( T_gt, T_est )
%CALCRELATIVEPOSEERROR Calculates the RMS of the relative pose error for estimated poses (T_est) based on ground
%truth poses (T_gt). Output is the mean for all possible d-steps.

poseNum = size(T_est,3);

RMSE_RPE_est_list = zeros(1, poseNum-1);

%Iterative through different step sizes
for dstep = 1:(poseNum-1)

    RPE_est =  zeros(4,4, poseNum - dstep);

    for i = 1:(poseNum-dstep)
        RPE_est(:,:,i) = inv(inv(T_gt(:,:,i))*T_gt(:,:,i+dstep))*inv(T_est(:,:,i))*T_est(:,:,i+dstep); 
    end

    %Calculate the root mean squared error of all the relative pose errors
    RMSE_RPE_est = 0;

    for i = 1:size(RPE_est,3)
        RMSE_RPE_est = RMSE_RPE_est + norm(RPE_est(1:3,4,i))^2;
    end
    RMSE_RPE_est_list(dstep) = sqrt(RMSE_RPE_est/size(RPE_est,3));
end

RMSE_RPE_est = mean(RMSE_RPE_est_list);

end

