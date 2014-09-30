function newFactors = addLandmarkObservations(oldFactors, kptId, observedLandmarks)
%ADDLANDMARKOBSERVATIONS Adds all landmark observations to GTSAM factors

ol = observedLandmarks([observedLandmarks(:).id] == kptId);


for obs_i = 1:size(ol.pixelObs,2)
    newFactors.add(GenericProjectionFactorCal3_S2(Point2(ol.pixelObs(:, obs_i)), mono_model_n_robust, ol.poseKeys(obs_i), kptId, K_GTSAM,  Pose3(inv(T_camimu))));
end


end

