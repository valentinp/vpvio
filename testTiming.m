%The learning pipeline
numSubsets = 1;
%[clusterWeights] = VIOPipelineV2_LearnClusters(K, T_camimu, monoImageData, rgbImageData, imuData, pipelineOptions, noiseParams, xInit, g_w, clusteringModel, T_wIMU_GT);
[predVectorSpace] = VIOPipelineV2_LearnRandomSubsets(K, T_camimu, monoImageData, rgbImageData, imuData, pipelineOptions, noiseParams, xInit, g_w, numSubsets, T_wIMU_GT);
