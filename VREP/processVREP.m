function [T_wIMU_GT, monoImageData, rgbImageData, imuData] = processVREP()

if exist('vrepData.mat', 'file') == 2
    loadedVars = load('vrepData.mat');
    VREPrgbImageData = loadedVars.VREPrgbImageData;
    VREPimuData = loadedVars.VREPimuData;
    VREPGroundTruth = loadedVars.VREPGroundTruth;
    timestamps = loadedVars.timestamps;
else
    [VREPrgbImageData, VREPimuData, VREPGroundTruth, timestamps] = getImageAndIMUData();
    save('vrepData.mat', 'VREPGroundTruth', 'VREPimuData', 'VREPrgbImageData', 'timestamps');
end


imuData = VREPimuData;
imuData.initialVelocity = [-0.1 0 0]';
imuData.timestamps = timestamps;

monoImageData.rectImages = [];
monoImageData.timestamps = timestamps - 1e-4;

rgbImageData.timestamps = timestamps - 1e-4;

%Put ground truth into the first frame
T_wIMU_GT(:,:,1) = eye(4);
for pose_i = 2:size(VREPGroundTruth,3)
    T_wIMU_GT(:,:,pose_i) = VREPGroundTruth(:,:,1)*inv(VREPGroundTruth(:,:, pose_i));
end

%Process grayscale data
for img_i = 1:size(VREPrgbImageData, 4)
    %Rotation is necessary because the projection VREP does is not in the
    %standard camera co-ordinate frame
    rgbImageData.rectImages(:,:,:, img_i) = imrotate(VREPrgbImageData(:,:,:,img_i)./255,180);
    monoImageData.rectImages(:,:, img_i) = 255*rgb2gray(rgbImageData.rectImages(:,:,:, img_i));
end

end