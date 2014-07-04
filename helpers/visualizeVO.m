function visualizeVO (T_gt, T_vo, landmarks_w, suffix)
    addpath([getenv('HOME') '/Dropbox/UTIAS - MASc/Code/MATLAB/kinematics_toolbox/screws']);
    figure;
    %subplot(2,1,1);
%     drawframetraj(T_gt);
%     set(gca,'FontSize',16);
%     title(['Ground Truth' suffix]);
%     xlabel('X');
%     ylabel('Y');
%     zlabel('Z');
%     hold on;
%     view(13,12)
%     %xlim([min(T_gt(1, 4,:))-1 max(T_gt(1, 4,:))+1]);
%     plot3(landmarks_w(1,:),landmarks_w(2,:),landmarks_w(3,:), 'b*')
%  %   subplot(2,1,2);
%     figure;
    plot3(landmarks_w(1,:),landmarks_w(2,:),landmarks_w(3,:), 'r*')
    drawframetraj(T_vo);
    set(gca,'FontSize',16);
    title(['VIO Pipeline' suffix]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    hold on;
    %view(13,12)
    %xlim([min(T_vo(1, 4,:))-1 max(T_vo(1, 4,:))+1]);
end