function [T_wc_list, landmarks_w] =  importG2ODataExpMap(fileName)
%IMPORTG2ODATA Imports poses and landmarks from a file output by g2o

landmarks_w = [];
T_wc_list = [];

fid = fopen(fileName, 'r');

tline = fgetl(fid);
while ischar(tline)
   %linePieces = strsplit(tline, ' ');
   if length(tline) < 10
                    tline = fgetl(fid);

       continue;
   end
   elementType = tline(1:10);
   
   if strcmp(elementType, 'VERTEX_XYZ')
       linePieces = sscanf(tline, '%*s %d %f %f %f');
       landmark = linePieces(2:4);
       landmarks_w(:, end+1) = landmark(:);
   elseif strcmp(elementType, 'VERTEX_SE3')
       linePieces = sscanf(tline, '%*s %d %f %f %f %f %f %f %f');
       transform = linePieces(2:8);
       t_kw_w = transform(1:3);
       %Extract quaternion, ensure scalar is the first element
       q_wk = zeros(4,1);
       q_wk(2:4,1) = transform(4:6);
       q_wk(1,1) = transform(7);
       R_wk = rotmat_from_quat(q_wk);
       if isempty(T_wc_list)
          T_wc_list = [R_wk t_kw_w; 0 0 0 1];
       else
           T_wc_list(:,:,end+1) = [R_wk t_kw_w; 0 0 0 1];
       end
   end
             tline = fgetl(fid);
end
fclose(fid);

