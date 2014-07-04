function [T_wc_list, landmarks_w] =  importG2ODataSSBA(fileName)
%IMPORTG2ODATA Imports poses and landmarks from a file output by g2o

landmarks_w = [];
T_wc_list = [];

fid = fopen(fileName, 'r');

tline = fgetl(fid);
landmarks_w = [];
    linePieces = sscanf(tline, '%d %d %d');
    skip = linePieces(1) + 1;   
tline = fgetl(fid);

while ischar(tline)
   %linePieces = strsplit(tline, ' ');
   
    
    if skip > 0
        skip = skip - 1;
        tline = fgetl(fid);
        continue;
    end
    

       linePieces = sscanf(tline, '%d %f %f %f %f %f %f %f %f %f %f %f %f');
       T_wk = [linePieces(2:5)';linePieces(6:9)'; linePieces(10:13)'];
       if isempty(T_wc_list)
          T_wc_list = inv([T_wk; 0 0 0 1]);
       else
           T_wc_list(:,:,end+1) = inv([T_wk; 0 0 0 1]);
       end
       
       tline = fgetl(fid);
end
fclose(fid);

