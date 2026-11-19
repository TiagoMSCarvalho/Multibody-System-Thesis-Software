addpath('txt');

fid=fopen('Ypos_Damp_Only.txt');
datavec = fscanf(fid,'%e');
fclose(fid);