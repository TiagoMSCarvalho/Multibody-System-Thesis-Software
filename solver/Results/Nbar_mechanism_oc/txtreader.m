addpath('txt');

fid=fopen('B0_xvel.txt');
datavec = fscanf(fid,'%e');
fclose(fid);