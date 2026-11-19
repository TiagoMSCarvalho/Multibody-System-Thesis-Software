addpath('txt');

fid=fopen('P3zpos.txt');
datavec = fscanf(fid,'%e');
fclose(fid);