addpath('txt');

fid=fopen('Adams_time.txt');
datavec = fscanf(fid,'%e');
fclose(fid);