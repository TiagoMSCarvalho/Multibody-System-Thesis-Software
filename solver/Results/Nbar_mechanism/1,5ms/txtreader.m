addpath('txt');

fid=fopen('b0yvel.txt');
datavec = fscanf(fid,'%e');
fclose(fid);