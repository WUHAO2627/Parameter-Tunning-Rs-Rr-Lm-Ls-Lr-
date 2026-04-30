clc;
clear all;
%分三步运行，分别对应MotorModel_1、MotorModel_2、MotorModel_3这三个仿真，每个仿真运行前，
%先更改matlab运行路径至仿真文件所在的目录下，比如要运行MotorModel_1，就把路径调到MotorModel_1所在
%的地址路径下，再跑
h0 = 40;
h1 = 90;
Ts = 2.222222e-4;%1/4500;
h11 = (2-Ts*h1)/(2+Ts*h1);
h12 = Ts/(2+Ts*h1);
h13 = Ts/(2+Ts*h1);
h01 = (2-Ts*h0)/(2+Ts*h0);
h02 = Ts/(2+Ts*h0);
h03 = Ts/(2+Ts*h0);



