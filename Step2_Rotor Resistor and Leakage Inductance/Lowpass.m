clc;
clear all;
h0 = 40;
h1 = 90;
Ts = 2.222222e-4;%1/4500;
h11 = (2-Ts*h1)/(2+Ts*h1);
h12 = Ts/(2+Ts*h1);
h13 = Ts/(2+Ts*h1);
h01 = (2-Ts*h0)/(2+Ts*h0);
h02 = Ts/(2+Ts*h0);
h03 = Ts/(2+Ts*h0);



