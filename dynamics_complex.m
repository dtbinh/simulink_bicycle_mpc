function [sys,x0,str,ts]=dynamics_complex(t,x,u,flag)
switch flag
    case 0   %µ÷ÓÃ³õÊŒ»¯º¯Êý
        [sys,x0,str,ts]=mdlInitializeSizes(t,x,u);
    case 1   %µ÷ÓÃ�?¬�?ø×ŽÌ¬µÄžü�?Âº¯Êý
        sys=mdlDerivatives(t,x,u);
    case 3   %µ÷ÓÃÊä³ö�?¿µÄŒÆËãº¯Êý
        sys=mdlOutputs(t,x,u);
    case {2,4,9}  %ÎŽÊ¹ÓÃ¹ýµÄflagÖµ
        sys=[];
    otherwise %ŽŠÀíŽíÎó
        error(['Unhandled flag=',num2str(flag)]);
end
%=======================================================================
%µ±flagÎª0Ê±œø�?�?Õûžö�?µ�?³µÄ³õÊŒ»¯
%=======================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u)
%Ê×�?Èµ÷ÓÃsimsizesº¯ÊýµÃ³ö�?µ�?³¹æÄ£²ÎÊýsizes,²¢žùŸ�?ÀëÉ¢�?µ�?³µÄÊµŒÊÇé¿öÉèÖÃ
%sizes±ä�?¿
sizes=simsizes;
sizes.NumContStates=6;
sizes.NumDiscStates=0;
sizes.NumOutputs=6;
sizes.NumInputs=2;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[8;0;0;0;0;0];   
str=[];
ts=[0 0];

%=======================================================================
%µ±flagÎª1Ê±,žü�?ÂÕûžö�?¬�?ø�?µ�?³µÄ×ŽÌ¬±ä�?¿
%=======================================================================
function sys=mdlDerivatives(t,x,u)
%2018-07-23, Yushu Yu



%input signal 
delta_f = u(1);   %steering angle 
a_x = u(2);    %acc 

if (x(1)<= 0)
    x(1) = 1e-4;
end 

%states:
xp_dot = x(1);  %lateral speed
yp_dot = x(2);  %longitudinal speed
psi_dot = x(3); 
epsi = x(4);
ey= x(5);  %lateral position
s = x(6);  %logitudinal position 

%constants: 
a = 1.41; 
b = 1.576; 
mu =0.5; 
Fzf = 21940/2; 
Fzr = 21940/2; 
cf = 65000; 
cr = 65000; 
m = 2194; 
Iz = 4770; 
psi_dot_com = 0;
p =Iz/(m*b);

%state equation: 
f_x = [ yp_dot*psi_dot;... 
    -2*(cf+cr)/(m*xp_dot)*yp_dot-2*(a*cf-b*cr)/m/xp_dot*psi_dot-xp_dot*psi_dot; ...
     -2*(a*cf-b*cr)/Iz/xp_dot*yp_dot-2*(a*a*cf+b*b*cr)/Iz/xp_dot*psi_dot;...
     psi_dot - psi_dot_com;...
     yp_dot*cos(epsi) + xp_dot*sin(epsi); ...
     xp_dot*cos(epsi)-yp_dot*cos(epsi)];
 
g_x = [0, 1; ...
    2*cf/m, 0; ...
    2*a*cf/Iz, 0;...
    0, 0;...
    0, 0;...
    0, 0];

sys = f_x + g_x*u;
%diff: 
% yp_dot_dot = 2*(a+b)*mu*Fzf*beta_yf /(m*b) - psi_dot*xp_dot; 
% xp_dot_dot = 2*mu*Fzf*beta_xf/m+2*mu*Fzr*beta_r/m + psi_dot*yp_dot - psi_dot*psi_dot*p; 
% psi_dot_dot = 2*a*mu*Fzf*beta_yf/Iz - 2*b*cr*yp_dot/(Iz*xp_dot) + 2*b*cr*(b+p)*psi_dot/(Iz*xp_dot);   %if xp_dot=0, may induce inf or NaN, should be dealt carefully 
% epsi_dot = psi_dot - psi_dot_com; 
% ey_dot = yp_dot + xp_dot*epsi; 
% s_dot = xp_dot; 

% sys = [yp_dot_dot; xp_dot_dot; psi_dot_dot; epsi_dot; ey_dot; s_dot];
 
% sys = zeros(6,1);
 

%=======================================================================
%µ±flagÎª3Ê±,ŒÆËã�?µ�?³µÄÊä³ö±ä�?¿:·µ»Ø�?œžö×ŽÌ¬
%=======================================================================
function sys=mdlOutputs(t,x,u)
sys = x; 
if (sys(1)<= 0)
    sys(1) = 1e-4;
end
 
 


