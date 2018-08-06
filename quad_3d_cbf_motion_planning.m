%% CLF-CBF-QP geometric control of a single 3D-moving quadrotor
%Yu Yushu
% ------------------------------------------------
% The corresponding data of system trajectory
% is stored in mat file 
% ------------------------------------------------
% T: the time duration of simulation 
%
% 

function quad_3d_cbf_motion_planning(T)
clc;
close all;
if nargin < 1
    T = 10; 
end


%% Simulation based on ODE function
% control function handles
ctrl_hdl1 = @virtual_Control;
current_hdl = ctrl_hdl1;
ctrl_hdl_str = func2str(current_hdl);

% initial condition of different trials
% -------------------------------------------------
    % x(1:3): position
    % x(4:6): velocity
    % x(7:15): rotation matrix
    % x(16:18): body angular velocity
% -------------------------------------------------
% initial condition
r0=[0;0;0];
dr0=[1;0;0];
ddr0=[0;0;0];
dddr0=[0;0;0];

y0=[r0;dr0;ddr0;dddr0];

% option of ode function 
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-8);

% simulation process
disp('The simulation process has started.');
disp(strcat('Controller: ', ctrl_hdl_str));
disp('---------------------------------------------');
tspan = [0 T];
[t1, y1] = ode15s(@quad_3d_ode, tspan, y0, options, current_hdl);

% save all the data into .mat file 
 
save('sim_data.mat');
disp('Data successfully stored!');


end


% geometric backstepping by Taeyoung Lee
function [u] = virtual_Control(~, y, trajd)
%commands
rhat=[trajd(1);trajd(2);trajd(3)];
rhatd=[trajd(4);trajd(5);trajd(6)];
rhatdd=[trajd(7);trajd(8);trajd(9)];
rhatddd=[trajd(10);trajd(11);trajd(12)];
rhatdddd=[trajd(13);trajd(14);trajd(15)];

%sensed 
r=[y(1);y(2);y(3)];
rd=[y(4);y(5);y(6)];
rdd=[y(7);y(8);y(9)];
rddd=[y(10);y(11);y(12)];

%obstacles:
ob=[5;0;0];


%parameters:
 
k1=diag([1,1,1]);
k2=diag([1,1,1]);
k3=diag([1,1,1]);
k4=diag([1,1,1]);

%nominal input
v_nom=rhatdddd-k1*(r-rhat)-k2*(rd-rhatd)-k3*(rdd-rhatdd)-k4*(rddd-rhatddd);

%parameter:
Ds=2;
% coefficient of z:
cz=2;

h_x=cbf_planning([r(1)-ob(1);rd(1);rdd(1);rddd(1)]);
h_y=cbf_planning([r(2)-ob(2);rd(2);rdd(2);rddd(2)]);
h_z=cbf_planning([r(3)-ob(3);rd(3);rdd(3);rddd(3)]);

h=h_x.h+h_y.h+h_z.h/cz^4-Ds^4;
dh=h_x.dh+h_y.dh+h_z.dh/cz^4;
ddh=h_x.ddh+h_y.ddh+h_z.ddh/cz^4;
dddh=h_x.dddh+h_y.dddh+h_z.dddh/cz^4;

relative_r=r-ob;
 

%parameter:
k_ob=[1.2,1.2,0.5,0.3];

%optimal problem: 
%variable is v-vnom
A=[-4*relative_r(1)^3, -4*relative_r(2)^3, -4/cz^4*relative_r(3)^3];
b=k_ob*[h;dh;ddh;dddh]+...
  (24*rd.^4+144*relative_r.*(rd.^2).*rdd+36*(relative_r.^2).*(rdd.^2)+48*(relative_r.^2).*rd.*rddd)'*ones(3,1);
b=b-A*v_nom;

H=eye(3,3);
f2 = zeros(3, 1);

optoption_1 = optimset('Display', 'off', 'TolFun', 1e-10);
x= quadprog(H, f2, A, b, [], [], [], [], [], optoption_1);

v=x+v_nom;

out=[v;h];
    
u=v; 
    
    
    
end


%% Ode Function of this 3d-Moving Quadrotor
function [dy] = quad_3d_ode(t, y, ctrl_hdl)
dy = zeros(12, 1);

 
% convert the current state 
dr = y(4:6);
ddr = y(7:9);
dddr = y(10:12);
 

% get the current reference and control input 
trajd = traj_gen(t);
u = feval(ctrl_hdl, t, y, trajd);


%update the system dynamics 
dy(1:3) = dr;
dy(4:6) = ddr;
dy(4:6) = dddr;
dy(4:6) = u;

% -----------------------------------------------
% check simulation time for stability property
debug = 1;
if debug == 1
    disp(['The current time is ', num2str(t)]);
end
% -----------------------------------------------
end


function out = traj_gen(t)

% r=7.5;
% pos = [r*cos(1/r*t); r*sin(1/r*t); 0];
% vel = [-sin(1/r*t); cos(1/r*t); 0];
% acc = [-1/r*cos(1/r*t); -1/r*sin(1/r*t); 0];
% dacc =[1/r^2*sin(1/r*t); -1/r^2*cos(1/r*t); 0]; 
% d2acc = [1/r^3*cos(1/r*t); 1/r^3*sin(1/r*t); 0];


v=1;
pos = [v*t; 0; 0];
vel = [v; 0; 0];
acc = [0; 0; 0];
dacc =[0; 0; 0]; 
d2acc = [0; 0; 0];


out=[pos; vel; acc; dacc; d2acc];
end


 
