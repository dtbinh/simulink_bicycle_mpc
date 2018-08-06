function output_safety = mpc_controller(u)
%calculate the safet_certificate of the system

%u: the input and the state of the system and the obstacles 

%feedback states:
xp_dot = u(1);  %lateral speed
yp_dot = u(2);  %longitudinal speed
psi_dot = u(3); 
epsi = u(4);
ey= u(5);  %lateral position
s = u(6);  %logitudinal position 

%reference trajectory
tra_com = [u(7);u(8);u(9)];  %epsi, ey, s, notice the variables are in this order, velocity and acc are the same 
tra_com_dot = [u(10);u(11);u(12)];
tra_com_ddot = [u(13);u(14);u(15)]; 
time = u(16); 

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


horizon = 1; 

coef_.A = zeros(10,2);
coef_.B = zeros(10,1);

% %using the m-file: 
% coef_ = cbf_seperate(y);   %single constraints 
% 
% % using the mex-file: (should run unicycle_c_seperate.m firstly)
% out = unicycle_input_RUN(t, t+horizon, y(1), y(2), y(3), y(4), ...
%     coef_(1), coef_(2), coef_(3), ... %row 1 
%     coef_(1), coef_(2), coef_(3), ... %row 2 
%     coef_(1), coef_(2), coef_(3), ... %row 3 
%     coef_(1), coef_(2), coef_(3), ... %row 4 
%     coef_(1), coef_(2), coef_(3), ...  %row 5 
%     coef_(1), coef_(2), coef_(3), ... %row 6 
%     coef_(1), coef_(2), coef_(3), ... %row 7
%     coef_(1), coef_(2), coef_(3), ... %row 8 
%     coef_(1), coef_(2), coef_(3), ... %row 9
%     coef_(1), coef_(2), coef_(3));

% only static obstacles: 
% coef_ = cbf_seperate_mult_constraints(y); 

% can address dynamics obstacles: 
% coef_ = cbf_seperate_mult_dynamic_constraints([y; t]); 

% if (coef_.C == 1)
%     %the feasible control space is empty
%     u = [-4; 0]; 
% else
% %     using the mex-file: (should run unicycle_c_seperate.m firstly)
    out = bicycle_input_s_RUN(time, time+horizon, ...
        u(1), u(2), u(3), u(4), u(5), u(6),  ...
        coef_.A(1,1), coef_.A(1,2), coef_.B(1), ... %row 1 
        coef_.A(2,1), coef_.A(2,2), coef_.B(2),  ... %row 2 
        coef_.A(3,1), coef_.A(3,2), coef_.B(3),  ... %row 3 
        coef_.A(4,1), coef_.A(4,2), coef_.B(4),  ... %row 4 
        coef_.A(5,1), coef_.A(5,2), coef_.B(5),  ...  %row 5 
        coef_.A(6,1), coef_.A(6,2), coef_.B(6), ... %row 6 
        coef_.A(7,1), coef_.A(7,2), coef_.B(7),  ... %row 7
        coef_.A(8,1), coef_.A(8,2), coef_.B(8), ... %row 8 
        coef_.A(9,1), coef_.A(9,2), coef_.B(9),  ... %row 9
        coef_.A(10,1), coef_.A(10,2), coef_.B(10));

 
    if (out.CONVERGENCE_ACHIEVED ==1)
        mpc_control = out.CONTROLS(1,2:end)';   %the MPC control 
    else
        mpc_control = [0; -4];  %the MPC solver does not converge
    end

 

global output_safety; 
% output_safety = [out; ...
%     results_2(1).A_n_angle_fix(1); results_2(1).A_n_angle_fix(2); results_2(1).B_n_angle_fix; ...
%     results_2(1).A_n_angle_moving(1); results_2(1).A_n_angle_moving(2); results_2(1).B_n_angle_moving; ...
%     results_2(1).A_n_dis(1); results_2(1).A_n_dis(2); results_2(1).B_n_dis; ...
%     results_2(1).h_angle_fix; results_2(1).h_angle_moving;  results_2(1).h_dis; beta_2(1); value_min];
output_safety = [mpc_control;  zeros(14,1)];
 

