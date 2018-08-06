close all;

P_com=reference(:,2:3);
P_sens=state(:,5:6);

figure(1);   
plot(P_com(:,2),P_com(:,1),'-.',P_sens(:,2),P_sens(:,1)),grid; hold on;
% axis equal;
xlabel('X(m)');ylabel('Y(m)');
title('POSITION TRACKING:SENSED VS COMMAND');
legend('traj_{com}','traj_{sen}');

% plot(ob(:,1),ob(:,2), '*r');
global pos_ob_array_pre;
for i=1:size(pos_ob_array_pre,2)
    plot(pos_ob_array_pre(1,i), pos_ob_array_pre(2,i), '*r');
end
 
figure(2);   
subplot(4,1,1);
plot(t,P_com(:,1),'-.',t,P_sens(:,1)),grid;
ylabel('ey(m)');
title('POSITION:SENSED VS COMMAND');
legend('P_c_o_m','P_s_e_n');

subplot(4,1,2);
plot(t,P_com(:,2),'-.',t,P_sens(:,2)),grid;
ylabel('s(m)');
 xlabel('time(s)');
 
 subplot(4,1,3);
plot(t,state(:,1) ),grid;
ylabel('v_x(m/s)');
 xlabel('time(s)');
 
 subplot(4,1,4);
plot(t,state(:,4)/pi*180 ),grid;
ylabel('\psi(degree)');
 xlabel('time(s)');

 
figure(3);
subplot(2,1,1);
plot(t,input(:,1)),grid;
ylabel('control input \delta'); 

subplot(2,1,2);
plot(t,input(:,2)),grid;
ylabel('control input a');xlabel('time(s)');


figure(5);
subplot(3,1,1);
plot(t,test(:,1)),grid;  ylabel('An 1');
title('Angle constraint');

subplot(3,1,2);
plot(t,test(:,2)),grid;  ylabel('An 2');

subplot(3,1,3);
plot(t,test(:,3)),grid; ylabel('bn ');
xlabel('time(s)');

figure(6);
subplot(3,1,1);
plot(t,test(:,4)),grid; ylabel('An 1 ');
title('distance constraint');

subplot(3,1,2);
plot(t,test(:,5)),grid; ylabel('An 2 ');

subplot(3,1,3);
plot(t,test(:,6)),grid; ylabel('b_n');
xlabel('time(s)');

figure(7);
subplot(3,1,1);
plot(t,test(:,7)),grid; ylabel('An 1 ');
title('oppsite constraint');

subplot(3,1, 2);
plot(t,test(:,8)),grid; ylabel('An 2 ');

subplot(3,1,3);
plot(t,test(:,9)),grid; ylabel('b_n');
xlabel('time(s)');


figure(4);
subplot(5,1,1);
plot(t,test(:,10)),grid; 
ylabel('angle ');
title('constraint function'); 

subplot(5,1,2);  
plot(t,test(:,11)),grid;
ylabel('oppsite');

subplot(5,1,3);
plot(t,test(:,12)),grid;
ylabel('distance'); 

subplot(5,1,4);
plot(t,test(:,13)),grid;
ylabel('\beta');

subplot(5,1,5);
plot(t,test(:,14)),grid;
ylabel('value_min ');
xlabel('time(s)');


 


