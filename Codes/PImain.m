m1 = 10; % Mass of link 1
m2 = 5; % Mass of link 2
l1 = 0.2; % Length of link 1
l2 = 0.1; % Length of link 2
g = 9.81; % Gravitational acceleration

q10 = 0.1; 
q20 = 0.1;
x10 = 0;
x20 = 0;
q1dot0 = 0; 
q2dot0 = 0;
t0 = 0; 
tf = 10;

q1_fin = 0;
q2_fin = 0;

kp1 = 250;
ki1 = 3; 
kp2 = 250; 
ki2 =3
;

tspan = [t0 tf];

IC =[q10, q20, x10, x20, q1dot0, q2dot0];

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);

[time, state_values] = ode45(@(t, s) pi_k(t, s, m1, m2, l1, l2, g, kp1, ki1, kp2, ki2,q1_fin, q2_fin),tspan,IC, options);

q1 = state_values(:, 1);
q2 = state_values(:, 2);

figure;
subplot(2, 1, 1);
plot(time, q1, 'r');
xlabel('Time (s)');
ylabel('q1 (rad)');
sgtitle('Joint Angles vs. Time (PI Control)');

subplot(2, 1, 2);
plot(time, q2, 'g');
xlabel('Time (s)');
ylabel('q2 (rad)');

e1 = q1_fin - q1;
e2 = q2_fin - q2;

figure;
subplot(2, 1, 1);
plot(time, e1, 'r');
xlabel('Time (s)');
ylabel('e1 (rad)');
sgtitle('Error in Joint Angles vs. Time (PI Control)');

subplot(2, 1, 2);
plot(time, e2, 'g');
xlabel('Time (s)');
ylabel('e2 (rad)');
