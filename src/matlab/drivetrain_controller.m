% TODO:
% get mass
% get J (or guesstimate)
% log data from robot

close all;
load 'drivetrain_spin_low'
load 'straight'

%Coefficients describing the system
m = 68;
rb = 0.6477 / 2.0;
J = 7; % fiddle
stall_current = 133.0;
R = 12.0 / stall_current / 4 / 0.4; % fiddle
Km = (12.0 - R * 2.7) / (4800.0 / 60.0 * 2.0 * pi); % 12v and 4650
Kt = 0.008;
r = 0.04445; % 3.5 inches diameter

G_low = 32.0 / 3;
G_high = 45.0 / 30.0 * 50.0 / 15.0;
dt = 0.02;

G = G_low;

msp = (1.0 / m + rb ^ 2.0 / J);
msn = (1.0 / m - rb ^ 2.0 / J);
tc = -Km * Kt * G ^ 2.0 / (R * r ^ 2.0);
mp = G * Kt / (R * r);

%Generating system model
A = [0 1 0 0; 0 msp*tc 0 msn*tc; 0 0 0 1; 0 msn*tc 0 msp*tc];
B = [0 0; msp * mp msn * mp; 0 0; msn * mp msp * mp];
C = [1 0 0 0; 0 0 1 0];
D = [0 0; 0 0];

%Continuous to discrete time
dm = c2d(ss(A, B, C, D), dt);

%Controller
hp = .74;
lp = .8;
K = place(dm.a, dm.b, [hp, hp, lp, lp]);

%Observer
hlp = 0.08;
llp = 0.10;
L = place(dm.a', dm.c', [hlp, hlp, llp, llp])';

% Plot what we computed
fd = fopen('/Users/ebakan/Desktop/control_loops_matlab/drivecontroller.h', 'w');
n = 1;
sm = [];
%writeMatHeader(fd, size(dm.a, 1), size(dm.b, 2));
writeMatFlash(fd, dm.a, 'A');
writeMatFlash(fd, dm.b, 'B');
writeMatFlash(fd, dm.c, 'C');
writeMatFlash(fd, dm.d, 'D');
writeMatFlash(fd, L, 'L');
writeMatFlash(fd, K, 'K');
writeMatFlash(fd, [12; 12], 'U_max');
writeMatFlash(fd, [-12; -12], 'U_min');
%writeMatFooter(fd);
fclose(fd);

% MAGIC
full_model = dss([dm.a (-dm.b * K); eye(4) (dm.a - dm.b * K - L * dm.c)], [0, 0; 0, 0; 0, 0; 0, 0; L], [C, [0, 0, 0, 0; 0, 0, 0, 0]], 0, eye(8), 0.01);

n = 1;
sm_strait = [];
t = straight(1, 1) + dt * (n - 1);
x = [straight(1, 2); 0; straight(1, 3); 0];
while t < straight(end, 1)
    sm_strait(n, 1) = t;
    sm_strait(n, 2) = (x(1,1) + x(3,1)) / 2.0;
    t = t + dt;
    x = dm.a * x + dm.b * [12.0; 12.0];
    n = n + 1;
end

figure;
plot(straight(:, 1), (straight(:, 2) + straight(:, 3)) / 2.0, sm_strait(:, 1), sm_strait(:, 2));
legend('actual', 'sim');
%%
n = 1;
sm_spin = [];
t = drivetrain_spin_low(1, 1) + dt * (n - 1);
x = [drivetrain_spin_low(1, 2); 0; drivetrain_spin_low(1, 3); 0];
while t < drivetrain_spin_low(end, 1)
    sm_spin(n, 1) = t;
    sm_spin(n, 2) = (x(1,1) - x(3,1)) / 2.0;
    t = t + dt;
    x = dm.a * x + dm.b * [drivetrain_spin_low(n, 4); drivetrain_spin_low(n, 5)];
    n = n + 1;
end

figure;
plot(drivetrain_spin_low(:, 1), (drivetrain_spin_low(:, 2) - drivetrain_spin_low(:, 3)) / 2.0, sm_spin(:, 1), sm_spin(:, 2));
legend('actual', 'sim');

%figure;
%nyquist(full_model);


%%
t = 0;
x = [0; 0; 0; 0;];
while t < logging(end, 1)
    sm(n, 1) = t;
    sm(n, 2) = x(1,1);
    sm(n, 3) = x(3,1);
    t = t + dt;
    x = dm.a * x + dm.b * [12.0; 12.0];
    n = n + 1;
end

figure;
plot(logging(:, 1), logging(:, 2), sm(:, 1), sm(:, 2));
legend('actual', 'sim');
