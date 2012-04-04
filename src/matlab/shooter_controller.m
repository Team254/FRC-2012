%%
load 'x254_shooter_spinup'
close all

J = 0.0029;
stall_current = 108.7;
stall_torque = 75.4; %% in-lbs
R = 12.0 / stall_current + 0.024 + .003;
Km = (11 - R * 1.4) / (20770.0 / 60.0 * 2.0 * pi);
Kt = stall_torque * 0.00706155181 / stall_current;
G = 12.0/60.0;
dt = 0.02;

A = [0 1; 0 -Km * Kt / (J * G * G* R)];
B = [0; Kt / (J * G * R)];
C = [1 0];
D = 0;

dm = c2d(ss(A, B, C, D), dt);

%K = place(dm.a, dm.b, [.65, .975]);
K = place(dm.a, dm.b, [.65, .96]);
rpl = .36;
ipl = 0.07;
L = place(dm.a', dm.c', [rpl + 1j * ipl, rpl - 1j * ipl])';

% Plot what we computed

fd = fopen('/Users/ebakan/Desktop/shootercontroller.h', 'w');

n = 45;
sm = [];

full_model = dss([dm.a (-dm.b * K); eye(2) (dm.a - dm.b * K - L * dm.c)], [0; 0; L], [C 0 0], 0, eye(4), 0.01);

t = x254_shooter_spinup(1, 1) + .27;
x = [x254_shooter_spinup(1, 2); 0];
while t < x254_shooter_spinup(end, 1) - 1
    sm(n, 1) = t;
    sm(n, 2) = x(1,1);
    t = t + dt;
    x = dm.a * x + dm.b * x254_shooter_spinup(n, 3);
    n = n + 1;
end

figure;
plot(x254_shooter_spinup(:, 1), x254_shooter_spinup(:, 2), sm(:, 1), sm(:, 2));
legend('actual', 'sim');

%figure;
%nyquist(full_model);

n = 1;
sm = [];
%writeMatHeader(fd, size(dm.a, 1), size(dm.b, 2));
writeMatFlash(fd, dm.a, 'A');
writeMatFlash(fd, dm.b, 'B');
writeMatFlash(fd, dm.c, 'C');
writeMatFlash(fd, dm.d, 'D');
writeMatFlash(fd, L, 'L');
writeMatFlash(fd, K, 'K');
writeMatFlash(fd, 12, 'U_max');
writeMatFlash(fd, -1, 'U_min');
%writeMatFooter(fd);
fclose(fd);

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

