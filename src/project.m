% Jared Jonas - Multi Pendulum Control
% Project created for ChE 286 - Model Predictive Control

% Setup
clear; clear Logger;
close all;
Logger.get(0);
rng(0);

% Parameters
K = 3;          % Number of links
T = 60;         % Simulation end time
Nt = 30;        % MPC time horizon
Nte = Nt;       % MHE time horizon
th0 = zeros(1, K);  dth0 = zeros(1, K);
x0 = [0 th0 0 dth0]';   % Initial state

% Create model and simulation objects
pend = PendulumModel(); pend.K = K;
sim = Simulation(pend);
sim.Nt = Nt;    sim.Nte = Nte;
sim.vsig = [1e-5 ones(1, K)*1e-3];                  % Sensor noise
sim.wsig = [1 ones(1, K) 1 ones(1, K)*1e2]*1e-5;    % Process noise
sim.step_fun = @step;   % Gain scheduling function
sim.xtarg = x0;         % Steady state target
set_gains(sim, [4000 4000], [600 1]);   % Set MPC stage+arrival costs

% Simulate
res = sim.simulate(x0, T);
fprintf("Cost: %d\t Travel: %f\t Max acc: %f\n", sum(res.u.^2), ...
    max(res.x)-min(res.x), max(abs(res.u)));

% Plot
anim = Animation();
anim.add_sim_result(res, 0);
anim.animate();



%%% Helper functions
function step(obj, t)
    % Target position and penalties
    goals = [6 1 7 4 5 2 3 0; 1 1 0 1 1 1 1 1];
    gains = [4000 100; 1 400];

    tind = 1+floor(t/7.5);
    eq_pt = dec2bin(goals(1, tind), 3) == '1';
    g = goals(2, tind) + 1;

    % Set target and MPC gains accordingly
    obj.xtarg = [0 eq_pt*pi zeros(1, 4)]';
    set_gains(obj, [4000 gains(1, g)], [600 gains(2, g)]);
end

% Set gains according to best found form
function set_gains(obj, Qt, Qx)
    K = obj.model.K;
    obj.Qt = diag([Qt(1)*ones(1, K+1) Qt(2)*ones(1, K+1)]);
    obj.Qx = diag([Qx(1) Qx(2)*ones(1, K) zeros(1, K+1)]);
end
