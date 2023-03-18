% Setup
clear; clear Logger;
close all;
log = Logger.get(0);
rng(0);

% Parameters
state_x = false;
T = 30;
QNs = [1000 500 1000 1000 1000; 50 50 65 80 100];
Ns = logspace(-2, 0.5, 25);
data = [];

for K=4:8
    % Create model and simulation objects
    pend = PendulumModel();
    pend.K = K; pend.state_x = state_x;
    sim = Simulation(pend);
    sim.Nt = QNs(2, K-3);    sim.Nte = QNs(2, K-3);
    set_gains(sim, QNs(1, K-3)*[1 1], [0 0]);

    % Set initial conditions and set points
    dth0 = zeros(1, K);
    th0 = zeros(1, K); 
    thss = ones(1, K)*pi;
    sim.xtarg = [0 thss 0 dth0]'; 
    x0 = sim.xtarg;

    % Simulate system at various noise levels
    for N=Ns
        sim.vsig = N*[1e-5 ones(1, K)*1e-3];
        sim.wsig = N*[1 ones(1, K) 1 ones(1, K)*1e2]*1e-5;
        
        % Simulate
        log.log(0, "K=%f\tN=%f", K, N);
        res = sim.simulate(x0, T);

        travel = max(res.x)-min(res.x);
        max_acc = max(abs(res.u));

        flag = false;
        if (res.t(end) ~= 29.95) % Failed
            travel = 0;
            max_acc = 0;
            flag = true;
        end

        data = [data; K N travel max_acc]; %#ok
        save("noise_data", "data");

        if flag
            break;
        end
    end
end

%%% Helper functions
function set_gains(obj, Qt, Qx)
    K = obj.model.K;
    obj.Qt = diag([Qt(1)*ones(1, K+1) Qt(2)*ones(1, K+1)]);
    obj.Qx = diag([Qx(1) Qx(2)*ones(1, K) zeros(1, K+1)]);
end
