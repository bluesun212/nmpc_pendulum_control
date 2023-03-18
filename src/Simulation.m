classdef Simulation < handle
    properties
        Delta = 0.05;       % Simulation time step
        c = 3;              % Collocation points per step
        Nt = 30;            % Regulator time horizon
        Nte = 30;           % Estimator time horizon
        Qt = 1;             % Regulator terminal cost coefficient
        Qx = 0;             % Regulator cart x stage cost coefficient
        xtarg = [];         % System steady state target
        wsig = 1e-5;        % Process noise multiplier
        vsig = 1e-5;        % Sensor noise multiplier
        step_fun;           % Function to be called at each step
        use_est = true;     % Whether to use the estimator to get x
        use_reg = true;     % Whether to use the controller to get u
        use_noise = true;   % Whether to inject noise 
        
        % Internal properties
        Nr = struct();
        logger;
        model;
        mpc;
        ode;
    end

    methods
        function obj = Simulation(pend_model)
            obj.logger = Logger.get();
            obj.logger.log(0, "Initializing simulation");
            obj.mpc = import_mpctools();
            obj.model = pend_model;

            % Create CasADi models from symbolic model
            [Msym, Fsym, xsym, usym] = obj.model.get_plant_model();
            M = matlabFunction(Msym, 'vars', {xsym});
            F = matlabFunction(Fsym, 'vars', {xsym, usym});
            obj.ode = @(x, u) M(x)\F(x, u);

            obj.Nr.x = length(xsym);
            obj.Nr.u = 1;
            obj.Nr.y = obj.model.K+1;
        end

        function sol = simulate(obj, x0, stop_time)
            % Create CasADi functions
            obj.logger.log(0, "Creating MPC objects");
            
            obj.Nr.t = obj.Nt;
            obj.Nr.c = obj.c;
            N = obj.Nr;

            f_plant = obj.mpc.getCasadiIntegrator(obj.ode, obj.Delta, ...
                [N.x N.u], {'x', 'u'});
            f_model = obj.mpc.getCasadiFunc(obj.ode,  ...
                [N.x N.u], {'x', 'u'});

            % Set up NMPC
            guess = struct('u', zeros(N.u, N.t), ...
                'x', repmat(x0, 1, N.t+1));
            par = struct('xt', obj.xtarg, 'Qx', obj.Qx, 'Qt', obj.Qt);

            stage_cost = @(x, u, xt, Q) (x-xt)'*Q*(x-xt) + u^2;
            term_cost = @(x, ~, xt, Q) (x-xt)'*Q*(x-xt);
            l = obj.mpc.getCasadiFunc(stage_cost, ...
                {N.x N.u N.x [N.x N.x]}, {'x', 'u', 'xt', 'Qx'}, {'l'});
            Vf = obj.mpc.getCasadiFunc(term_cost, ...
                {N.x N.u N.x [N.x N.x]}, {'x', 'u', 'xt', 'Qt'}, {'Vf'});

            reg = obj.mpc.nmpc('f', f_model, 'N', N, 'l', l, ...
                'Vf', Vf, 'Delta', obj.Delta, 'guess', guess, 'par', par);

            % Set up NMHE
            measure = @(x) x(1:N.y, :);
            
            Winv = diag(obj.wsig.^-2);
            Vinv = diag(obj.vsig.^-2);
            uest = zeros(N.u, obj.Nte);
            yest = measure(repmat(x0, 1, obj.Nte+1));

            estage_cost = @(w, v) w'*Winv*w + v'*Vinv*v;
            h = obj.mpc.getCasadiFunc(measure, N.x, {'x'}, {'h'});
            le = obj.mpc.getCasadiFunc(estage_cost, [N.x N.y], ...
                {'w', 'v'}, {'le'});
            Ne = obj.Nr;
            Ne.t = obj.Nte;

            est = obj.mpc.nmhe('f', f_model, 'l', le, 'h', h, 'N', Ne, ...
                'u', uest, 'y', yest, 'Delta', obj.Delta, ...
                'wadditive', true);

            % Set up simulation variables
            Ns = stop_time/obj.Delta;
            t = 0:obj.Delta:stop_time;
            v = obj.use_noise*diag(obj.vsig)*randn(N.y, Ns);
            w = obj.use_noise*diag(obj.wsig)*randn(N.x, Ns);
            x = nan(N.x, Ns+1);
            x(:, 1) = x0;
            y = nan(N.y, Ns);
            u = nan(N.u, Ns);
            u(1) = 0;
            uprev = u(1);
            
            % Start simulation
            obj.logger.log(0, "Starting simulation");
            for i=1:Ns
                if isa(obj.step_fun, 'function_handle')
                    obj.step_fun(obj, t(i));
                end

                % Estimate states
                y(:, i) = measure(x(:, i)) + v(:, i);
                xest = x(:, i);

                est.newmeasurement(y(:, i), uprev);
                if obj.use_est
                    est.solve();
                    est.saveguess();
                    xest = est.var.x(:, end);

                    if check_solve(est.status, obj.logger, 'Estimator')
                        break;
                    end
                end
                
                % Find a control law for this step
                u(i) = 0;
                reg.fixvar('x', 1, xest);
                reg.par.xt = obj.xtarg;
                reg.par.Qx = obj.Qx;
                reg.par.Qt = obj.Qt;

                if obj.use_reg
                    reg.solve();
                    reg.saveguess();
                    u(i) = reg.var.u(1);

                    if check_solve(reg.status, obj.logger, 'Regulator')
                        break;
                    end
                end

                uprev = u(i);

                % Update user with percent complete
                line = sprintf('%.02f%% completed', i/Ns*100);
                obj.logger.log_eline(0, '%s', line);
            
                % Simulate plant with given input
                x(:, i+1) = full(f_plant(x(:, i), u(i))) + w(:, i);
            end

            obj.logger.stop_eline();

            % Cut off any NaNs and make sizes equal
            x = rmmissing(x, 2); y = rmmissing(y, 2); u = rmmissing(u, 2);
            Ns = min([size(x, 2) size(y, 2) size(u, 2)]);
            x = x(:, 1:Ns); y = y(:, 1:Ns); u = u(1:Ns); t = t(1:Ns);

            % Build return struct
            sol = struct();
            sol.model = obj.model;
            K = obj.model.K;
            sol.x = x(1, :); sol.th = x(2:K+1, :);
            sol.dx = x(K+2, :); sol.dth = x(K+2:end, :);
            sol.y = y; sol.u = u; sol.t = t;
        end
    end
end

function err = check_solve(stat, log, solver)
    err = ~any(strcmp('Solve_Succeeded', {'Solve_Succeeded', ...
        'Solved_To_Acceptable_Level'}));
    if err
        log.stop_eline();
        log.log(0, "%s error: %s", solver, stat);
    end
end
