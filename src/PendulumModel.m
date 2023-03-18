classdef PendulumModel
    properties
        K = 1;          % Number of linkages
        L = 1;          % Linkage length
        r = 1/2;        % Position of COM on linkage from [0, 1]
        m = 1;          % Linkage mass
        M = 10;         % Cart mass
        g = 9.81;       % Gravitational constant
        I = 1/12;       % Linkage moment of inertia around COM
        c = 0.1;        % Friction coefficient 
        state_x = false;% Cart position is a state or input?
    end

    methods
        % Symbolic derivation of K-link pendulum
        function [M, F, x, u] = get_plant_model(obj)
            % Define symbolic variables to derive plant model
            q =   [sym('x', 'real');   sym('th', [obj.K 1], 'real')];
            dq =  [sym('dx', 'real');  sym('dth', [obj.K 1], 'real')];
            ddq = [sym('ddx', 'real'); sym('ddth', [obj.K 1], 'real')];
            u = sym('u');
            k = obj.K + 1;

            % Get equations of motion using a variational approach
            % adding in non-conservative forces (friction)
            Lg = obj.get_lagrangian(q, dq);
            eq = find_euler_lagrange_eqs(Lg, q, dq, ddq) + obj.c*dq;

            % Replace all ddxs with us, then remove dx dynamics
            if ~obj.state_x
                eq = subs(eq, 'ddx', 'u');
                eq(1) = ddq(1);
            end

            % Put in form Mq'' = f(q, q', u)
            eq(1) = eq(1) - u;
            M22 = jacobian(eq, ddq);
            F22 = M22*ddq - eq;

            % Now put in final form
            M = [eye(k) zeros(k); zeros(k) M22];
            F = [dq; F22];
            x = [q; dq];
        end

        function [xs, xc] = get_kinematics(obj, q)
            % Set up matrices
            xs = [[q(1); 0] zeros(2, length(q)-1)];
            xc = xs;
        
            % Iterate through each joint and find its coordinates
            for i=2:length(q)
                crd = [sin(q(i)); -cos(q(i))];
                xs(:, i) = xs(:, i-1) + obj.L*crd;
                xc(:, i) = xs(:, i-1) + obj.L*obj.r*crd;
            end
        end

        function L = get_lagrangian(obj, q, dq)
            [~, xc] = obj.get_kinematics(q);
            dxc = find_total_derivative(xc, q, dq);
            L = obj.M/2*dq(1)^2; % Kinetic energy of cart
        
            % For each joint, Li = 1/2 mv'v + 1/2 Iq^2 - mg[0 1]xc
            for i=2:length(q)
                L = L + obj.m/2*(dxc(:, i)'*dxc(:, i)) + ...
                    obj.I/2*dq(i)^2 - obj.m*obj.g*[0 1]*xc(:, i);
            end
        
            L = simplify(L);
        end
    end
end

% Other functions
function df = find_total_derivative(f, x, dx)
    df = 0;
    for i=1:length(x)
        df = df + diff(f, x(i))*dx(i);
    end
end

function eq = find_euler_lagrange_eqs(L, q, dq, ddq)
    eq = sym(zeros(length(q), 1));

    for i=1:length(q)
        tder = find_total_derivative(diff(L, dq(i)), [q; dq], [dq; ddq]);
        eq(i, 1) =  tder - diff(L, q(i));
    end

    eq = simplify(eq);
end