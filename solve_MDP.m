function output_data = solve_MDP(input_data)
% Filename: solve_MDP.m
% Author: Trung Tran and Ilya Kolmanovsky
% Email: trungbt@umich.edu, ilya@umich.edu
% Copyright: © 2024 Trung Tran and Ilya Kolmanovsky. All rights reserved.
% Update Date: 2024-07-18

% Description: This script is the function to solve the MDP in the 
% computational example in the MECC 2024 conference paper "Stochastic P
% redictive Control with Time-Joint State Constraint Violation 
% Probability Objective for Drone Delivery of Medical Products". 

rng('default');
import casadi.*
which('casadiMEX')

%% Declare variables
global solver

global X nx sx data_x data_x_idx X_ns
% X - the set of states
% nx - the number of states
% sx - the size of states (from Cartesian product)
% data_x - the histories of the states over the simulation time
% data_x_idx - the histories of the states index over the simulation time
% X_ns - the set of constraint violation output - X_ns stands for "X not save"


global U nu su data_u
% U - the set of actions (input)
% nu - the number of actions
% su - the size of actions (from Cartesian product)
% data_u - the histories of the actions over the simulation time

% % global Y ny sy data_y data_y_idx
% % % Y - the set of outputs
% % % ny - the number of outputs
% % % sy - the size of outputs (from Cartesian product)
% % % data_y - the histories of the outputs over the simulation time
% % % data_y_idx - the histories of the outputs index over the simulation time

global P P_mat
% P - a cell, where each entry holds the state transition matrix under the
%     corresponding input u. For example: P{u}(x1, x2) - the probability of
%     transitioning from state x1 to x2 under input u.
% P_mat - another way to represent P based on the state

global C
% C - a cell, where each entry holds the cost of transition under the
%     corresponding input u. For example: C{u}(x1, x2) - the cost of
%     transitioning from state x1 to x2 under input u.


global pi_0_m1
global P_y_given_x
% P_y_given_x - the observation probability

global PI_t pi_t
% pi_t - the information state -- the belief (distribution) of the current
%        state of the system
% PI_t - contains historical values of pi_t

global t % current time begins from 0
t = 0;

%% Unloading variables
X = input_data.X;
nx = input_data.nx;
sx = input_data.sx;
X_ns = input_data.X_ns;
initial_data_x = input_data.X0;
current_state = find(ismember(X, initial_data_x, 'rows')); % initial condition
exclude_cost_constraint = input_data.exclude_cost_constraint;

nu = input_data.nu;

P = input_data.P;
C = input_data.C;
N = input_data.N;
time = input_data.time;
solver = input_data.solver;
method = input_data.method;

%% main code
% Create holder for outputing data
data_u = zeros(length(time), 1);
data_violation_chance = zeros(length(time), 1);
data_cost_constraint = zeros(length(time), 1);

data_x = zeros(length(time), length(sx));
data_x_idx = zeros(length(time), 1);

PI_t = sparse(length(time), nx);
data_gamma = zeros(nu, length(time));

fval_list = zeros(length(time), 1);
exitflag_list = zeros(length(time), 1);
output_list = cell(length(time), 1);
comp_time = zeros(length(time), 1);

%--------------------------------------------------------------------------
% Compile P_mat
tic()
make_P_mat()
toc()
disp('Finish compiling P_mat!')

% Randomizing initial guess
Gamma_t0 = rand(nu, N);
for i = 1:N
    Gamma_t0(:, i) = Gamma_t0(:, i)/sum(Gamma_t0(:, i));
end
x0 = reshape(Gamma_t0, nu*N, 1); %initial guess

% Obtaining equality constraint
[Aeq, beq] = get_eq_constr(N);
lb = zeros(nu*N, 1);
ub = ones(nu*N, 1);

% Finding the maximum state-action-state cost that is not the penalty cost
%----------------------------------------------------------------------
% Modification:
max_cost = 2*N; % for this problem, it is 1*N
%----------------------------------------------------------------------

if strcmp(solver, 'fmincon')
    %fmincon_options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
    fmincon_options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
    
elseif strcmp(solver, 'casadi')
    fun = @(x, p) myfun_mod(x, N, p);
    nonlcon = @(x, p) my_constr_mod_2(x, N, max_cost, p);
    
    opti = casadi.Opti();
    
    var_x = opti.variable(nu*N, 1); % decision variable
    param_p = opti.parameter(1, nx); % parameter
    
    opti.minimize(fun(var_x, param_p));
    opti.subject_to(lb <= var_x <= ub);
    
    if exclude_cost_constraint == 0
        opti.subject_to(nonlcon(var_x, param_p) <= 0);
    end
    
    opti.subject_to(Aeq*var_x == beq);
    opti.solver(method);
    
elseif strcmp(solver, 'casadi_SX')
    var_x = SX.sym('x', nu*N, 1); % decision variable
    param_p = SX.sym('p', 1, nx); % parameter %for pi_t
    
    fun = @(x, p) myfun_mod(x, N, p);
    
    if exclude_cost_constraint == 0
        nonlcon = @(x, p) my_constr_mod_2(x, N, max_cost, p);
        fun_g = [nonlcon(var_x, param_p); Aeq*var_x - beq];
        lbg = [-inf;zeros(size(beq))];
        ubg = [0;zeros(size(beq))];
    else
        fun_g = [Aeq*var_x - beq];
        lbg = [zeros(size(beq))];
        ubg = [zeros(size(beq))];
    end

    nlp = struct('x',var_x, 'p', param_p, 'f', fun(var_x, param_p), 'g', fun_g);
    S = nlpsol('S', method, nlp);
end

for t = 0:1:time(end)
    disp(strcat('time t= ', num2str(t)))
    
    data_x_idx(t+1) = current_state;
    data_x(t+1, :) = X(current_state, :);
    
    %----------------------------------------------------------------------
    % Modification: fully observable state
    %----------------------------------------------------------------------
    pi_t =  current_state_distribution_fully_observable_states(current_state);
    %----------------------------------------------------------------------
    
    PI_t(t+1, :) = pi_t;
    
    if strcmp(solver, 'fmincon')
        fun = @(x) myfun_mod(x, N, pi_t);
        
        nonlcon = @(x) my_constr_mod(x, N, max_cost, pi_t);
        
        tic()
        if exclude_cost_constraint == 0
            [x_sol, fval, exitflag, output] = fmincon(fun, x0, [], [], Aeq, beq, lb, ub, nonlcon, fmincon_options);
        else
            [x_sol, fval, exitflag, output] = fmincon(fun, x0, [], [], Aeq, beq, lb, ub, [], fmincon_options);
        end
        
        comp_time(t+1) = double(toc())
        %pause(1)
        
        fval_list(t+1)= fval;
        exitflag_list(t+1) = exitflag;
        output_list{t+1} = output;
        disp('Done!')
        
        %if exitflag == 1
        %    pause(1)
        %end
        
        data_violation_chance(t+1) = fun(x_sol);
        data_cost_constraint(t+1) = nonlcon(x_sol);
        
    elseif strcmp(solver, 'casadi')
        % Solve using CasaDi ----------------------------------------------
        opti.set_value(param_p, pi_t)
        opti.set_initial(var_x, x0);
        
        tic()
        sol = opti.solve();
        comp_time(t+1) = double(toc())
        x_sol = sol.value(var_x);
        %param_p_temp = sol.value(param_p);
        
        %fval_list(t+1)= fval;
        exitflag_list(t+1) = sol.stats.success;
        %output_list{t+1} = output;
        disp('Done!')
        
        data_violation_chance(t+1) = sol.value(fun(var_x, param_p));
        data_cost_constraint(t+1) = sol.value(nonlcon(var_x, param_p));
        
        % -----------------------------------------------------------------
    elseif strcmp(solver, 'casadi_SX')

        tic()
        sol = S('x0',x0,'p',pi_t,'lbx',0,'ubx',1,'lbg',lbg,'ubg',ubg);
        comp_time(t+1) = double(toc())
        disp('Done!')
        
        x_sol = full(sol.x);
        data_violation_chance(t+1) = full(sol.f);
        data_cost_constraint(t+1) = full(sol.g(1));
        
    end
    
    x0 = [x_sol(nu+1 : end); (1/nu)*ones(nu, 1)]; % warm starting next step
    Gamma_t_sol = reshape(x_sol, nu, N);
    
    % Choose an input based on pmf of the solution
    pmf_array_input = Gamma_t_sol(:, 1)';
    data_gamma(:, t+1) = pmf_array_input;
    current_input = sample_from_pmf(pmf_array_input);
    data_u(t+1) = current_input;
    
    % Propagate the dynamic based on the chosen input
    pmf_array_states = P{current_input}(data_x_idx(t+1), :);
    current_state = sample_from_pmf(pmf_array_states);
end

%% Compiling output data
output_data = struct();
output_data.data_x = data_x;
output_data.data_x_idx = data_x_idx;
output_data.PI_t = PI_t;
output_data.data_violation_chance = data_violation_chance;
output_data.data_cost_constraint = data_cost_constraint;

output_data.data_u = data_u;
output_data.data_gamma = data_gamma;

output_data.fval_list = fval_list;
output_data.exitflag_list = exitflag_list;
output_data.comp_time = comp_time;

end

%% Support Functions
%%-------------------------------------------------------------------------
% GENERAL FUNCTIONS
%%-------------------------------------------------------------------------

function output_pi_t = current_state_distribution_fully_observable_states(current_state)
global nx
% fully observable states
output_pi_t = sparse(1, nx);
output_pi_t(current_state) = 1;
end

% This function create the P_mat cell (for state propagation).
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function make_P_mat()
global nx P nu P_mat

P_mat = {};
for x = 1:nx
    disp(x)
    P_mat_temp = zeros(nx, nu);
    
    for u = 1:nu
        P_mat_temp(:, u) = P{u}(:, x);
    end
    P_mat{x} = P_mat_temp;
end

end

% -------------------------------------------------------------------------

% This function calculates the predicted state distribution.
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function PI_tau = predicted_state_distribution(Gamma_t, pi_t)
global nx P_mat solver
import casadi.*

% Gamma_t is nu x N
size_Gamma_t = size(Gamma_t);
N = size_Gamma_t(2);


if strcmp(solver, 'fmincon')
    % PI_tau is (N+1) x nx, where the first row is pi_t, and the last
    % row is pi_(t+N)_t
    PI_tau = zeros(N+1, length(pi_t));
    
    PI_tau(1, :) = pi_t;
    pi_tau = pi_t;
    
    for n = 1:N
        pi_taup1 = zeros(1, nx);
        
        for x = 1 : nx
            pi_taup1(x) = pi_tau * P_mat{x} * Gamma_t(:, n);
        end
        
        PI_tau(n+1, :) = pi_taup1;
        pi_tau = pi_taup1;
    end
    
elseif strcmp(solver, 'casadi')
    % PI_tau is (N+1) x nx, where the first row is pi_t, and the last
    % row is pi_(t+N)_t
    
    PI_tau = MX(N+1, length(pi_t));
    
    PI_tau(1, :) = pi_t;
    pi_tau = pi_t;
    
    for n = 1:N
        pi_taup1 = MX(1, nx);
        
        for x = 1 : nx
            pi_taup1(x) = pi_tau * P_mat{x} * Gamma_t(:, n);
        end
        
        PI_tau(n+1, :) = pi_taup1;
        pi_tau = pi_taup1;
    end

elseif strcmp(solver, 'casadi_SX')
    % PI_tau is (N+1) x nx, where the first row is pi_t, and the last
    % row is pi_(t+N)_t
    
    PI_tau = SX(N+1, length(pi_t));
    
    PI_tau(1, :) = pi_t;
    pi_tau = pi_t;
    
    for n = 1:N
        pi_taup1 = SX(1, nx);
        
        for x = 1 : nx
            pi_taup1(x) = pi_tau * P_mat{x} * Gamma_t(:, n);
        end
        
        PI_tau(n+1, :) = pi_taup1;
        pi_tau = pi_taup1;
    end
    
end

end

% Note: new predicted_state_distribution 
function pi_taup1 = predicted_state_distribution_2(pi_tau, gamma_tau)
global solver P_mat nx
import casadi.*

if strcmp(solver, 'fmincon')
    pi_taup1 = zeros(1, nx);
    for x = 1 : nx
        pi_taup1(x) = pi_tau * P_mat{x} * gamma_tau;
    end   
 
elseif strcmp(solver, 'casadi')
    pi_taup1 = MX(1, nx);
    for x = 1 : nx
        pi_taup1(x) = pi_tau * P_mat{x} * gamma_tau;
    end
    
elseif strcmp(solver, 'casadi_SX')
    pi_taup1 = SX(1, nx);
    for x = 1 : nx
        pi_taup1(x) = pi_tau * P_mat{x} * gamma_tau;
    end
end

end

%-------------------------------------------------------------------------
% This function calculates the expected cost
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
% Note: old expected cost
function cost = expected_cost(Gamma_t, pi_t)
global nu C P solver

% Gamma_t is nu x N
size_Gamma_t = size(Gamma_t);
N = size_Gamma_t(2);

% ---------------------------------------------------------------------
% Solver for fmincon
%----------------------------------------------------------------------
if strcmp(solver, 'fmincon')    
    % PI_tau is (N+1)*nx
    PI_tau = predicted_state_distribution(Gamma_t, pi_t);
    
    % Calculating the expected cost
    temp_mat = zeros(nu, N);
    for u = 1:nu
        for n = 1:N
            % PI_tau(n, :) is the predicted_state_distribution at time t+n
            % and has dim (1, nx)
            % C{u}.*P{u} is the cost times the transition probability under
            % action u.
            temp_mat(u, n) = sum(PI_tau(n, :)*(P{u}.*C{u}));
        end
    end
    
    cost = sum(temp_mat.*Gamma_t, 'all');
end

% ---------------------------------------------------------------------
% Solver for casadi
%----------------------------------------------------------------------
if strcmp(solver, 'casadi')
    % PI_tau is (N+1)*nx
    PI_tau = predicted_state_distribution(Gamma_t, pi_t);
    
    % Calculating the expected cost
    temp_mat = MX(nu, N);
    for u = 1:nu
        for n = 1:N
            % PI_tau(n, :) is the predicted_state_distribution at time t+n
            % and has dim (1, nx)
            % C{u}.*P{u} is the cost times the transition probability under
            % action u.
            temp_mat(u, n) = sum(PI_tau(n, :)*(P{u}.*C{u}));
        end
    end
    
    cost = sum(temp_mat.*Gamma_t, 'all');

%----------------------------------------------------------------------
end
end

% Note: new expected cost (should give the same result)
function cost = expected_cost_2(Gamma_t, pi_t)
    global nu C P solver
    import casadi.*

    % Gamma_t is nu x N
    size_Gamma_t = size(Gamma_t);
    N = size_Gamma_t(2);
    
    if strcmp(solver, 'fmincon')
        % Calculating the expected cost
        cost = 0;
        pi_tau = pi_t;
    
        for n = 1:N
            for u = 1:nu
                % pi_tau is the predicted_state_distribution at time t+n and has dim (1, nx)
                % C{u}.*P{u} is the cost times the transition probability under action u.
                temp_cost = Gamma_t(u, n) * sum(pi_tau*(P{u}.*C{u}));
            end
            cost = cost + temp_cost;
            pi_taup1 = predicted_state_distribution_2(pi_tau, Gamma_t(:, n));
            pi_tau = pi_taup1;
        end
 
    elseif strcmp(solver, 'casadi')
        % Calculating the expected cost
        cost = MX(1);
        pi_tau = pi_t;
    
        for n = 1:N
            for u = 1:nu
                % pi_tau is the predicted_state_distribution at time t+n and has dim (1, nx)
                % C{u}.*P{u} is the cost times the transition probability under action u.
                temp_cost = Gamma_t(u, n) * sum(pi_tau*(P{u}.*C{u}));
                %u
            end
            cost = cost + temp_cost;
            pi_taup1 = predicted_state_distribution_2(pi_tau, Gamma_t(:, n));
            pi_tau = pi_taup1;
        end
        
    elseif strcmp(solver, 'casadi_SX')
        % Calculating the expected cost
        cost = SX(1);
        pi_tau = pi_t;
    
        for n = 1:N
            for u = 1:nu
                % pi_tau is the predicted_state_distribution at time t+n and has dim (1, nx)
                % C{u}.*P{u} is the cost times the transition probability under action u.
                temp_cost = Gamma_t(u, n) * sum(pi_tau*(P{u}.*C{u}));
                %u
            end
            cost = cost + temp_cost;
            pi_taup1 = predicted_state_distribution_2(pi_tau, Gamma_t(:, n));
            pi_tau = pi_taup1;
        end
    end
end

%-------------------------------------------------------------------------
% This function calculates the joint-chance of constraint violation
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function prob = chance_constr(Gamma_t, pi_t)
global X_ns P nx nu X solver
import casadi.*

% Gamma_t is nu x N
size_Gamma_t = size(Gamma_t);
N = size_Gamma_t(2);

% ---------------------------------------------------------------------
% Solver for fmincon
%----------------------------------------------------------------------
if strcmp(solver, 'fmincon')
    P_x_tau_V_bar_taum1 = zeros(N+1, nx);
    P_x_tau_V_bar_tau = zeros(N+1, nx);
    P_V_tau = zeros(N+1, 1);
    
elseif strcmp(solver, 'casadi')
    P_x_tau_V_bar_taum1 = MX(N+1, nx);
    P_x_tau_V_bar_tau = MX(N+1, nx);
    P_V_tau = MX(N+1, 1);
elseif strcmp(solver, 'casadi_SX')
    P_x_tau_V_bar_taum1 = SX(N+1, nx);
    P_x_tau_V_bar_tau = SX(N+1, nx);
    P_V_tau = SX(N+1, 1);
end
    
    P_V_tau(1) = 0;
    P_x_tau_V_bar_tau(1, :) = pi_t;
    
    % for tau = t+1 to t+N
    for tau = 2:(N+1)
        temp_vec = P_x_tau_V_bar_tau(tau-1, :); % row vector
        
        %(eqn 37)
        temp_mat = sparse(nx, nx);
        for u = 1:nu
            temp_mat = temp_mat + P{u}*Gamma_t(u, tau-1);
            %u
        end
        
        %(eqn 36b)
        temp_result = temp_vec*temp_mat;
        P_x_tau_V_bar_taum1(tau, :) = temp_result;
        
        %(eqn 36a)
        P_V_tau(tau) = P_V_tau(tau-1) + sum(temp_result.*double(X_ns == 1));
        
        %(eqn 36c)
        if strcmp(solver, 'fmincon')
            temp_result_2 = sparse(1, nx);
        elseif strcmp(solver, 'casadi')
            temp_result_2 = MX(1, nx);
        elseif strcmp(solver, 'casadi_SX')
            temp_result_2 = SX(1, nx);
        end
        
        for x = 1:nx
            if X_ns(x) == 0
                temp_result_2(x) = P_x_tau_V_bar_taum1(tau, x);
            end
        end
        P_x_tau_V_bar_tau(tau, :) = temp_result_2;
    end
    
    prob = P_V_tau(end);

end

% -------------------------------------------------------------------------
% These are wrapper functions for the objective functions
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function cost = myfun(x, N, pi_t)
% x - each element of Gamma_t
global nu
Gamma_t = reshape(x, nu, N);
cost = expected_cost(Gamma_t, pi_t);
end

% Note: [PASS INTERNAL REVIEW: Feb 28, 2024]
function cost = myfun_mod(x, N, pi_t)
% x - each element of Gamma_t
global nu
Gamma_t = reshape(x, nu, N);
cost = chance_constr(Gamma_t, pi_t);
end

% -------------------------------------------------------------------------
% These are wrapper functions for inequality constraints
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function [c, ceq] = my_constr(x, N, epsilon, pi_t)
% x - each element of Gamma_t
global nu
Gamma_t = reshape(x, nu, N);

c = chance_constr(Gamma_t, pi_t) - epsilon;
ceq = [];
end

% Note: [PASS INTERNAL REVIEW: Feb 28, 2024]
function [c, ceq] = my_constr_mod(x, N, max_cost, pi_t)
% x - each element of Gamma_t
global nu
Gamma_t = reshape(x, nu, N);
c = expected_cost_2(Gamma_t, pi_t) - max_cost;
ceq = [];
end

% Note: [PASS INTERNAL REVIEW: Feb 28, 2024]
function [c] = my_constr_mod_2(x, N, max_cost, pi_t)
% x - each element of Gamma_t
global nu
Gamma_t = reshape(x, nu, N);
c = expected_cost_2(Gamma_t, pi_t) - max_cost;
end

% -------------------------------------------------------------------------

% This function calculate the equality constraints to be used for fmincon
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function [Aeq, beq] = get_eq_constr(N)
% N is the time horizon
global nu

% Sum of every column of Gamma_t must be 1
Aeq = zeros(N, nu*N);
for n = 1:N
    start_idx = nu*(n-1)+1;
    end_idx = start_idx + nu - 1;
    Aeq(n, start_idx:end_idx) = 1;
end

beq = ones(N, 1);
end

% This function is used for generating a action given a pmf array of action
% with probabilities
% Note: [PASS INTERNAL REVIEW: June 29, 2023]
function sample_idx = sample_from_pmf(pmf_array)
% Create cdf_array
cdf_array = zeros(size(pmf_array)); % pmf_array is a row vector
cdf_array(1) = pmf_array(1);
for i = 2:length(pmf_array)
    cdf_array(i) = cdf_array(i-1) + pmf_array(i);
end
cdf_array = [0, cdf_array];

% Randomly generate 1 sample between 0 and 1
x = rand;
sample_idx = find(cdf_array < x, 1, 'last');
end

