clc; clear all; close all; format compact;
addpath('casadi-3.6.5-windows64-matlab2018b')
import casadi.*
which('casadiMEX')

rng('default');
load('paper_results\MDP_data.mat')

% Filename: main.m
% Author: Trung Tran and Ilya Kolmanovsky
% Email: trungbt@umich.edu, ilya@umich.edu
% Copyright: © 2024 Trung Tran and Ilya Kolmanovsky. All rights reserved.
% Update Date: 2024-07-18

% Description: This script is the main script to run the computational
% example from the MECC 2024 conference paper "Stochastic Predictive 
% Control with Time-Joint State Constraint Violation Probability Objective 
% for Drone Delivery of Medical Products". There are several sections of 
% the code:

% Section 1 - Loading original data: this section load geographical data
% for the locations of the depots, local hospitals, etc.

% Section 2 - Further discretization of geographical data: this section 
% further discretize the geographical data so that every link has the same 
% amount of distance; this, as a result, takes into account the time. This
% section is currently commented out, due to scalability issue.

% Section 3 - Visualizing the data using geoplot

% Section 4 - Defining states, actions, transitions, costs, and
% constraints: this section compiles the basic elements of MDP problem. For
% large MDP problem, this takes a long time to compile, so it is advised
% that we save the result of this section once the run is finished.
% -------------------------------------------------------------------------
% Note: To change the MDP settings, e.g. drone's delivery capacities, 
% battery levels, stock levels at service locations, demand dynamics, etc.,
% please change the 'Defining the states' subsection.
%--------------------------------------------------------------------------

% Section 5 - Checking if transition matrix passed the requirement

% Section 6 - Solve MDP. The section calls the solve_MDP() function
% (included in the file). To quickly get started or debug, it is suggested 
% that the data is loaded (e.g., MDP_data.mat), and Section 6-7 are run 
% directly, bypassing Sections 1-5.
% -------------------------------------------------------------------------
% Note: To change the run settings, e.g. solver options, time horizon, 
% initial condition, please change the 'Control panel' subsection. Note
% that the option 'casadi_SX' (symbolic) takes a bit long to complile, but
% once that is done, fast computations can be achieved.
%--------------------------------------------------------------------------

% Section 7 - Plot solution: visualize the result

% %% Section 1 - Loading original geographical data
% % -------------------------------------------------------------------------
% % Note: Data should be ordered so that the service points come first, and
% % then the depot locations. This is our convention for now.
% %--------------------------------------------------------------------------
% 
% raw_data = readtable('geo_data.csv');
% data_long = raw_data.('LONG');
% data_lat = raw_data.('LAT');
% data_flag = raw_data.('FLAG'); % 0 - service point; 1 - depot;
% data_name = raw_data.('NAME');
% 
% % Create a WGS84 reference ellipsoid with 'km' length unit.
% wgs84 = wgs84Ellipsoid("km"); 
% 
% % Find the distance between any locations
% dist_mat = zeros(length(data_lat)); %km
% 
% for i = 1:length(data_lat)
%     for j = 1:length(data_lat)
% 
%         lat1 = data_lat(i);
%         long1 = data_long(i);
% 
%         lat2 = data_lat(j);
%         long2 = data_long(j);
%         dist_mat(i,j) = distance(lat1,long1, lat2,long2, wgs84);
%     end
% end
% 
% dist_mat = round(dist_mat/30); % get whole number of multiple of 25km
% connectivity_graph = ones(size(dist_mat));
% original_connectivity_graph = connectivity_graph;
% 
% % For the service location, staying at the location at one time step
% % requires flying, hence, we need to modify the distance to itself
% for i = 1:length(dist_mat)
%     if data_flag(i) ~=1
%         dist_mat(i, i) = 1;
%     end
% end
% 
% disp('Done with map creation!')
% 
% %% Section 2 - Further discretization of geographical data
% % -------------------------------------------------------------------------
% % Note: This part of the code is to further divide the links into smaller
% % links with equal length. Doing this will take into account the transition
% % time, but the trade-off is increasing the size of the MDP. The
% % scalibility problem of MDP will be addressed in future work, as noted in 
% % the discussion section in the paper.
% %--------------------------------------------------------------------------
% 
% % connect_lat = [];
% % connect_long = [];
% % connect_flag = [];
% % connect_name = {};
% % 
% % % extend distance matrix
% % for i = 1:length(data_lat)
% %     for j = i+1:length(data_lat)
% %         temp_dist = dist_mat(i, j);
% %         
% %         if temp_dist == 1
% %             continue
% %         else
% %             connectivity_graph(i, j) = 0;
% %             connectivity_graph(j, i) = 0;
% %             dist_mat(i, j) = NaN;
% %             dist_mat(j, i) = NaN;
% %             
% %             for k = 1:temp_dist-1
% %                 % interpolate a point
% %                 alpha = k/temp_dist;
% %                 temp_connect_lat = (1-alpha)*data_lat(i) + alpha*data_lat(j);
% %                 temp_connect_long = (1-alpha)*data_long(i) + alpha*data_long(j);
% %                 
% %                 connect_lat = [connect_lat; temp_connect_lat];
% %                 connect_long =[connect_long; temp_connect_long];
% %                 connect_flag = [connect_flag; 0.5];
% %                 connect_name{end+1, 1} = [];
% %                 
% %                 % change connectivity graph
% %                 connectivity_graph = blkdiag(connectivity_graph, 1); %padded with 0 with 1 as diag
% %                 dist_mat = blkdiag(dist_mat, 0); %padded with NaN with 0 as diag
% %                 dist_mat(end, 1:end-1) = NaN;
% %                 dist_mat(1:end-1, end) = NaN;
% %                                 
% %                 if k == 1
% %                     connectivity_graph(i, end) = 1;
% %                     connectivity_graph(end, i) = 1;
% %                     dist_mat(i, end) = 1;
% %                     dist_mat(end, i) = 1;
% %                 end
% %                 
% %                 if k == temp_dist-1
% %                     connectivity_graph(end, j) = 1;
% %                     connectivity_graph(j, end) = 1;
% %                     dist_mat(end, j) = 1;
% %                     dist_mat(j, end) = 1;
% %                 end
% %                 
% %                 if k>1
% %                     connectivity_graph(end-1, end) = 1;
% %                     connectivity_graph(end, end-1) = 1;
% %                     dist_mat(end-1, end) = 1;
% %                     dist_mat(end, end-1) = 1;
% %                 end                    
% %             end
% %         end
% %     end
% % end
% % 
% % data_lat = [data_lat; connect_lat];
% % data_long = [data_long; connect_long];
% % data_flag = [data_flag; connect_flag]; %1 - depot; 0 - service point; 0.5 - connection
% % data_name = [data_name; connect_name];
% % 
% % % For the service location, staying at the location at one time step
% % % requires flying, hence, we need to modify the distance to itself
% % for i = 1:length(dist_mat)
% %     if data_flag(i) ~=1
% %         dist_mat(i, i) = 1;
% %     end
% % end

%% Section 3 - Visualizing the data using geoplot
service_color = [0.8500 0.3250 0.0980]; %orange
depot_color = [0 0.4470 0.7410]; %blue
connect_color = [0 0 0]; %black

figure
for i = 1:length(original_connectivity_graph)
    for j = i+1:length(original_connectivity_graph)
        geoplot(data_lat([i j]), data_long([i j]),"k", 'Color', [.65 .65 .65])

        if i==1 && j == 2
            hold on
        end

    end
end

geoplot(data_lat(data_flag==0), data_long(data_flag==0),...
    "o", 'color', service_color)
geoplot(data_lat(data_flag==1), data_long(data_flag==1),...
    "o", 'color', depot_color)
geoplot(data_lat(data_flag==0.5), data_long(data_flag==0.5),...
    "o", 'color', connect_color)

geolimits([min(data_lat) - 0.07, max(data_lat) + 0.07],...
          [min(data_long) - 0.07, max(data_long) + 0.07]);

geobasemap 'streets-light'

for i = 1:length(data_lat)
    if data_flag(i) == 0
        temp_color = service_color;
    elseif data_flag(i) == 0.5
        temp_color = connect_color;
    else
        temp_color = depot_color;
    end

    text(data_lat(i)+0.025, data_long(i)+0.025, data_name(i),...
        'FontSize', 10, 'FontWeight', 'bold', 'color', temp_color) 
    text(data_lat(i)-0.025, data_long(i)-0.025, num2str(i),...
        'FontSize', 10, 'FontWeight', 'bold', 'color', temp_color)
end

saveas(gcf,'results/map.jpg')
saveas(gcf,'results/map.fig')

% %% Section 4 - Defining states, actions, transitions, costs, and constraints
% LN = 1000; % large number for penalty
% 
% %%-------------------------------------------------------------------------
% % Defining the states
% %%-------------------------------------------------------------------------
% D = [0, 1]; % Delivery capacity levels of the drone
% max_D = max(D);
% nd = length(D);
% 
% B = [-1, (0:1:7)]'; % Battery levels (remaning range) of the drone
% max_B = max(B);
% nb = length(B);
% 
% nv_bar = length(dist_mat); % number of vertices
% 
% Li = [0, 1, 2]; % Medical stock levels at each service location
% max_Li = max(Li);
% nli = length(Li);
% global pli
% pli = 0.06;
% 
% % -------------------------------------------------------------------------
% % Note: hard coded for the current number of service point.
% % Change if new map is considered.
% %--------------------------------------------------------------------------
% sx = [nd, nb, nv_bar, nli, nli, nli, nli];
% %--------------------------------------------------------------------------
% 
% nx = prod(sx);
% 
% X = zeros(nx, 7);
% for x = 1:nx
%     X(x, :) = state_ind2sub(x, sx);
% end
% 
% %%-------------------------------------------------------------------------
% % Defining the actions
% %%-------------------------------------------------------------------------
% nu = max_D + nv_bar;
% % if u is in 1:maxD -- delivery amount; we remove the u = 0, because it is a dummy action
% % if u is in maxD+1:maxD+nv_tilde -- movement to vertex (u - maxD)
% 
% %%-------------------------------------------------------------------------
% % Defining constraint violation space
% %%-------------------------------------------------------------------------
% % The constraints is X - X_ns, where X_ns (X "not safe") is:
% X_ns = zeros(1, nx);
% for i = 1:nx
%     x = X(i, :);
%     dx = x(1);
%     bx = x(2);
%     vx = x(3);
%     l1x = x(4);
%     l2x = x(5);
%     l3x = x(6);
%     l4x = x(7);
% 
%     if (bx == -1) || (l1x == 0) || (l2x == 0) || (l3x == 0) || (l4x == 0)
%         X_ns(i) = 1;
%     end
% end
% 
% %%-------------------------------------------------------------------------
% % Assembling state transition matrix and cost matrix
% %%-------------------------------------------------------------------------
% % In this section, x and y represent the states.
% P = {};
% C = {};
% 
% for u = 1:nu
%     mini_Pxay = sparse(nx, nx); % state transition matrix according to the action
%     mini_Cxay = sparse(nx, nx); % cost matrix according to the action
% 
%     for i = 1:nx
%         disp(i);
%         x = X(i, :); % = [dx, bx, vx, l1x, l2x]
% 
%         dx = x(1);
%         bx = x(2);
%         vx = x(3);
%         flag_x = data_flag(vx);
%         l1x = x(4);
%         l2x = x(5);
%         l3x = x(6);
%         l4x = x(7);
% 
%         for j = 1:nx
%             y = X(j, :); % = [dy, by, vy, l1y, l2y]
% 
%             dy = y(1);
%             by = y(2);
%             vy = y(3);
%             flag_y = data_flag(vy);
%             l1y = y(4);
%             l2y = y(5);
%             l3y = y(6);
%             l4y = y(7);
% 
%             % For debugging
%             %if (u==1)&&(i==49)&&(j==49)
%             %    pause(1)
%             %end
% 
%             % Delivery probabilities --------------------------------------
%             if (u > max_D) && (dx == dy)
%                 % (u > max_D): if u is a movement action
%                 % movement action does not changes vehicle's stock level
%                 Pd = 1;
% 
%             elseif (u <= max_D)
%                 % (u <= max_D): if u is delivering action
% 
%                 if (flag_x == 1)
%                     % (flag_x == 1): if current node is the depot
%                     if  dy == max_D % maximize the delivery level
%                         Pd = 1;
%                     else
%                         continue
%                     end
% 
%                 elseif (flag_x == 0)
%                     % (flag_x == 0): if current node is the service point
%                     % delivering deplete the vehicle's capacity
%                     if  dy == max([0, dx - u]) 
%                         % dx-dy has to match the amount delivered
%                         Pd = 1;
%                     else
%                         continue
%                     end     
% 
%                 elseif (flag_x == 0.5)&&(dy == dx)
%                     % (flag_x == 0.5): if current node is a connecting node
%                     % delivering action does not changes vehicle's stock level
%                     Pd = 1;
%                 else
%                     continue
%                 end  
%             else
%                 continue
%             end
% 
%             % Battery transition probabilities ----------------------------
%             if (u <= max_D) && (bx ~= -1)
%                 % (u <= max_D): if u is delivering action
%                 % (bx ~= -1): if we haven't run out of charge
% 
%                 if (flag_x ~= 1) && (by == max([-1, bx - dist_mat(vx, vx)]))
%                     % (flag_x ~= 1): if current node is a service point or a connecting node
%                     % delivering cost some the battery when flying
%                     Pb = 1;
% 
%                 elseif (flag_x == 1) && (by == max_B)
%                      % refilling at depot refill the battery to full
%                      Pb = 1;
%                 else
%                     continue
%                 end
% 
%             elseif(u > max_D) && (bx ~= -1)
%                 % (u > max_D): if u is a movement action
%                 % (bx ~= -1): if we haven't run out of charge
%                 temp_bat_consumption = dist_mat(vx, u - max_D);
% 
%                 if isnan(temp_bat_consumption)
%                     % if the drone is attempting to move to an unconnected
%                     % vertex, then it stay where it is and consume some energy 
%                     temp_bat_consumption = dist_mat(vx, vx);
%                     if (by == max([-1, bx - temp_bat_consumption]))
%                         Pb = 1;
%                     else
%                         continue
%                     end
% 
%                 elseif (by == max([-1, bx - temp_bat_consumption]))
%                     Pb = 1;
%                 else
%                     continue
%                 end
% 
%             elseif (bx == -1) && (by == -1) && (vy == vx)
%                 Pb = 1;
%             else
%                 continue
%             end
% 
%             % Movement transition probabilities ----------------------------
%             if (u <= max_D) && (vx == vy)
%                 % (u <= max_D): if u is delivering action
%                 Pm = 1;
% 
%             elseif (u > max_D)
%                 % (u > max_D): if u is a movement action
%                 temp = u - max_D;
%                 if (connectivity_graph(vx, temp) == 1) && (bx ~= -1) && (temp == vy)
%                     Pm = 1;
%                 elseif (connectivity_graph(vx, temp) == 1) && (bx == -1) && (vx == vy)
%                     Pb = 1;
%                 elseif (connectivity_graph(vx, temp) == 0) && (vx == vy)
%                     Pm = 1;
%                 else
%                     continue
%                 end
% 
% 
%             else
%                 continue
%             end
% 
%             % Evolving probabilities --------------------------------------
%             % -------------------------------------------------------------
%             % Note: hard cod   ed for the current number of service point.
%             % Change if new map is considered.
%             %--------------------------------------------------------------
%             if (u <= max_D) && (vx == 1)
%                 % (u <= max_D): if u is delivering action
%                 % (vx == 1): service point #1
%                 temp_l1y = min([max_Li, l1x + u, l1x + dx]);
%                 if l1y == temp_l1y
%                     P1 = 1;
%                 else
%                     continue
%                 end
%             elseif (u > max_D) || (vx ~= 1)
%                 P1 = Peli(l1y, l1x);
%             else
%                 continue
%             end
% 
%             if (u <= max_D) && (vx == 2)
%                 % (u <= max_D): if u is delivering action
%                 % (vx == 2): service point #2
%                 temp_l2y = min([max_Li, l2x + u, l2x + dx]);
%                 if l2y == temp_l2y
%                     P2 = 1;
%                 else
%                     continue
%                 end
%             elseif (u > max_D) || (vx ~= 2)
%                 P2 = Peli(l2y, l2x);
%             else
%                 continue
%             end
% 
%             if (u <= max_D) && (vx == 3)
%                 % (u <= max_D): if u is delivering action
%                 % (vx == 3): service point #3
%                 temp_l3y = min([max_Li, l3x + u, l3x + dx]);
%                 if l3y == temp_l3y
%                     P3 = 1;
%                 else
%                     continue
%                 end
%             elseif (u > max_D) || (vx ~= 3)
%                 P3 = Peli(l3y, l3x);
%             else
%                 continue
%             end
% 
%             if (u <= max_D) && (vx == 4)
%                 % (u <= max_D): if u is delivering action
%                 % (vx == 4): service point #4
%                 temp_l4y = min([max_Li, l4x + u, l4x + dx]);
%                 if l4y == temp_l4y
%                     P4 = 1;
%                 else
%                     continue
%                 end
%             elseif (u > max_D) || (vx ~= 4)
%                 P4 = Peli(l4y, l4x);
%             else
%                 continue
%             end
% 
%             % Saving to state transition matrix 
%             mini_Pxay(i,j) = Pd*Pb*Pm*P1*P2*P3*P4; 
% 
%             %--------------------------------------------------------------
%             % Making cost matrix
%             %--------------------------------------------------------------
%             if (u <= max_D) && (flag_x ~= 0.5)
%                 % (u <= max_D): if u is delivering action
%                 % (flag_x ~= 0.5): current state is not in connecting node
%                 mini_Cxay(i,j) = (bx - by); % only take into account the battery
% 
%             elseif (u > max_D) && (connectivity_graph(vx, u-max_D) == 1)
%                 % (u > max_D): if u is movement action
%                 % (connectivity_graph(vx, u-max_D) == 1): current state is connected to u
%                 mini_Cxay(i,j) = (bx - by); % only take into account the battery
% 
%             elseif (u <= max_D) && (flag_x == 0.5)
%                 mini_Cxay(i,j) = LN;
% 
%             elseif (u > max_D) && (connectivity_graph(vx, u-max_D) == 0)
%                 mini_Cxay(i,j) = LN;
% 
%             end
%             %--------------------------------------------------------------
%         end
% 
%         if mod(i, 1000) == 0
%             clc
%         end
%     end
%     P{u} = mini_Pxay;
%     C{u} = mini_Cxay;
% end
% 
% disp('Done with P and C cells!')
% 
% %% Section 5 - Checking if transition matrix passed the requirement
% for u = 1:nu
%     temp = sum(P{u}, 2);
%     temp = full(temp);
%     temp2 = find(abs(temp-1) >= 0.0001);
%     u
%     if length(temp2) == 0
%         disp('State transition matrix pass!')
%     end
% end


%% Section 6 - Solve MDP
%--------------------------------------------------------------------------
% Control panel
%--------------------------------------------------------------------------
solver = 'casadi_SX'; %'fmincon' or 'casadi' or 'casadi_SX'
method = 'ipopt'; % 'ipopt', 'sqpmethod', 'scpgen', 'blocksqp'
exclude_cost_constraint = 1;

time = 0:1:25; % the time histories
N = 10; % the time horizon

% Initial state condition
initial_data_x = [max_D, max_B, 5, max_Li-1, max_Li, max_Li-1, max_Li];

%--------------------------------------------------------------------------
% Adding data into structs
%--------------------------------------------------------------------------
input_data = struct();
input_data.X = X;
input_data.nx = nx;
input_data.sx = sx;
input_data.X_ns = X_ns;
input_data.X0 = initial_data_x;

input_data.nu = nu;

input_data.P = P;
input_data.C = C;
input_data.N = N;
input_data.time = time;
input_data.solver = solver;
input_data.method = method;
input_data.exclude_cost_constraint = 1;

%--------------------------------------------------------------------------
% Solving MDP
%--------------------------------------------------------------------------
output_data = solve_MDP(input_data);

%--------------------------------------------------------------------------
% Collecting output data
%--------------------------------------------------------------------------
data_x = output_data.data_x;
data_x_idx = output_data.data_x_idx;
PI_t = output_data.PI_t;
data_violation_chance = output_data.data_violation_chance;
data_cost_constraint = output_data.data_cost_constraint;

data_u = output_data.data_u;
data_gamma = output_data.data_gamma;

fval_list = output_data.fval_list;
exitflag_list = output_data.exitflag_list;
comp_time = output_data.comp_time;

%% Section 7 - Plot solution
% Figure 1.1
tick_names_vertex = cell(nu, 1);
for v_bar = 1:nv_bar
    if ~isempty(data_name{v_bar})
            tick_names_vertex{v_bar} = strcat([num2str(v_bar),  ' (', data_name{v_bar}, ')']);
    else
            tick_names_vertex{v_bar} = strcat([num2str(v_bar)]);
    end
end
figure
subplot(3,1,1)
plot(time, data_x(:, 3))
set(gca,'ytick',[1:nv_bar],'yticklabel',tick_names_vertex)
title('Drone''s Location')
grid on
xlim([0, length(time)-1])
ylim([0.5, nv_bar+0.5])
%xlabel('time step')

% Figure 1.2
tick_names_action = cell(nu, 1);
for u = 1:nu
    if u <= max_D
        tick_names_action{u} = strcat('d=', num2str(u));
    else 
        if ~isempty(data_name{u-max_D})
            tick_names_action{u} = strcat(['fly to: ', num2str(u-max_D),  '(', data_name{u-max_D}, ')']);
        else
            tick_names_action{u} = strcat(['fly to: ', num2str(u-max_D)]);
        end
    end
end
subplot(3,1,2)
plot(time, data_u)
set(gca,'ytick',[1:nu],'yticklabel',tick_names_action)
title('Action')
grid on
xlim([0, length(time)-1])
ylim([0.5, nu+0.5])
%xlabel('time step')

% Figure 1.3
subplot(3,1,3)
plot(time, data_violation_chance)
title('Time-Joint Constraint Violation Probability')
hold on
yline(1, 'r--')
yline(0, 'b--')
grid on
xlim([0, length(time)-1])
ylim([0-0.05, 1+0.05])
xlabel('time step')
ylabel('probability')

saveas(gcf,'results/vertex.jpg')
saveas(gcf,'results/vertex.fig')

% Figure 2.1
figure
subplot(2, 1, 1)
plot(time, data_x(:,4), '--b')
hold on
plot(time, data_x(:,5), '--r')
plot(time, data_x(:,6), '--m')
plot(time, data_x(:,7), '--g')
legend('demand level at 1 (Mayange)',...
       'demand level at 2 (Ruli)',...
       'demand level at 3 (Cyamuhinda)',...
       'demand level at 4 (Nyanza)',...
       'Location','best')
title('Stock Levels at Local Healthcare Facilities')
ylim([0-0.5, max_Li + 0.5])
grid on
xlim([0, length(time)-1])
ylabel('level')
%xlabel('time step')

% Figure 2.2
subplot(2, 1, 2)
plot(time, data_x(:,2))
hold on
plot(time, data_x(:,1), '--')
yline(-1, 'k:')
yline(max_B, 'k:')
yline(0, 'k:')
hold off
title('Drone''s Battery and Product Levels')
grid on
ylabel('level')
ylim([-2.5-0.05, max_B+0.55])
xlim([0, length(time)-1])
xlabel('time step')
legend('battery level', 'product level','','','')

saveas(gcf,'results/level.jpg')
saveas(gcf,'results/level.fig')

rmpath('casadi-3.6.5-windows64-matlab2018b')

%% Support functions
%%-------------------------------------------------------------------------
% PROBLEM-SPECIFIC FUNCTIONS
%%-------------------------------------------------------------------------
% Evolving transition proability
% Note: [PASS INTERNAL REVIEW: April 5, 2023]
function prob = Peli(li2, li1)
    global pli
    if (li1 == 0) && (li2 == 0)
        prob = 1;
    elseif (li1 > 0) && (li2 == li1 - 1)
        prob = pli;
    elseif (li1 > 0) && (li2 == li1)
        prob = 1 - pli;
    else
        prob = 0;
    end
end

%%-------------------------------------------------------------------------
% Note: hard coded for the current number of service point.
% Change if new map is considered.
%%-------------------------------------------------------------------------

% Custom index to subscript function
% Note: [PASS INTERNAL REVIEW: April 5, 2023]
function sub = state_ind2sub(x, s)
    % Inputs:
    % s is the shape of the state
    % x is the index of the state
    
    [d, b, v, l1, l2, l3, l4] = ind2sub(s, x);
    d = d-1; % d \in {0, ... nd}
    b = b-2; % b \in {-1, 0, ... nb}
    % v index stays the same
    l1 = l1 - 1; % l1 \in {0, ... nl1}
    l2 = l2 - 1; % l1 \in {0, ... nl2}
    l3 = l3 - 1; % l1 \in {0, ... nl2}
    l4 = l4 - 1; % l1 \in {0, ... nl2}
    sub = [d, b, v, l1, l2, l3, l4];
end

%%-------------------------------------------------------------------------