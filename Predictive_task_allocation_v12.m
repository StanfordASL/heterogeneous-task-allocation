% Predictive Task Allocation (Based on paper)

close all, clear all, clc

%% Initialize by User

% Grid Size
grid_num_x = 8;
grid_num_y = 10;

% Number of Heterogeneous Agent Types
num_agent_type = 2;

% Number of Agents per Type
num_agents_per_type = zeros(num_agent_type,1);
for i=1:1:num_agent_type
    num_agents_per_type(i) = 3;
end
num_agents_per_type(end) = 6;

% Number of Intruders per Type
num_intruders_per_type = zeros(num_agent_type+1,1);
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    num_intruders_per_type(i) = 3;
end
num_intruders_per_type(end) = 4;

% Probability of Intruder Appearing
prob_intruder_type_appearing = zeros(num_agent_type+1,1);
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    prob_intruder_type_appearing(i,1) = 0.2;
end

% Probability of Intruder Appearing
reward_intruder_type = zeros(num_agent_type+1,1);
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    reward_intruder_type(i,1) = 10;
end
reward_intruder_type(end) = 5;

% Intruder Step Size
intruder_step_size = 1;

% Simulation Run Time Steps
t_final = 10;

% Flag Plot in Continuous Video
flag_plot_continuous_video = 1; % set to 1 if you want a continuous video

% Flag to Store Video
flag_store_video = 0; % set to 1 if you want to store video

% Flag to set type of random numbers
flag_use_stored_random_numbers = 0; % If set to 1 if you want to use stored random numbers, else use any other number for general rand output

% Maximum L1 Distance Agent can Move
threshold_max_distance = 2;

% Flag for task allocation type
flag_type_task_allocation = 5; % 1 = Dumb Hurestic 1, 2 = Dumb Hurestic 2, 3 = Centralized Table II, 4 = Distributed PFSF, Centralized Homogeneous, 5 = Distributed PFSF, Distributed Homogeneous,

% Time Horizon Window
time_horizon = 3; % integer >= 0

%% Initialize Variables

standard_font_size = 25;
title_font_size = 40;

color_array = ['r' 'b' 'g' 'c' 'm' 'y'];
color_array(num_agent_type+1) = 'w';

if (flag_store_video == 1)
    myVideo1 = VideoWriter('video_v12.mp4', 'MPEG-4');
    myVideo1.FrameRate = 5;  % Default 30
    myVideo1.Quality = 100;    % Default 75
    open(myVideo1);
end

% Indicator variable representing intruder's precense
intruder_present = [];
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    intruder_present{i} = zeros(num_intruders_per_type(i),1);
end

% Variable representing intruder's position
intruder_position = [];
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    intruder_position{i} = zeros(num_intruders_per_type(i),2);
end

% Variable representing intruder's bin
intruder_bin = [];
for i=1:1:num_agent_type+1  % extra 1 for shared tasks
    intruder_bin{i} = zeros(num_intruders_per_type(i),2);
end

% Variable representing agent's position
agent_position = [];
for i=1:1:num_agent_type
    for j=1:1:num_agents_per_type(i)
        agent_position{i}(j,1:2) = [ceil(grid_num_x*func_use_stored_rand(flag_use_stored_random_numbers))-0.5   ceil(grid_num_y*func_use_stored_rand(flag_use_stored_random_numbers))-0.5];
    end
end

% Variable representing agent's bin
agent_bin = [];
for i=1:1:num_agent_type
    agent_bin{i} = zeros(num_agents_per_type(i),2);
end

total_reward_available_array = zeros(t_final,1);
total_reward_earned_array = zeros(t_final,1);
cumulative_total_reward_available_array = zeros(t_final,1);
cumulative_total_reward_earned_array = zeros(t_final,1);
fraction_total_reward_earned_array = zeros(t_final,1);
fraction_cumulative_total_reward_earned_array = zeros(t_final,1);


%% Run Time Step Loop


for t=1:1:t_final
    disp(t)
    
    %% Update Intruder Position
    old_intruder_position = intruder_position;
    old_intruder_present = intruder_present;
    old_intruder_bin = intruder_bin;
    
    for i=1:1:num_agent_type+1
        for j=1:1:num_intruders_per_type(i)
            
            % Update Intruder's Position
            if intruder_present{i}(j) == 1
                intruder_position{i}(j,:) = intruder_position{i}(j,:) + [2*intruder_step_size*func_use_stored_rand(flag_use_stored_random_numbers)-intruder_step_size   2*intruder_step_size*func_use_stored_rand(flag_use_stored_random_numbers)-intruder_step_size];
                
                if intruder_position{i}(j,1) < 0 || intruder_position{i}(j,1) > grid_num_x || intruder_position{i}(j,2) < 0 || intruder_position{i}(j,2) > grid_num_y
                    intruder_present{i}(j) = 0;
                    
                    if (intruder_position{i}(j,1) < 0 )
                        intruder_position{i}(j,1) = -1;
                    elseif (intruder_position{i}(j,1) > grid_num_x)
                        intruder_position{i}(j,1) = grid_num_x + 1;
                    end
                    
                    if (intruder_position{i}(j,2) < 0)
                        intruder_position{i}(j,2) = -1;
                    elseif (intruder_position{i}(j,2) > grid_num_y)
                        intruder_position{i}(j,2) = grid_num_y + 1;
                    end
                end
                
            end
            
            % If we want to add intruder "probabilistically"
            if old_intruder_present{i}(j) == 0
                if func_use_stored_rand(flag_use_stored_random_numbers) <= prob_intruder_type_appearing(i,1)
                    intruder_present{i}(j) = 1;
                    
                    % Update its position
                    this_bin_boundary = ceil(func_use_stored_rand(flag_use_stored_random_numbers)*(2*grid_num_x+2*grid_num_y-4));
                    
                    if this_bin_boundary <= grid_num_x % lower x boundary
                        intruder_position{i}(j,:) = [func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_x    func_use_stored_rand(flag_use_stored_random_numbers)];
                        old_intruder_position{i}(j,:) = [intruder_position{i}(j,1)    -1];
                        
                    elseif this_bin_boundary <= grid_num_x + grid_num_y - 1 % right y boundary
                        intruder_position{i}(j,:) = [grid_num_x-func_use_stored_rand(flag_use_stored_random_numbers) func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_y];
                        old_intruder_position{i}(j,:) = [grid_num_x+1 intruder_position{i}(j,2)];
                        
                    elseif this_bin_boundary <= 2*grid_num_x + grid_num_y - 2 % upper x boundary
                        intruder_position{i}(j,:) = [func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_x   grid_num_y-func_use_stored_rand(flag_use_stored_random_numbers)];
                        old_intruder_position{i}(j,:) = [intruder_position{i}(j,1)   grid_num_y+1];
                        
                    else % left y boundary
                        intruder_position{i}(j,:) = [func_use_stored_rand(flag_use_stored_random_numbers) func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_y];
                        old_intruder_position{i}(j,:) = [-1 intruder_position{i}(j,2)];
                        
                    end
                end
            end
            
            intruder_bin{i}(j,:) = ceil(intruder_position{i}(j,:));
            
        end
    end
    
    %% Update Intruder Probability
    
    if t==1
        % COMMON TO ALL TIME STEPS
        
        total_grid_bins = grid_num_x * grid_num_y;
        
        total_grid_boundary_bins = 2*grid_num_x + 2*grid_num_y - 4;
        
        % transition probability of random walk
        num_trial_point_trans_prob = 100000;
        pos_trial_point_trans_prob = zeros(num_trial_point_trans_prob,2);
        for n_trial = 1:1:num_trial_point_trans_prob
            pos_trial_point_trans_prob(n_trial,:) = [rand rand] + [2*intruder_step_size*rand-intruder_step_size   2*intruder_step_size*rand-intruder_step_size];
        end
        
        center_bin = [1 1];
        trans_prob_table = zeros(1,3);
        count = 0;
        for this_bin_x = (center_bin(1) - ceil(2*intruder_step_size)):1:(center_bin(1) + ceil(2*intruder_step_size))
            for this_bin_y = (center_bin(2) - ceil(2*intruder_step_size)):1:(center_bin(2) + ceil(2*intruder_step_size))
                count = count + 1;
                num_in_bin = sum((pos_trial_point_trans_prob(:,1) >= (this_bin_x-1)) & (pos_trial_point_trans_prob(:,1) <= this_bin_x) & (pos_trial_point_trans_prob(:,2) >= (this_bin_y-1)) & (pos_trial_point_trans_prob(:,2) <= this_bin_y));
                trans_prob_table(count,:) = [this_bin_x-center_bin(1)   this_bin_y-center_bin(2)   num_in_bin/num_trial_point_trans_prob];
            end
        end
        
        trans_prob_matrix = zeros(total_grid_bins,total_grid_bins);
        for i=1:1:grid_num_x
            for j=1:1:grid_num_y
                culmulative_bin_from = func_culmulative_bin(i,j,grid_num_x,grid_num_y);
                
                center_bin = [i j];
                count = 0;
                for this_bin_x = (center_bin(1) - ceil(2*intruder_step_size)):1:(center_bin(1) + ceil(2*intruder_step_size))
                    for this_bin_y = (center_bin(2) - ceil(2*intruder_step_size)):1:(center_bin(2) + ceil(2*intruder_step_size))
                        count = count + 1;
                        
                        culmulative_bin_to = func_culmulative_bin(this_bin_x,this_bin_y,grid_num_x,grid_num_y);
                        
                        if culmulative_bin_to ~= 0
                            trans_prob_matrix(culmulative_bin_to,culmulative_bin_from) = trans_prob_table(count,3);
                        end
                    end
                end
                
            end
        end
        
        grid_boundary_bins = zeros(total_grid_boundary_bins,1);
        count = 0;
        for i=1:1:grid_num_x
            count = count + 1;
            grid_boundary_bins(count,:) = func_culmulative_bin(i,1,grid_num_x,grid_num_y);
            count = count + 1;
            grid_boundary_bins(count,:) = func_culmulative_bin(i,grid_num_y,grid_num_x,grid_num_y);
        end
        for i=2:1:grid_num_y-1
            count = count + 1;
            grid_boundary_bins(count,:) = func_culmulative_bin(1,i,grid_num_x,grid_num_y);
            count = count + 1;
            grid_boundary_bins(count,:) = func_culmulative_bin(grid_num_x,i,grid_num_x,grid_num_y);
        end
        
    end
    
    % SPECIFIC TO THIS TIME STEP
    
    T_f_tau = zeros(total_grid_bins,time_horizon+2,num_agent_type+1);
    
    for tau = 1:1:time_horizon+2
        for i=1:1:num_agent_type+1
            
            if tau == 1
                for k=1:1:num_intruders_per_type(i)
                    if old_intruder_present{i}(k) == 1
                        this_intruder_bin = old_intruder_bin{i}(k,:);
                        culmulative_bin = func_culmulative_bin(this_intruder_bin(1),this_intruder_bin(2),grid_num_x,grid_num_y);
                        T_f_tau(culmulative_bin,tau,i) = T_f_tau(culmulative_bin,tau,i) + reward_intruder_type(i);
                    end
                end
                
            elseif tau == 2
                for k=1:1:num_intruders_per_type(i)
                    if intruder_present{i}(k) == 1
                        this_intruder_bin = intruder_bin{i}(k,:);
                        culmulative_bin = func_culmulative_bin(this_intruder_bin(1),this_intruder_bin(2),grid_num_x,grid_num_y);
                        T_f_tau(culmulative_bin,tau,i) = T_f_tau(culmulative_bin,tau,i) + reward_intruder_type(i);
                    end
                end
                
            else
                T_f_tau(:,tau,i) = trans_prob_matrix * T_f_tau(:,tau-1,i);
                
                for k=1:1:num_intruders_per_type(i)
                    if intruder_present{i}(k) == 0
                        this_intruder_appearing_prob = ((1-prob_intruder_type_appearing(i))^(tau-3)) * prob_intruder_type_appearing(i) * reward_intruder_type(i);
                        T_f_tau(grid_boundary_bins,tau,i) = T_f_tau(grid_boundary_bins,tau,i) + (this_intruder_appearing_prob/total_grid_boundary_bins);
                    end
                end
                
            end
            
        end
    end
    
    %% Update Agent Position
    old_agent_position = agent_position;
    new_agent_position = agent_position;
    
    for i=1:1:num_agent_type
        for j=1:1:num_agents_per_type(i)
            agent_bin{i}(j,:) = ceil(agent_position{i}(j,:));
        end
    end
    
    
    switch flag_type_task_allocation
        
        case 1
            %% Dumb Hurestic 1
            
            % loop over all agent fleets
            for i=1:1:num_agent_type
                
                cost_mat = [];
                final_pos = [];
                
                % private intruder
                for k=1:1:num_intruders_per_type(i)
                    if intruder_present{i}(k) == 1
                        this_intruder_bin = intruder_bin{i}(k,:);
                        final_pos = [final_pos; this_intruder_bin];
                        this_cost_mat = (100 + vecnorm(agent_bin{i} - this_intruder_bin,1,2)).*(vecnorm(agent_bin{i} - this_intruder_bin,1,2) > threshold_max_distance) - reward_intruder_type(i);
                        cost_mat = [cost_mat, this_cost_mat];
                    end
                end
                
                % shared intruder
                for k=1:1:num_intruders_per_type(num_agent_type+1)
                    if intruder_present{num_agent_type+1}(k) == 1
                        this_intruder_bin = intruder_bin{num_agent_type+1}(k,:);
                        final_pos = [final_pos; this_intruder_bin];
                        this_cost_mat = (100 + vecnorm(agent_bin{i} - this_intruder_bin,1,2)).*(vecnorm(agent_bin{i} - this_intruder_bin,1,2) > threshold_max_distance) - reward_intruder_type(num_agent_type+1);
                        cost_mat = [cost_mat, this_cost_mat];
                    end
                end
                
                if ~isempty(cost_mat)
                    
                    % solve assignment
                    [ASSIGN,COST] = munkres(cost_mat);
                    
                    % update position
                    for j=1:1:num_agents_per_type(i)
                        if ASSIGN(j) ~= 0
                            
                            this_intruder_bin = final_pos(ASSIGN(j), :);
                            
                            if vecnorm(agent_bin{i}(j,:) - this_intruder_bin,1,2) <= threshold_max_distance
                                new_agent_position{i}(j,:) = this_intruder_bin - [0.5 0.5];
                            else
                                % find closest reachable bin
                                this_agent_bin = agent_bin{i}(j,:);
                                deficit = this_intruder_bin - this_agent_bin;
                                limit = threshold_max_distance;
                                
                                while limit >= 1
                                    if abs(deficit(1)) > abs(deficit(2))
                                        this_agent_bin(1) = this_agent_bin(1) + sign(deficit(1));
                                    elseif abs(deficit(1)) < abs(deficit(2))
                                        this_agent_bin(2) = this_agent_bin(2) + sign(deficit(2));
                                    elseif grid_num_x >= grid_num_y
                                        this_agent_bin(1) = this_agent_bin(1) + sign(deficit(1));
                                    else
                                        this_agent_bin(2) = this_agent_bin(2) + sign(deficit(2));
                                    end
                                    limit = limit - 1;
                                end
                                
                                new_agent_position{i}(j,:) = this_agent_bin - [0.5 0.5];
                                
                            end
                            
                        end
                        
                    end
                    
                end
                
            end
            
            
            
        case 2
            %% Dumb Hurestic 2
            
            cost_mat = [];
            final_pos = [];
            
            % loop over all intruders
            
            for i=1:1:num_agent_type+1
                for k=1:1:num_intruders_per_type(i)
                    
                    if intruder_present{i}(k) == 1
                        this_intruder_bin = intruder_bin{i}(k,:);
                        final_pos = [final_pos; this_intruder_bin];
                        
                        this_cost_mat = [];
                        
                        for j=1:1:num_agent_type
                            if (j == i) || (i == num_agent_type+1)
                                this_cost_mat_l = (100 + vecnorm(agent_bin{j} - this_intruder_bin,1,2)).*(vecnorm(agent_bin{j} - this_intruder_bin,1,2) > threshold_max_distance) - reward_intruder_type(i);
                            else
                                this_cost_mat_l = 1000*(vecnorm(agent_bin{j} - this_intruder_bin,1,2) > -1);
                            end
                            
                            this_cost_mat = [this_cost_mat; this_cost_mat_l];
                        end
                        
                        cost_mat = [cost_mat, this_cost_mat];
                    end
                end
            end
            
            if ~isempty(cost_mat)
                
                % solve assignment
                [ASSIGN,COST] = munkres(cost_mat);
                
                % update position
                count = 0;
                for i=1:1:num_agent_type
                    for j=1:1:num_agents_per_type(i)
                        count = count + 1;
                        
                        if ASSIGN(count) ~= 0
                            
                            this_intruder_bin = final_pos(ASSIGN(count), :);
                            
                            if vecnorm(agent_bin{i}(j,:) - this_intruder_bin,1,2) <= threshold_max_distance
                                new_agent_position{i}(j,:) = this_intruder_bin - [0.5 0.5];
                            else
                                % find closest reachable bin
                                this_agent_bin = agent_bin{i}(j,:);
                                deficit = this_intruder_bin - this_agent_bin;
                                limit = threshold_max_distance;
                                
                                while limit >= 1
                                    if abs(deficit(1)) > abs(deficit(2))
                                        this_agent_bin(1) = this_agent_bin(1) + sign(deficit(1));
                                    elseif abs(deficit(1)) < abs(deficit(2))
                                        this_agent_bin(2) = this_agent_bin(2) + sign(deficit(2));
                                    elseif grid_num_x >= grid_num_y
                                        this_agent_bin(1) = this_agent_bin(1) + sign(deficit(1));
                                    else
                                        this_agent_bin(2) = this_agent_bin(2) + sign(deficit(2));
                                    end
                                    limit = limit - 1;
                                end
                                
                                new_agent_position{i}(j,:) = this_agent_bin - [0.5 0.5];
                                
                            end
                            
                        end
                        
                    end
                end
                
            end
            
        case 3
            %% centralized LP-relaxed assignment (Table II)
            
            if t==1
                
                % COMMON TO ALL TIME STEPS
                
                % variables
                % x^q_tau  total_grid_bins X total_grid_bins X total_num_agents X time_horizon
                % y^q_tau  total_grid_bins X total_num_agents X (time_horizon+1)
                % z^q_tau  total_grid_bins X total_num_agents X (time_horizon+1)
                
                total_num_agents = sum(num_agents_per_type);
                
                total_x_q_tau = total_grid_bins * total_grid_bins * total_num_agents * (time_horizon+1);
                
                total_y_q_tau = total_grid_bins * total_num_agents * (time_horizon+2);
                
                total_z_q_tau = total_grid_bins * total_num_agents * (time_horizon+2);
                
                lower_bound = zeros(total_x_q_tau + total_y_q_tau + total_z_q_tau,1);
                upper_bound = ones(total_x_q_tau + total_y_q_tau + total_z_q_tau,1);
                
                A_in_equality_entries = zeros(10,3);
                b_in_equality = zeros(10,1);
                count = 0;
                count_Aineq = 0;
                
                % Eq (1g)
                for tau=1:1:time_horizon+1
                    for i=1:1:num_agent_type
                        for j=1:1:num_agents_per_type(i)
                            for g=1:1:total_grid_bins
                                count = count + 1;
                                b_in_equality(count,1) = 0;
                                
                                count_Aineq = count_Aineq + 1;
                                culmulative_variable_number = func_culmulative_variable_number('y',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                                
                                for g2=1:1:total_grid_bins
                                    if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                        count_Aineq = count_Aineq + 1;
                                        culmulative_variable_number = func_culmulative_variable_number('x',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g2);
                                        A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
                                    end
                                end
                                
                            end
                        end
                    end
                end
                
                % Eq (1h)
                tau = time_horizon+1;
                for i=1:1:num_agent_type
                    for j=1:1:num_agents_per_type(i)
                        for g=1:1:total_grid_bins
                            count = count + 1;
                            b_in_equality(count,1) = 0;
                            
                            count_Aineq = count_Aineq + 1;
                            culmulative_variable_number = func_culmulative_variable_number('y',j,i,g,tau+1,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                            A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                            
                            for g2=1:1:total_grid_bins
                                if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                    count_Aineq = count_Aineq + 1;
                                    culmulative_variable_number = func_culmulative_variable_number('x',j,i,g2,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g);
                                    A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
                                end
                            end
                            
                        end
                    end
                end
                
                % Eq (1i)
                for tau=1:1:time_horizon+2
                    for g=1:1:total_grid_bins
                        count = count + 1;
                        b_in_equality(count,1) = 1;
                        
                        for i=1:1:num_agent_type
                            for j=1:1:num_agents_per_type(i)
                                count_Aineq = count_Aineq + 1;
                                culmulative_variable_number = func_culmulative_variable_number('y',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                            end
                        end
                        
                    end
                end
                
                % Eq (1k)
                for tau=1:1:time_horizon+1
                    for i=1:1:num_agent_type
                        for j=1:1:num_agents_per_type(i)
                            for g=1:1:total_grid_bins
                                count = count + 1;
                                b_in_equality(count,1) = 0;
                                
                                count_Aineq = count_Aineq + 1;
                                culmulative_variable_number = func_culmulative_variable_number('z',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                                
                                for g2=1:1:total_grid_bins
                                    if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                        count_Aineq = count_Aineq + 1;
                                        culmulative_variable_number = func_culmulative_variable_number('x',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g2);
                                        A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
                                    end
                                end
                                
                            end
                        end
                    end
                end
                
                % Eq (1l)
                tau = time_horizon+1;
                for i=1:1:num_agent_type
                    for j=1:1:num_agents_per_type(i)
                        for g=1:1:total_grid_bins
                            count = count + 1;
                            b_in_equality(count,1) = 0;
                            
                            count_Aineq = count_Aineq + 1;
                            culmulative_variable_number = func_culmulative_variable_number('z',j,i,g,tau+1,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                            A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                            
                            for g2=1:1:total_grid_bins
                                if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                    count_Aineq = count_Aineq + 1;
                                    culmulative_variable_number = func_culmulative_variable_number('x',j,i,g2,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g);
                                    A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
                                end
                            end
                            
                        end
                    end
                end
                
                % Eq (1m)
                for tau=1:1:time_horizon+2
                    for i=1:1:num_agent_type
                        for g=1:1:total_grid_bins
                            count = count + 1;
                            b_in_equality(count,1) = 1;
                            
                            for j=1:1:num_agents_per_type(i)
                                count_Aineq = count_Aineq + 1;
                                culmulative_variable_number = func_culmulative_variable_number('z',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
                            end
                            
                        end
                    end
                end
                
                A_in_equality = sparse(A_in_equality_entries(:,1),A_in_equality_entries(:,2),A_in_equality_entries(:,3),count,total_x_q_tau+total_y_q_tau+total_z_q_tau);
                
                A_equality_entries = zeros(10,3);
                count = 0;
                count_Aeq = 0;
                
                % Eq (1d)
                tau = 1;
                for i=1:1:num_agent_type
                    for j=1:1:num_agents_per_type(i)
                        
                        this_agent_bin = agent_bin{i}(j,:);
                        culmulative_bin = func_culmulative_bin(this_agent_bin(1),this_agent_bin(2),grid_num_x,grid_num_y);
                        
                        for g=1:1:total_grid_bins
                            count = count + 1;
                            
                            for g2=1:1:total_grid_bins
                                if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                    count_Aeq = count_Aeq + 1;
                                    culmulative_variable_number = func_culmulative_variable_number('x',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g2);
                                    A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, 1];
                                end
                            end
                            
                        end
                    end
                end
                
                % Eq (1e)
                for tau=1:1:time_horizon
                    for i=1:1:num_agent_type
                        for j=1:1:num_agents_per_type(i)
                            for g=1:1:total_grid_bins
                                count = count + 1;
                                
                                for g2=1:1:total_grid_bins
                                    if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                        count_Aeq = count_Aeq + 1;
                                        culmulative_variable_number = func_culmulative_variable_number('x',j,i,g2,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g);
                                        A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, 1];
                                    end
                                end
                                
                                for g2=1:1:total_grid_bins
                                    if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                        count_Aeq = count_Aeq + 1;
                                        culmulative_variable_number = func_culmulative_variable_number('x',j,i,g,tau+1,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g2);
                                        A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, -1];
                                    end
                                end
                                
                            end
                        end
                    end
                end
                
                A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count,total_x_q_tau+total_y_q_tau+total_z_q_tau);
                
            end
            
            % SPECIFIC TO THIS TIME STEP
            
            f_cost = zeros(total_x_q_tau+total_y_q_tau+total_z_q_tau,1);
            
            for tau = 1:1:time_horizon+2
                for g=1:1:total_grid_bins
                    for i=1:1:num_agent_type
                        for j=1:1:num_agents_per_type(i)
                            
                            culmulative_variable_number = func_culmulative_variable_number('y',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                            f_cost(culmulative_variable_number,1) = f_cost(culmulative_variable_number,1) + T_f_tau(g,tau,num_agent_type+1);
                            
                            culmulative_variable_number = func_culmulative_variable_number('z',j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0);
                            f_cost(culmulative_variable_number,1) = f_cost(culmulative_variable_number,1) + T_f_tau(g,tau,i);
                            
                        end
                    end
                end
            end
            
            b_equality = zeros(10,1);
            count = 0;
            
            % Eq (1d)
            tau = 1;
            for i=1:1:num_agent_type
                for j=1:1:num_agents_per_type(i)
                    
                    this_agent_bin = agent_bin{i}(j,:);
                    culmulative_bin = func_culmulative_bin(this_agent_bin(1),this_agent_bin(2),grid_num_x,grid_num_y);
                    
                    for g=1:1:total_grid_bins
                        count = count + 1;
                        
                        
                        if g == culmulative_bin
                            b_equality(count,1) = 1;
                        else
                            b_equality(count,1) = 0;
                        end
                    end
                end
            end
            
            % Eq (1e)
            for tau=1:1:time_horizon
                for i=1:1:num_agent_type
                    for j=1:1:num_agents_per_type(i)
                        for g=1:1:total_grid_bins
                            count = count + 1;
                            b_equality(count,1) = 0;
                            
                        end
                    end
                end
            end
            
            
            [x_sol , f_val] = linprog(-f_cost,A_in_equality,b_in_equality,A_equality,b_equality,lower_bound,upper_bound);
            
            % Select assignment
            tau = 1;
            for i=1:1:num_agent_type
                for j=1:1:num_agents_per_type(i)
                    
                    best_bin_prob = 0;
                    best_bin = 0;
                    
                    for g=1:1:total_grid_bins
                        this_bin_prob = 0;
                        
                        for g2=1:1:total_grid_bins
                            if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                culmulative_variable_number = func_culmulative_variable_number('x',j,i,g2,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g);
                                this_bin_prob = this_bin_prob + x_sol(culmulative_variable_number);
                            end
                        end
                        
                        if best_bin_prob < this_bin_prob
                            best_bin_prob = this_bin_prob;
                            best_bin = g;
                        end
                    end
                    
                    [best_bin_x, best_bin_y] = func_bin_from_culmulative_bin(best_bin,grid_num_x,grid_num_y,total_grid_bins);
                    new_agent_position{i}(j,:) = [best_bin_x best_bin_y] - [0.5 0.5];
                end
            end
            
        case {4, 5}
            %% distributed PFSF assignment (Algorithm 1)
            
            % COMMON TO ALL TIME STEPS
            
            % variables
            % x^q_tau  total_grid_bins X total_grid_bins X total_num_agents X time_horizon
            % y^q_tau  total_grid_bins X total_num_agents X (time_horizon+1)
            % z^q_tau  total_grid_bins X total_num_agents X (time_horizon+1)
            
            total_num_agents = sum(num_agents_per_type);
            
            total_x_q_tau = total_grid_bins * total_grid_bins * total_num_agents * (time_horizon+1);
            
            total_y_q_tau = total_grid_bins * total_num_agents * (time_horizon+2);
            
            total_z_q_tau = total_grid_bins * total_num_agents * (time_horizon+2);
            
            % SPECIFIC TO THIS TIME STEP
            
            % Distributed Private First
            
            disp('Distributed Private First')
            
            PF_x_sol = [];
            PF_reward = 0;
            
            for i_fleet = 1:1:num_agent_type
                
                homo_T_f_tau = T_f_tau(:,:,i_fleet) + (1/num_agent_type)*T_f_tau(:,:,num_agent_type+1);
                homo_num_agents = num_agents_per_type(i_fleet);
                homo_agent_bin = agent_bin{i_fleet};
                
                if flag_type_task_allocation == 4
                    [homo_reward, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                    
                elseif flag_type_task_allocation == 5
                    [homo_reward, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                    
                else
                    disp('Shouldnt reach here!')
                    
                end
                
                PF_x_sol = [PF_x_sol; homo_x_sol];
                PF_reward = PF_reward + homo_reward;
                
            end
            
            % Distributed Shared First
            
            disp('Distributed Shared First')
            
            homo_T_f_tau = T_f_tau(:,:,num_agent_type+1);
            homo_num_agents = total_num_agents;
            homo_agent_bin = [];
            for i_fleet = 1:1:num_agent_type
                homo_agent_bin = [homo_agent_bin; agent_bin{i_fleet}];
            end
            
            if flag_type_task_allocation == 4
                [homo_reward, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                
            elseif flag_type_task_allocation == 5
                [homo_reward, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                
            else
                disp('Shouldnt reach here!')
                
            end
            homo_y_sol = homo_z_sol;
            
            SF_x_sol = [];
            SF_reward = 0;
            for i_fleet = 1:1:num_agent_type
                
                homo_T_f_tau = T_f_tau(:,:,i_fleet);
                
                for j=1:1:num_agents_per_type(i_fleet)
                    for tau = 1:1:time_horizon+2
                        for g=1:1:total_grid_bins
                            
                            culmulative_variable_number = func_culmulative_variable_number('y',j,i_fleet,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,0) - total_x_q_tau;
                            if homo_y_sol(culmulative_variable_number,1) >= 0.5
                                homo_T_f_tau(g,tau) = homo_T_f_tau(g,tau) + T_f_tau(g,tau,num_agent_type+1);
                            end
                        end
                    end
                end
                
                homo_num_agents = num_agents_per_type(i_fleet);
                homo_agent_bin = agent_bin{i_fleet};
                
                if flag_type_task_allocation == 4
                    [homo_reward, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                    
                elseif flag_type_task_allocation == 5
                    [homo_reward, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
                    
                else
                    disp('Shouldnt reach here!')
                    
                end
                
                SF_x_sol = [SF_x_sol; homo_x_sol];
                SF_reward = SF_reward + homo_reward;
                
            end
            
            % Select assignment
            if PF_reward >= SF_reward
                x_sol = PF_x_sol;
                disp('Private First selected!')
            else
                x_sol = SF_x_sol;
                disp('Shared First selected!')
            end
            
            tau = 1;
            for i=1:1:num_agent_type
                for j=1:1:num_agents_per_type(i)
                    
                    best_bin_prob = 0;
                    best_bin = 0;
                    
                    for g=1:1:total_grid_bins
                        this_bin_prob = 0;
                        
                        for g2=1:1:total_grid_bins
                            if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                                culmulative_variable_number = func_culmulative_variable_number('x',j,i,g2,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g);
                                this_bin_prob = this_bin_prob + x_sol(culmulative_variable_number);
                            end
                        end
                        
                        if best_bin_prob < this_bin_prob
                            best_bin_prob = this_bin_prob;
                            best_bin = g;
                        end
                    end
                    
                    [best_bin_x, best_bin_y] = func_bin_from_culmulative_bin(best_bin,grid_num_x,grid_num_y,total_grid_bins);
                    new_agent_position{i}(j,:) = [best_bin_x best_bin_y] - [0.5 0.5];
                end
            end
            
            
            
        otherwise
            disp('Shouldnt reach here!')
            
    end
    
    % Final update after assignment
    agent_position = new_agent_position;
    
    %% Update Reward Computation
    
    for i=1:1:num_agent_type
        for j=1:1:num_agents_per_type(i)
            agent_bin{i}(j,:) = ceil(agent_position{i}(j,:));
        end
    end
    
    this_reward_available = 0;
    this_reward_collected = 0;
    
    for i=1:1:num_agent_type
        for k=1:1:num_intruders_per_type(i)
            
            if intruder_present{i}(k) == 1
                this_reward_available = this_reward_available + reward_intruder_type(i);
                this_intruder_bin = intruder_bin{i}(k,:);
                
                flag_collect_reward = 0;
                for j=1:1:num_agents_per_type(i)
                    if (this_intruder_bin(1) == agent_bin{i}(j,1)) && (this_intruder_bin(2) == agent_bin{i}(j,2))
                        flag_collect_reward = 1;
                    end
                end
                
                this_reward_collected = this_reward_collected + flag_collect_reward*reward_intruder_type(i);
            end
        end
    end
    
    for k=1:1:num_intruders_per_type(num_agent_type+1)
        
        if intruder_present{num_agent_type+1}(k) == 1
            this_reward_available = this_reward_available + reward_intruder_type(num_agent_type+1);
            this_intruder_bin = intruder_bin{num_agent_type+1}(k,:);
            
            flag_collect_reward = 0;
            for i=1:1:num_agent_type
                for j=1:1:num_agents_per_type(i)
                    if (this_intruder_bin(1) == agent_bin{i}(j,1)) && (this_intruder_bin(2) == agent_bin{i}(j,2))
                        flag_collect_reward = 1;
                    end
                end
            end
            this_reward_collected = this_reward_collected + flag_collect_reward*reward_intruder_type(num_agent_type+1);
        end
    end
    
    total_reward_available_array(t,1) = this_reward_available;
    total_reward_earned_array(t,1) = this_reward_collected;
    fraction_total_reward_earned_array(t,1) = this_reward_collected/this_reward_available;
    cumulative_total_reward_available_array(t,1) = sum(total_reward_available_array(1:t,1));
    cumulative_total_reward_earned_array(t,1) = sum(total_reward_earned_array(1:t,1));
    fraction_cumulative_total_reward_earned_array(t,1) = cumulative_total_reward_earned_array(t,1)/cumulative_total_reward_available_array(t,1);
    
    %% Plotting
    if flag_plot_continuous_video == 1
        num_video_steps = 10;
    else
        num_video_steps = 1;
    end
    
    for nv = 0:1/num_video_steps:1
        
        h4=figure(1);
        clf
        set(h4,'Color',[1 1 1]);
        set(h4,'units','normalized','outerposition',[0 0 1 1])
        set(h4,'PaperPositionMode','auto');
        
        subplot(2,4,[1 2 5 6])
        hold on
        
        % plot grid
        for gi = 0:1:grid_num_x
            plot([gi gi],[0 grid_num_y],'-k','LineWidth',1.0)
        end
        for gj = 0:1:grid_num_y
            plot([0 grid_num_x],[gj gj],'-k','LineWidth',1.0)
        end
        
        % plot intruders
        for i=1:1:num_agent_type+1
            for j=1:1:num_intruders_per_type(i)
                if (intruder_present{i}(j) == 1) || (old_intruder_present{i}(j) == 1)
                    plot(nv*(intruder_position{i}(j,1)-old_intruder_position{i}(j,1))+old_intruder_position{i}(j,1), nv*(intruder_position{i}(j,2)-old_intruder_position{i}(j,2))+old_intruder_position{i}(j,2), 's','MarkerSize',20,'MarkerFaceColor',color_array(i),'MarkerEdgeColor','k','LineWidth',2.0)
                    %                     if old_intruder_present{i}(j) == 1 || old_intruder_present{i}(j) == 0
                    %                         plot(nv*(intruder_position{i}(j,1)-old_intruder_position{i}(j,1))+old_intruder_position{i}(j,1), nv*(intruder_position{i}(j,2)-old_intruder_position{i}(j,2))+old_intruder_position{i}(j,2), 's','MarkerSize',20,'MarkerFaceColor',color_array(i),'MarkerEdgeColor','k','LineWidth',2.0)
                    %                     else
                    %                         plot(intruder_position{i}(j,1),intruder_position{i}(j,2),'s','MarkerSize',20,'MarkerFaceColor',color_array(i),'MarkerEdgeColor','k','LineWidth',2.0)
                    %                     end
                end
            end
        end
        
        % plot agents
        for i=1:1:num_agent_type
            for j=1:1:num_agents_per_type(i)
                plot(nv*(agent_position{i}(j,1)-old_agent_position{i}(j,1))+old_agent_position{i}(j,1), nv*(agent_position{i}(j,2)-old_agent_position{i}(j,2))+old_agent_position{i}(j,2), 'o','MarkerSize',20-3*i,'MarkerFaceColor',color_array(i),'MarkerEdgeColor',color_array(i),'LineWidth',2.0)
            end
        end
        
        
        xlabel('X axis','fontsize',standard_font_size,'FontName','Times New Roman')
        ylabel('Y axis','fontsize',standard_font_size,'FontName','Times New Roman')
        title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        axis equal
        xlim([-1 grid_num_x+1])
        ylim([-1 grid_num_y+1])
        hold off
        
        
        subplot(2,4,3)
        hold on
        
        plot([1:1:t],total_reward_available_array(1:t,1),'-k','LineWidth',2)
        plot([1:1:t],total_reward_earned_array(1:t,1),'-r','LineWidth',2)
        legend('Available','Earned')
        xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
        ylabel('Reward','fontsize',standard_font_size,'FontName','Times New Roman')
        %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
        xlim([1,max(t,2)])
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        hold off
        
        subplot(2,4,7)
        hold on
        
        plot([1:1:t],fraction_total_reward_earned_array(1:t,1),'-r','LineWidth',2)
        %     legend('Available','Earned')
        xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
        ylabel('Fraction Reward','fontsize',standard_font_size,'FontName','Times New Roman')
        %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
        xlim([1,max(t,2)])
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        hold off
        
        subplot(2,4,4)
        hold on
        
        plot([1:1:t],cumulative_total_reward_available_array(1:t,1),'-k','LineWidth',2)
        plot([1:1:t],cumulative_total_reward_earned_array(1:t,1),'-r','LineWidth',2)
        legend('Available','Earned')
        xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
        ylabel('Cumulative Reward','fontsize',standard_font_size,'FontName','Times New Roman')
        %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
        xlim([1,max(t,2)])
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        hold off
        
        subplot(2,4,8)
        hold on
        
        plot([1:1:t],fraction_cumulative_total_reward_earned_array(1:t,1),'-r','LineWidth',2)
        %     legend('Available','Earned')
        xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
        ylabel('Fraction Cumulative Reward','fontsize',standard_font_size,'FontName','Times New Roman')
        %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
        xlim([1,max(t,2)])
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        hold off
        
        
        drawnow limitrate
        
        if (flag_store_video == 1)
            F = getframe(h4);
            writeVideo(myVideo1, F);
        end
        
    end
end

if (flag_store_video == 1)
    close(myVideo1);
end