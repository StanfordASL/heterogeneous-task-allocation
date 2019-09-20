% Predictive Task Allocation
close all, clear all, clc

%% Initialize by User

grid_num_x = 8;
grid_num_y = 10;

num_agents = 3;
flag_starting_condition = 2; % Set 1 for starting in a row, 2 for random

max_num_intruder = 3;
probability_create_intruder = 0.1;

flag_patrol_intruder = 1; % Set 0 for only patrol, set 1 for tracking intruder

flag_max_distance_constraint = 1; % Set 0 for no constraint, set 1 to use threshold_max_distance as constraint

threshold_max_distance = 2;

flag_store_video = 1;

flag_type_task_allocation = 9; % 1: Dumb Heuristic , 2,6: Task Assignment (1 step) Section 4, 3,7: Task Assignment (multi-step homogenenous) Section 5.A, 4,8: Task Assignment (multi-step heterogeneous) Section 6.A, 9: Generate and visualize stored rand solution

reward_checking_intruder = 500;

reward_patrol_increment = 0;

time_receding_horizon = 5; % integer >= 1

flag_predictive_task_fixed_floating = 2; % 1 for fixed, 2 for floating

flag_show_probability_cloud = 1; % Put to 1 if you want to see the probability cloud of next steps in cases 7 and 8, else put to 0
threshold_probability = 0.05; % show cloud probability above this value

flag_use_stored_random_numbers = 1; % If set to 1, then random numbers are generated using func_use_stored_rand
flag_generate_visualize_store_rand_solution = 2; % Put 1 to generate solution in new_pos_agent_array.mat, put 2 to visulaize this solution in new_pos_agent_array.mat

t_final = 30; % Simulation Run Time Steps

%% Initialize Variables

standard_font_size = 25;
title_font_size = 40;

patrol_grid_reward = zeros(grid_num_x*grid_num_y,1);

patrol_grid_location = zeros(grid_num_x*grid_num_y,2);
for g_x=1:1:grid_num_x
    for g_y=1:1:grid_num_y
        patrol_grid_location((g_x-1)*grid_num_y+g_y,:) = [g_y-0.5, g_x-0.5];
    end
end

weight_distance = 0.1;

if flag_starting_condition == 1
    % Set 1 for starting in a row
    pos_agent = [1:1:num_agents]';
    
elseif flag_starting_condition == 2
    % Set 2 for starting randomly
    pos_agent = zeros(num_agents,1);
    for n=1:1:num_agents
        flag_good_start_location_chosen = 0;
        while flag_good_start_location_chosen == 0
            this_location = ceil(func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_x*grid_num_y);
            flag_good_start_location_chosen = 1;
            for n_prev = 1:1:n-1
                if pos_agent(n_prev,1) == this_location
                    flag_good_start_location_chosen = 0;
                end
            end
        end
        pos_agent(n,1) = this_location;
    end
end

if (flag_store_video == 1)
    myVideo1 = VideoWriter('my_video.mp4', 'MPEG-4');
    myVideo1.FrameRate = 5;  % Default 30
    myVideo1.Quality = 100;    % Default 75
    open(myVideo1);
end

flag_intruder_active = zeros(max_num_intruder,1);
flag_intruder_type = zeros(max_num_intruder,1);
pos_intruder = zeros(max_num_intruder,2);
vel_intruder = zeros(max_num_intruder,2);
bin_intruder = zeros(max_num_intruder,1);

if flag_max_distance_constraint == 0
    threshold_max_distance = grid_num_x*grid_num_y;
end

agent_check_intruder_type = zeros(max_num_intruder,1);

total_grid_cost = zeros(grid_num_x*grid_num_y,grid_num_x*grid_num_y);

for grid_i=1:1:grid_num_x*grid_num_y
    for grid_j=1:1:grid_num_x*grid_num_y
        
        pos_grid_i = [mod(grid_i-1,grid_num_y)+0.5  floor((grid_i-1)/grid_num_y)+0.5];
        pos_grid_j = [mod(grid_j-1,grid_num_y)+0.5  floor((grid_j-1)/grid_num_y)+0.5];
        
        dist = norm(pos_grid_i - pos_grid_j);
        
        if dist > threshold_max_distance
            total_grid_cost(grid_i,grid_j) = 1000000;
        else
            total_grid_cost(grid_i,grid_j) = dist;
        end
        
    end
end

total_reward_available_array = zeros(t_final,1);
total_reward_earned_array = zeros(t_final,1);
cumulative_total_reward_available_array = zeros(t_final,1);
cumulative_total_reward_earned_array = zeros(t_final,1);
fraction_total_reward_earned_array = zeros(t_final,1);
fraction_cumulative_total_reward_earned_array = zeros(t_final,1);

color_array = ['y' 'g' 'b' 'c' 'm'];

%% Initialize LP Matrices

switch flag_type_task_allocation
    
    case 1
        % Do nothing
    case 2
        % Do nothing
    case 3
        % Do nothing
    case 4
        % Do nothing
    case 5
        % Do nothing
        
    case 6
        n_variables = grid_num_x*grid_num_y;
        
        lower_bound = zeros(n_variables*n_variables,1);
        upper_bound = ones(n_variables*n_variables,1);
        
        A_inequality_entries = zeros(n_variables,3);
        b_inequality = zeros(n_variables,1);
        count = 0;
        for grid_j=1:1:n_variables
            for grid_i=1:1:n_variables
                this_grid = (grid_i-1)*n_variables + grid_j;
                count = count + 1;
                A_inequality_entries(count,:) = [grid_j, this_grid, 1];
            end
            b_inequality(grid_j,1) = 1;
        end
        A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
        
        
        A_equality_entries = zeros(n_variables,3);
        %         b_equality = zeros(n_variables,1);
        count = 0;
        for grid_i=1:1:n_variables
            for grid_j=1:1:n_variables
                this_grid = (grid_i-1)*n_variables + grid_j;
                count = count + 1;
                A_equality_entries(count,:) = [grid_i, this_grid, 1];
            end
            %             b_equality(grid_i,1) = pos_agent_array(grid_i);
        end
        A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3));
        
    case 7
        
        n_variables = grid_num_x*grid_num_y;
        
        lower_bound = zeros(time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables,1);
        upper_bound = ones(time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables,1);
        
        A_inequality_entries = zeros(n_variables,3);
        b_inequality = zeros(n_variables,1);
        count = 0;
        count_b = 0;
        for k=1:1:time_receding_horizon
            for grid_j=1:1:n_variables
                count_b = count_b + 1;
                for grid_i=1:1:n_variables
                    this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    count = count + 1;
                    A_inequality_entries(count,:) = [count_b, this_grid, 1];
                end
                b_inequality(count_b,1) = 1;
            end
        end
        
        for grid_j=1:1:n_variables
            count_b = count_b + 1;
            for k=1:1:time_receding_horizon
                this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j;
                count = count + 1;
                A_inequality_entries(count,:) = [count_b, this_grid, 1];
            end
            b_inequality(count_b,1) = 1;
        end
        
        for k=1:1:time_receding_horizon
            for grid_j=1:1:n_variables
                count_b = count_b + 1;
                this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j;
                count = count + 1;
                A_inequality_entries(count,:) = [count_b, this_grid, 1];
                for grid_i=1:1:n_variables
                    this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    count = count + 1;
                    A_inequality_entries(count,:) = [count_b, this_grid, -1];
                end
                b_inequality(count_b,1) = 0;
            end
        end
        A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
        
        A_equality_entries = zeros(n_variables,3);
        %         b_equality = zeros(n_variables,1);
        count = 0;
        count_b = 0;
        for grid_i=1:1:n_variables
            count_b = count_b + 1;
            k = 1;
            for grid_j=1:1:n_variables
                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                count = count + 1;
                A_equality_entries(count,:) = [count_b, this_grid, 1];
            end
            %             b_equality(count_b,1) = pos_agent_array(grid_i);
        end
        
        for k=1:1:time_receding_horizon-1
            for grid_j=1:1:n_variables
                count_b = count_b + 1;
                for grid_i=1:1:n_variables
                    this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    count = count + 1;
                    A_equality_entries(count,:) = [count_b, this_grid, 1];
                    
                    this_grid_2 = (k+1-1)*n_variables*n_variables + (grid_j-1)*n_variables + grid_i;
                    count = count + 1;
                    A_equality_entries(count,:) = [count_b, this_grid_2, -1];
                end
                %                 b_equality(count_b,1) = 0;
            end
        end
        A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count_b,time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables);
        
    case 8
        
        n_variables = grid_num_x*grid_num_y;
        
        lower_bound = zeros(num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables,1);
        upper_bound = ones(num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables,1);
        
        
        A_inequality_entries = zeros(n_variables,3);
        b_inequality = zeros(n_variables,1);
        count = 0;
        count_b = 0;
        
        for k=1:1:time_receding_horizon
            for grid_j=1:1:n_variables
                count_b = count_b + 1;
                
                for grid_i=1:1:n_variables
                    for n=1:1:num_agents
                        this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        count = count + 1;
                        A_inequality_entries(count,:) = [count_b, this_grid, 1];
                    end
                end
                b_inequality(count_b,1) = 1;
            end
        end
        
        for grid_j=1:1:n_variables
            count_b = count_b + 1;
            
            for k=1:1:time_receding_horizon
                for n=1:1:num_agents
                    this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j;
                    count = count + 1;
                    A_inequality_entries(count,:) = [count_b, this_grid, 1];
                end
            end
            
            b_inequality(count_b,1) = 1;
        end
        
        
        for k=1:1:time_receding_horizon
            for grid_j=1:1:n_variables
                for n=1:1:num_agents
                    count_b = count_b + 1;
                    
                    this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j;
                    count = count + 1;
                    A_inequality_entries(count,:) = [count_b, this_grid, 1];
                    
                    for grid_i=1:1:n_variables
                        this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        count = count + 1;
                        A_inequality_entries(count,:) = [count_b, this_grid, -1];
                    end
                    
                    b_inequality(count_b,1) = 0;
                end
            end
        end
        
        A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
        
        A_equality_entries = zeros(n_variables,3);
        %         b_equality = zeros(n_variables,1);
        count = 0;
        count_b = 0;
        
        for grid_i=1:1:n_variables
            for n=1:1:num_agents
                count_b = count_b + 1;
                
                k=1;
                for grid_j=1:1:n_variables
                    this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    count = count + 1;
                    A_equality_entries(count,:) = [count_b, this_grid, 1];
                end
                
                %                 b_equality(count_b,1) = pos_agent_array(grid_i,n);
            end
        end
        
        for k=1:1:time_receding_horizon-1
            for grid_j=1:1:n_variables
                for n=1:1:num_agents
                    count_b = count_b + 1;
                    
                    for grid_i=1:1:n_variables
                        this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        count = count + 1;
                        A_equality_entries(count,:) = [count_b, this_grid, 1];
                        
                        this_grid_2 = (n-1)*time_receding_horizon*n_variables*n_variables + (k+1-1)*n_variables*n_variables + (grid_j-1)*n_variables + grid_i;
                        count = count + 1;
                        A_equality_entries(count,:) = [count_b, this_grid_2, -1];
                    end
                    
                    %                     b_equality(count_b,1) = 0;
                end
            end
        end
        A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count_b,num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables);
        
    case 9
        if flag_generate_visualize_store_rand_solution == 1 % Generate solution
            bin_intruder_array = zeros(num_agents,t_final);
            flag_intruder_active_array = zeros(num_agents,t_final);
            flag_intruder_type_array = zeros(num_agents,t_final);
            pos_agent_original = pos_agent;
        elseif flag_generate_visualize_store_rand_solution == 2 % visulaize solution
            load('pos_agent_array.mat')
        else
            
        end
        
    otherwise
        disp('Should not reach here!')
end


%% Run Algorithms

for t=1:1:t_final
    disp(t)
    
    %% Update Reward of All Grid Points
    patrol_grid_reward = patrol_grid_reward + reward_patrol_increment*ones(grid_num_x*grid_num_y,1);
    total_reward_available_array(t,1) = sum(patrol_grid_reward);
    
    % Remove Reward from Observed Grid Points
    for n=1:1:num_agents
        total_reward_earned_array(t,1) = total_reward_earned_array(t,1) + patrol_grid_reward(pos_agent(n),1);
        patrol_grid_reward(pos_agent(n),1) = 0;
    end
    
    
    % Update Intruder Motion
    for ni=1:1:max_num_intruder
        if flag_intruder_active(ni,1) == 0
            % Intruder is inactive
            
            if func_use_stored_rand(flag_use_stored_random_numbers) < probability_create_intruder % threshold
                % Intiate intruder
                
                flag_intruder_active(ni,1) = 1;
                
                flag_intruder_type(ni,1) = 1+round(func_use_stored_rand(flag_use_stored_random_numbers));
                
                chosen_side = ceil(4*func_use_stored_rand(flag_use_stored_random_numbers));
                
                switch chosen_side
                    
                    case 1
                        pos_intruder(ni,:) = [func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_y 0.1];
                        vel_intruder(ni,:) = [func_use_stored_rand(flag_use_stored_random_numbers)-0.5 func_use_stored_rand(flag_use_stored_random_numbers)];
                        
                    case 2
                        pos_intruder(ni,:) = [grid_num_y-0.1 func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_x];
                        vel_intruder(ni,:) = [-func_use_stored_rand(flag_use_stored_random_numbers) func_use_stored_rand(flag_use_stored_random_numbers)-0.5];
                        
                    case 3
                        pos_intruder(ni,:) = [func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_y grid_num_x-0.1];
                        vel_intruder(ni,:) = [func_use_stored_rand(flag_use_stored_random_numbers)-0.5 -func_use_stored_rand(flag_use_stored_random_numbers)];
                        
                    case 4
                        pos_intruder(ni,:) = [0.1 func_use_stored_rand(flag_use_stored_random_numbers)*grid_num_x];
                        vel_intruder(ni,:) = [func_use_stored_rand(flag_use_stored_random_numbers) func_use_stored_rand(flag_use_stored_random_numbers)-0.5];
                        
                    otherwise
                        disp('Should not reach here!')
                end
                
            end
            
        else
            % Intruder is active
            pos_intruder(ni,:) = pos_intruder(ni,:) + vel_intruder(ni,:);
        end
        
        
        if flag_intruder_active(ni,1) == 1
            if (pos_intruder(ni,1) >= 0) && (pos_intruder(ni,2) >= 0) && (pos_intruder(ni,1) <= grid_num_y) && (pos_intruder(ni,2) <= grid_num_x)
                this_x = ceil(pos_intruder(ni,1));
                this_y = ceil(pos_intruder(ni,2));
                this_bin = (this_y-1)*grid_num_y + this_x;
                bin_intruder(ni,1) = this_bin;
            else
                flag_intruder_active(ni,1) = 0;
                bin_intruder(ni,1) = 0;
                flag_intruder_type(ni,1) = 0;
                %                 agent_check_intruder_type(ni,1) = 0;
            end
        end
        
    end
    
    
    % Intruder Points Available
    for ni=1:1:max_num_intruder
        if flag_intruder_active(ni,1) == 1
            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                total_reward_available_array(t,1) = total_reward_available_array(t,1) + reward_checking_intruder;
            end
        end
    end
    
    % Collect Intruder Points
    for ni=1:1:max_num_intruder
        if flag_intruder_active(ni,1) == 1
            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                flag_this_intruder_checked_already = 0;
                
                for n=1:1:num_agents
                    if (pos_agent(n) == bin_intruder(ni,1)) && (flag_this_intruder_checked_already == 0)
                        total_reward_earned_array(t,1) = total_reward_earned_array(t,1) + reward_checking_intruder;
                        flag_this_intruder_checked_already = 1;
                        
                        if agent_check_intruder_type(ni,1) == 0
                            agent_check_intruder_type(ni,1) = flag_intruder_type(ni,1);
                        end
                    end
                end
            end
        else
            % do nothing
            agent_check_intruder_type(ni,1) = 0;
        end
    end
    
    % Update cumulative
    if t == 1
        cumulative_total_reward_available_array(t,1) = total_reward_available_array(t,1);
        cumulative_total_reward_earned_array(t,1) = total_reward_earned_array(t,1);
    else
        cumulative_total_reward_available_array(t,1) = cumulative_total_reward_available_array(t-1,1) + total_reward_available_array(t,1);
        cumulative_total_reward_earned_array(t,1) = cumulative_total_reward_earned_array(t-1,1) + total_reward_earned_array(t,1);
    end
    fraction_total_reward_earned_array(t,1) = total_reward_earned_array(t,1)/total_reward_available_array(t,1);
    fraction_cumulative_total_reward_earned_array(t,1) = cumulative_total_reward_earned_array(t,1)/cumulative_total_reward_available_array(t,1);
    
    
    
    
    %% Selection of Next Location
    new_pos_agent = zeros(num_agents,1);
    
    switch flag_type_task_allocation
        
        
        case 1
            %% dumb hurestic
            
            all_reward = zeros(2,4); %[reward patrol/intruder bin agent]
            
            for n=1:1:num_agents
                
                this_agent_pos = pos_agent(n,1);
                this_x = mod(this_agent_pos-1,grid_num_y) + 1;
                this_y = ((this_agent_pos - this_x)/grid_num_y) + 1;
                this_agent_location = [this_x-0.5, this_y-0.5];
                
                this_agent_distance = vecnorm(patrol_grid_location - this_agent_location,2,2);
                
                this_agent_pgr = patrol_grid_reward - weight_distance*this_agent_distance;
                
                this_agent_patrol_reward = [this_agent_pgr, zeros(grid_num_x*grid_num_y,1), [1:1:grid_num_x*grid_num_y]', n*ones(grid_num_x*grid_num_y,1)];
                
                greater_than_max_distance = logical(this_agent_distance > threshold_max_distance);
                
                this_agent_patrol_reward = this_agent_patrol_reward(~greater_than_max_distance,:);
                
                all_reward = [all_reward; this_agent_patrol_reward];
            end
            
            if flag_patrol_intruder == 1
                
                for ni=1:1:max_num_intruder
                    if flag_intruder_active(ni,1) == 1
                        if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                            
                            for n=1:1:num_agents
                                
                                this_agent_pos = pos_agent(n,1);
                                this_x = mod(this_agent_pos-1,grid_num_y) + 1;
                                this_y = ((this_agent_pos - this_x)/grid_num_y) + 1;
                                this_agent_location = [this_x-0.5, this_y-0.5];
                                
                                this_agent_distance_intruder = norm(this_agent_location - pos_intruder(ni,:));
                                
                                if this_agent_distance_intruder < threshold_max_distance
                                    
                                    this_agent_intruder_reward = reward_checking_intruder - weight_distance*this_agent_distance_intruder;
                                    
                                    this_agent_intruder_reward_array = [this_agent_intruder_reward, ni, bin_intruder(ni,1), n];
                                    
                                    all_reward = [all_reward; this_agent_intruder_reward_array];
                                    
                                end
                            end
                            
                        end
                    else
                        % do nothing
                        agent_check_intruder_type(ni,1) = 0;
                    end
                end
            end
            
            [B_ac,I_ac] = sort(all_reward(:,1),'descend');
            all_reward = all_reward(I_ac,:);
            
            count_assigned_agent = 0;
            
            while count_assigned_agent < num_agents
                this_agent = all_reward(1,4);
                this_bin = all_reward(1,3);
                new_pos_agent(this_agent,1) = this_bin;
                
                if all_reward(1,2) > 0
                    this_intruder = all_reward(1,2);
                    agent_check_intruder_type(this_intruder,1) = flag_intruder_type(this_intruder,1);
                    
                    this_intruder_appears = logical(all_reward(:,2)==this_intruder);
                    all_reward = all_reward(~this_intruder_appears,:);
                end
                
                count_assigned_agent = count_assigned_agent + 1;
                this_agent_appears = logical(all_reward(:,4)==this_agent);
                all_reward = all_reward(~this_agent_appears,:);
                
                this_bin_appears = logical(all_reward(:,3)==this_bin);
                all_reward = all_reward(~this_bin_appears,:);
            end
            
            
        case 2
            %% Sec.4 Task Assignment for Homogeneous Agents to Tasks at One Time Step
            
            n_variables = grid_num_x*grid_num_y;
            
            total_grid_reward = patrol_grid_reward + reward_patrol_increment*ones(n_variables,1) ; % at next time step
            
            if flag_patrol_intruder == 1
                
                for ni=1:1:max_num_intruder
                    if flag_intruder_active(ni,1) == 1
                        if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                            % This is Intruder's current bin
                            %                             total_grid_reward(bin_intruder(ni,1)) = total_grid_reward(bin_intruder(ni,1)) + reward_checking_intruder;
                            
                            % We want intruder's next bin
                            next_pos_intruder_ni = pos_intruder(ni,:) + vel_intruder(ni,:);
                            if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                this_x = ceil(next_pos_intruder_ni(1,1));
                                this_y = ceil(next_pos_intruder_ni(1,2));
                                this_bin = (this_y-1)*grid_num_y + this_x;
                                total_grid_reward(this_bin) = total_grid_reward(this_bin) + reward_checking_intruder;
                            end
                            
                        end
                    else
                        % Do nothing
                    end
                end
                
            end
            
            pos_agent_array = zeros(n_variables,1);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n)) = 1;
            end
            
            
            % Solve the LP
            
            cvx_begin
            cvx_solver mosek
            
            variable x(n_variables,n_variables)
            
            obj = 0;
            
            for grid_j=1:1:n_variables
                obj_grid_j = 0;
                for grid_i=1:1:n_variables
                    obj_grid_j = obj_grid_j + x(grid_i,grid_j);
                end
                obj = obj + total_grid_reward(grid_j)*obj_grid_j;
            end
            
            obj_cost = 0;
            for grid_j=1:1:n_variables
                for grid_i=1:1:n_variables
                    obj_cost = obj_cost + (weight_distance*total_grid_cost(grid_i,grid_j))*x(grid_i,grid_j);
                end
            end
            
            obj = obj - obj_cost;
            
            maximize obj
            
            subject to
            
            for grid_j=1:1:n_variables
                for grid_i=1:1:n_variables
                    x(grid_i,grid_j) >= 0
                    x(grid_i,grid_j) <= 1
                end
            end
            
            for grid_j=1:1:n_variables
                sum( x(:,grid_j) ) <= 1
            end
            
            for grid_i=1:1:n_variables
                sum( x(grid_i,:) ) == pos_agent_array(grid_i)
            end
            
            cvx_end
            
            cvx_solution_array = zeros(num_agents,3);
            
            cvx_solution_count = 0;
            for grid_i=1:1:n_variables
                for grid_j=1:1:n_variables
                    if x(grid_i,grid_j) > 0
                        %                         disp('New one')
                        %                         disp(grid_i)
                        %                         disp(grid_j)
                        
                        cvx_solution_count = cvx_solution_count + 1;
                        cvx_solution_array(cvx_solution_count,:) = [grid_i grid_j x(grid_i,grid_j)];
                    end
                end
            end
            
            %             if cvx_solution_count ~= num_agents
            %                 disp('Problem here with cvx solution!')
            %             end
            
            max_transition_prob_agent = zeros(num_agents,1);
            for n=1:1:num_agents
                for nc=1:1:cvx_solution_count
                    if (pos_agent(n) == cvx_solution_array(nc,1)) && (max_transition_prob_agent(n) < cvx_solution_array(nc,3))
                        new_pos_agent(n) = cvx_solution_array(nc,2);
                        max_transition_prob_agent(n) = cvx_solution_array(nc,3);
                    end
                end
            end
            
        case 6
            %% Sec.4 Task Assignment for Homogeneous Agents to Tasks at One Time Step (Initialize Matrix Before!)
            
            %             n_variables = grid_num_x*grid_num_y;
            
            total_grid_reward = patrol_grid_reward + reward_patrol_increment*ones(n_variables,1) ; % at next time step
            
            if flag_patrol_intruder == 1
                
                for ni=1:1:max_num_intruder
                    if flag_intruder_active(ni,1) == 1
                        if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                            % This is Intruder's current bin
                            %                             total_grid_reward(bin_intruder(ni,1)) = total_grid_reward(bin_intruder(ni,1)) + reward_checking_intruder;
                            
                            % We want intruder's next bin
                            next_pos_intruder_ni = pos_intruder(ni,:) + vel_intruder(ni,:);
                            if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                this_x = ceil(next_pos_intruder_ni(1,1));
                                this_y = ceil(next_pos_intruder_ni(1,2));
                                this_bin = (this_y-1)*grid_num_y + this_x;
                                total_grid_reward(this_bin) = total_grid_reward(this_bin) + reward_checking_intruder;
                            end
                            
                        end
                    else
                        % Do nothing
                    end
                end
                
            end
            
            pos_agent_array = zeros(n_variables,1);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n)) = 1;
            end
            
            % Solve the LP
            
            f_cost = zeros(n_variables*n_variables,1);
            % this_grid = (grid_i-1)*n_variables + grid_j;
            
            for grid_j=1:1:n_variables
                for grid_i=1:1:n_variables
                    this_grid = (grid_i-1)*n_variables + grid_j;
                    f_cost(this_grid,1) = f_cost(this_grid,1) + total_grid_reward(grid_j);
                end
            end
            
            for grid_j=1:1:n_variables
                for grid_i=1:1:n_variables
                    this_grid = (grid_i-1)*n_variables + grid_j;
                    f_cost(this_grid,1) = f_cost(this_grid,1) - (weight_distance*total_grid_cost(grid_i,grid_j));
                end
            end
            
            %             lower_bound = zeros(n_variables*n_variables,1);
            %             upper_bound = ones(n_variables*n_variables,1);
            %
            %             A_inequality_entries = zeros(10,3);
            %             b_inequality = zeros(n_variables,1);
            %             count = 0;
            %             for grid_j=1:1:n_variables
            %                 for grid_i=1:1:n_variables
            %                     this_grid = (grid_i-1)*n_variables + grid_j;
            %                     count = count + 1;
            %                     A_inequality_entries(count,:) = [grid_j, this_grid, 1];
            %                 end
            %                 b_inequality(grid_j,1) = 1;
            %             end
            %             A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
            
            
            %             A_equality_entries = zeros(10,3);
            b_equality = zeros(n_variables,1);
            %             count = 0;
            for grid_i=1:1:n_variables
                %                 for grid_j=1:1:n_variables
                %                     this_grid = (grid_i-1)*n_variables + grid_j;
                %                     count = count + 1;
                %                     A_equality_entries(count,:) = [grid_i, this_grid, 1];
                %                 end
                b_equality(grid_i,1) = pos_agent_array(grid_i);
            end
            %             A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3));
            
            [x , f_val] = linprog(-f_cost,A_inequality,b_inequality,A_equality,b_equality,lower_bound,upper_bound);
            
            % Hungarian algorithm
            Hungarian_cost_array = zeros(num_agents,n_variables);
            for n=1:1:num_agents
                grid_i = pos_agent(n);
                
                for grid_j=1:1:n_variables
                    this_grid = (grid_i-1)*n_variables + grid_j;
                    Hungarian_cost_array(n,grid_j) = x(this_grid,1);
                end
            end
            [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
            
            for n=1:1:num_agents
                new_pos_agent(n) = assignment_agent(n);
            end
            
        case 3
            %% Sec.5.A Task Assignment for Homogeneous Agents to Tasks over Multiple Time Step
            
            n_variables = grid_num_x*grid_num_y;
            
            floating_task_grid_reward = zeros(n_variables,time_receding_horizon);
            fixed_task_grid_reward = zeros(n_variables,time_receding_horizon);
            
            for k=1:1:time_receding_horizon
                floating_task_grid_reward(:,k) = patrol_grid_reward + k*reward_patrol_increment*ones(n_variables,1) ; % at that time step
            end
            
            if flag_patrol_intruder == 1
                
                for k=1:1:time_receding_horizon
                    
                    inactive_intruder = 0;
                    
                    for ni=1:1:max_num_intruder
                        if flag_intruder_active(ni,1) == 1
                            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                                
                                % We want intruder's bin in k^th time step. % k = 1 is current time step, so agents cant move
                                next_pos_intruder_ni = pos_intruder(ni,:) + k*vel_intruder(ni,:);
                                
                                if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                    this_x = ceil(next_pos_intruder_ni(1,1));
                                    this_y = ceil(next_pos_intruder_ni(1,2));
                                    this_bin = (this_y-1)*grid_num_y + this_x;
                                    fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + reward_checking_intruder;
                                else
                                    inactive_intruder = inactive_intruder + 1;
                                end
                                
                            end
                        else
                            inactive_intruder = inactive_intruder + 1;
                        end
                    end
                    
                    % Spread probability of inactive intruder
                    for g_x=[1 grid_num_y]
                        for g_y=1:1:grid_num_x
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                            
                            if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + predictive_task_reward;
                            elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                floating_task_grid_reward(this_bin,k) = floating_task_grid_reward(this_bin,k) + predictive_task_reward;
                            else
                                disp('Something is wrong here!')
                            end
                        end
                    end
                    
                    for g_x=2:1:grid_num_y-1
                        for g_y=[1 grid_num_x]
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                            
                            if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + predictive_task_reward;
                            elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                floating_task_grid_reward(this_bin,k) = floating_task_grid_reward(this_bin,k) + predictive_task_reward;
                            else
                                disp('Something is wrong here!')
                            end
                        end
                    end
                    
                    
                end
                
            end
            
            pos_agent_array = zeros(n_variables,1);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n)) = 1;
            end
            
            
            % Solve the LP
            
            cvx_begin
            cvx_solver mosek
            
            variables x(n_variables,n_variables,time_receding_horizon) y(n_variables,time_receding_horizon)
            
            obj = 0;
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    obj_grid_j = 0;
                    for grid_i=1:1:n_variables
                        obj_grid_j = obj_grid_j + x(grid_i,grid_j,k);
                    end
                    obj = obj + fixed_task_grid_reward(grid_j,k)*obj_grid_j;
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    obj = obj + floating_task_grid_reward(grid_j,k)*y(grid_j,k);
                end
            end
            
            obj_cost = 0;
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        obj_cost = obj_cost + (weight_distance*total_grid_cost(grid_i,grid_j))*x(grid_i,grid_j,k);
                    end
                end
            end
            
            obj = obj - obj_cost;
            
            maximize obj
            
            subject to
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        x(grid_i,grid_j,k) >= 0
                        x(grid_i,grid_j,k) <= 1
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    sum( x(:,grid_j,k) ) <= 1
                end
            end
            
            for grid_i=1:1:n_variables
                sum( x(grid_i,:,1) ) == pos_agent_array(grid_i)
            end
            
            for k=1:1:time_receding_horizon-1
                for grid_j=1:1:n_variables
                    sum( x(:,grid_j,k) ) == sum( x(grid_j,:,k+1) )
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    y(grid_j,k) >= 0
                    y(grid_j,k) <= 1
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    y(grid_j,k) <= sum( x(:,grid_j,k) )
                end
            end
            
            for grid_j=1:1:n_variables
                sum(y(grid_j,:)) <= 1
            end
            
            cvx_end
            
            
            %             % Old method (causes agents to go to same cell)
            %             cvx_solution_array = zeros(num_agents,3);
            %
            %             cvx_solution_count = 0;
            %             for grid_i=1:1:n_variables
            %                 for grid_j=1:1:n_variables
            %                     if x(grid_i,grid_j) > 0
            %                         %                         disp('New one')
            %                         %                         disp(grid_i)
            %                         %                         disp(grid_j)
            %
            %                         cvx_solution_count = cvx_solution_count + 1;
            %                         cvx_solution_array(cvx_solution_count,:) = [grid_i grid_j x(grid_i,grid_j,1)];
            %                     end
            %                 end
            %             end
            %
            %             %             if cvx_solution_count ~= num_agents
            %             %                 disp('Problem here with cvx solution!')
            %             %             end
            %
            %             max_transition_prob_agent = zeros(num_agents,1);
            %             for n=1:1:num_agents
            %                 for nc=1:1:cvx_solution_count
            %                     if (pos_agent(n) == cvx_solution_array(nc,1)) && (max_transition_prob_agent(n) < cvx_solution_array(nc,3))
            %                         new_pos_agent(n) = cvx_solution_array(nc,2);
            %                         max_transition_prob_agent(n) = cvx_solution_array(nc,3);
            %                     end
            %                 end
            %             end
            
            % New method (Hungarian algorithm)
            Hungarian_cost_array = zeros(num_agents,n_variables);
            for n=1:1:num_agents
                Hungarian_cost_array(n,:) = x(pos_agent(n),:,1);
            end
            [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
            
            for n=1:1:num_agents
                new_pos_agent(n) = assignment_agent(n);
            end
            
        case 7
            %% Sec.5.A Task Assignment for Homogeneous Agents to Tasks over Multiple Time Step (Initialize Matrix Before)
            
            %             n_variables = grid_num_x*grid_num_y;
            
            floating_task_grid_reward = zeros(n_variables,time_receding_horizon);
            fixed_task_grid_reward = zeros(n_variables,time_receding_horizon);
            
            for k=1:1:time_receding_horizon
                floating_task_grid_reward(:,k) = patrol_grid_reward + k*reward_patrol_increment*ones(n_variables,1) ; % at that time step
            end
            
            if flag_patrol_intruder == 1
                
                for k=1:1:time_receding_horizon
                    
                    inactive_intruder = 0;
                    
                    for ni=1:1:max_num_intruder
                        if flag_intruder_active(ni,1) == 1
                            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                                
                                % We want intruder's bin in k^th time step. % k = 1 is current time step, so agents cant move
                                next_pos_intruder_ni = pos_intruder(ni,:) + k*vel_intruder(ni,:);
                                
                                if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                    this_x = ceil(next_pos_intruder_ni(1,1));
                                    this_y = ceil(next_pos_intruder_ni(1,2));
                                    this_bin = (this_y-1)*grid_num_y + this_x;
                                    fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + reward_checking_intruder;
                                else
                                    inactive_intruder = inactive_intruder + 1;
                                end
                                
                            end
                        else
                            inactive_intruder = inactive_intruder + 1;
                        end
                    end
                    
                    % Spread probability of inactive intruder
                    for g_x=[1 grid_num_y]
                        for g_y=1:1:grid_num_x
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                            
                            if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + predictive_task_reward;
                            elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                floating_task_grid_reward(this_bin,k) = floating_task_grid_reward(this_bin,k) + predictive_task_reward;
                            else
                                disp('Something is wrong here!')
                            end
                        end
                    end
                    
                    for g_x=2:1:grid_num_y-1
                        for g_y=[1 grid_num_x]
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                            
                            if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + predictive_task_reward;
                            elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                floating_task_grid_reward(this_bin,k) = floating_task_grid_reward(this_bin,k) + predictive_task_reward;
                            else
                                disp('Something is wrong here!')
                            end
                        end
                    end
                    
                    
                end
                
            end
            
            pos_agent_array = zeros(n_variables,1);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n)) = 1;
            end
            
            
            
            f_cost = zeros(time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables,1);
            % for x this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            % for y this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        f_cost(this_grid,1) = f_cost(this_grid,1) + fixed_task_grid_reward(grid_j,k);
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j;
                    f_cost(this_grid,1) = f_cost(this_grid,1) + floating_task_grid_reward(grid_j,k);
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        f_cost(this_grid,1) = f_cost(this_grid,1) - (weight_distance*total_grid_cost(grid_i,grid_j));
                    end
                end
            end
            
            %             lower_bound = zeros(time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables,1);
            %             upper_bound = ones(time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables,1);
            %
            %             A_inequality_entries = zeros(10,3);
            %             b_inequality = zeros(n_variables,1);
            %             count = 0;
            %             count_b = 0;
            %             for k=1:1:time_receding_horizon
            %                 for grid_j=1:1:n_variables
            %                     count_b = count_b + 1;
            %                     for grid_i=1:1:n_variables
            %                         this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            %                         count = count + 1;
            %                         A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %                     end
            %                     b_inequality(count_b,1) = 1;
            %                 end
            %             end
            %
            %             for grid_j=1:1:n_variables
            %                 count_b = count_b + 1;
            %                 for k=1:1:time_receding_horizon
            %                     this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j;
            %                     count = count + 1;
            %                     A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %                 end
            %                 b_inequality(count_b,1) = 1;
            %             end
            %
            %             for k=1:1:time_receding_horizon
            %                 for grid_j=1:1:n_variables
            %                     count_b = count_b + 1;
            %                     this_grid = time_receding_horizon*n_variables*n_variables + (k-1)*n_variables + grid_j;
            %                     count = count + 1;
            %                     A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %                     for grid_i=1:1:n_variables
            %                         this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            %                         count = count + 1;
            %                         A_inequality_entries(count,:) = [count_b, this_grid, -1];
            %                     end
            %                     b_inequality(count_b,1) = 0;
            %                 end
            %             end
            %             A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
            %
            %             A_equality_entries = zeros(10,3);
            b_equality = zeros(n_variables,1);
            %             count = 0;
            count_b = 0;
            for grid_i=1:1:n_variables
                count_b = count_b + 1;
                %                 k = 1;
                %                 for grid_j=1:1:n_variables
                %                     this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                %                     count = count + 1;
                %                     A_equality_entries(count,:) = [count_b, this_grid, 1];
                %                 end
                b_equality(count_b,1) = pos_agent_array(grid_i);
            end
            %
            for k=1:1:time_receding_horizon-1
                for grid_j=1:1:n_variables
                    count_b = count_b + 1;
                    %                     for grid_i=1:1:n_variables
                    %                         this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    %                         count = count + 1;
                    %                         A_equality_entries(count,:) = [count_b, this_grid, 1];
                    %
                    %                         this_grid_2 = (k+1-1)*n_variables*n_variables + (grid_j-1)*n_variables + grid_i;
                    %                         count = count + 1;
                    %                         A_equality_entries(count,:) = [count_b, this_grid_2, -1];
                    %                     end
                    b_equality(count_b,1) = 0;
                end
            end
            %             A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count_b,time_receding_horizon*n_variables*n_variables + time_receding_horizon*n_variables);
            
            [x , f_val] = linprog(-f_cost,A_inequality,b_inequality,A_equality,b_equality,lower_bound,upper_bound);
            
            % Hungarian algorithm
            Hungarian_cost_array = zeros(num_agents,n_variables);
            for n=1:1:num_agents
                grid_i = pos_agent(n);
                k = 1;
                for grid_j=1:1:n_variables
                    this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    Hungarian_cost_array(n,grid_j) = x(this_grid,1);
                end
            end
            [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
            
            for n=1:1:num_agents
                new_pos_agent(n) = assignment_agent(n);
            end
            
            if flag_show_probability_cloud == 1
                probability_cloud = zeros(n_variables,time_receding_horizon);
                
                for k=1:1:time_receding_horizon
                    for grid_j=1:1:n_variables
                        this_prob = 0;
                        for grid_i=1:1:n_variables
                            this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                            this_prob = this_prob + x(this_grid,1);
                        end
                        probability_cloud(grid_j,k) = floor(this_prob/threshold_probability);
                    end
                end
            end
            
        case 4
            %% Sec.6.A Task Assignment for Heterogeneous Agents to Tasks over Multiple Time Step
            
            n_variables = grid_num_x*grid_num_y;
            
            floating_task_grid_reward = zeros(n_variables,time_receding_horizon,num_agents);
            fixed_task_grid_reward = zeros(n_variables,time_receding_horizon,num_agents);
            
            for k=1:1:time_receding_horizon
                for n=1:1:num_agents
                    floating_task_grid_reward(:,k,n) = patrol_grid_reward + k*reward_patrol_increment*ones(n_variables,1) ; % at that time step
                end
            end
            
            if flag_patrol_intruder == 1
                
                for k=1:1:time_receding_horizon
                    
                    inactive_intruder = 0;
                    
                    for ni=1:1:max_num_intruder
                        if flag_intruder_active(ni,1) == 1
                            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                                
                                % We want intruder's bin in k^th time step. % k = 1 is current time step, so agents cant move
                                next_pos_intruder_ni = pos_intruder(ni,:) + k*vel_intruder(ni,:);
                                
                                if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                    this_x = ceil(next_pos_intruder_ni(1,1));
                                    this_y = ceil(next_pos_intruder_ni(1,2));
                                    this_bin = (this_y-1)*grid_num_y + this_x;
                                    for n=1:1:num_agents
                                        fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + reward_checking_intruder;
                                    end
                                else
                                    inactive_intruder = inactive_intruder + 1;
                                end
                                
                            end
                        else
                            inactive_intruder = inactive_intruder + 1;
                        end
                    end
                    
                    % Spread probability of inactive intruder
                    for g_x=[1 grid_num_y]
                        for g_y=1:1:grid_num_x
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            for n=1:1:num_agents
                                predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                                
                                if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                    fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + predictive_task_reward;
                                elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                    floating_task_grid_reward(this_bin,k,n) = floating_task_grid_reward(this_bin,k,n) +  predictive_task_reward;
                                else
                                    disp('Something is wrong here!')
                                end
                            end
                        end
                    end
                    
                    for g_x=2:1:grid_num_y-1
                        for g_y=[1 grid_num_x]
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            for n=1:1:num_agents
                                predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                                
                                if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                    fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + predictive_task_reward;
                                elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                    floating_task_grid_reward(this_bin,k,n) = floating_task_grid_reward(this_bin,k,n) +  predictive_task_reward;
                                else
                                    disp('Something is wrong here!')
                                end
                            end
                        end
                    end
                    
                    
                end
                
            end
            
            pos_agent_array = zeros(n_variables,num_agents);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n),n) = 1;
            end
            
            % Solve the LP
            
            cvx_begin
            cvx_solver mosek
            
            variables x(n_variables,n_variables,time_receding_horizon,num_agents) y(n_variables,time_receding_horizon,num_agents)
            
            obj = 0;
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        %                         obj_grid_j = 0;
                        %                         for grid_i=1:1:n_variables
                        %                             obj_grid_j = obj_grid_j + x(grid_i,grid_j,k,n);
                        %                         end
                        obj = obj + fixed_task_grid_reward(grid_j,k,n)*sum(x(:,grid_j,k,n));
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        obj = obj + floating_task_grid_reward(grid_j,k,n)*y(grid_j,k,n);
                    end
                end
            end
            
            obj_cost = 0;
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        for n=1:1:num_agents
                            obj_cost = obj_cost + (weight_distance*total_grid_cost(grid_i,grid_j))*x(grid_i,grid_j,k,n);
                        end
                    end
                end
            end
            
            obj = obj - obj_cost;
            
            maximize obj
            
            subject to
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        for n=1:1:num_agents
                            x(grid_i,grid_j,k,n) >= 0
                            x(grid_i,grid_j,k,n) <= 1
                        end
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    sum(sum( x(:,grid_j,k,:) )) <= 1
                end
            end
            
            for grid_i=1:1:n_variables
                for n=1:1:num_agents
                    sum( x(grid_i,:,1,n) ) == pos_agent_array(grid_i,n)
                end
            end
            
            for k=1:1:time_receding_horizon-1
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        sum( x(:,grid_j,k,n) ) == sum( x(grid_j,:,k+1,n) )
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        y(grid_j,k,n) >= 0
                        y(grid_j,k,n) <= 1
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        y(grid_j,k,n) <= sum( x(:,grid_j,k,n) )
                    end
                end
            end
            
            for grid_j=1:1:n_variables
                sum(sum(y(grid_j,:,:))) <= 1
            end
            
            cvx_end
            
            
            
            % New method (Hungarian algorithm)
            Hungarian_cost_array = zeros(num_agents,n_variables);
            for n=1:1:num_agents
                Hungarian_cost_array(n,:) = x(pos_agent(n),:,1,n);
            end
            [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
            
            for n=1:1:num_agents
                new_pos_agent(n) = assignment_agent(n);
            end
            
        case 8
            %% Sec.6.A Task Assignment for Heterogeneous Agents to Tasks over Multiple Time Step (Initialize Matrix Before!)
            
            %             n_variables = grid_num_x*grid_num_y;
            
            floating_task_grid_reward = zeros(n_variables,time_receding_horizon,num_agents);
            fixed_task_grid_reward = zeros(n_variables,time_receding_horizon,num_agents);
            
            for k=1:1:time_receding_horizon
                for n=1:1:num_agents
                    floating_task_grid_reward(:,k,n) = patrol_grid_reward + k*reward_patrol_increment*ones(n_variables,1) ; % at that time step
                end
            end
            
            if flag_patrol_intruder == 1
                
                for k=1:1:time_receding_horizon
                    
                    inactive_intruder = 0;
                    
                    for ni=1:1:max_num_intruder
                        if flag_intruder_active(ni,1) == 1
                            if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                                
                                % We want intruder's bin in k^th time step. % k = 1 is current time step, so agents cant move
                                next_pos_intruder_ni = pos_intruder(ni,:) + k*vel_intruder(ni,:);
                                
                                if (next_pos_intruder_ni(1,1) >= 0) && (next_pos_intruder_ni(1,2) >= 0) && (next_pos_intruder_ni(1,1) <= grid_num_y) && (next_pos_intruder_ni(1,2) <= grid_num_x)
                                    this_x = ceil(next_pos_intruder_ni(1,1));
                                    this_y = ceil(next_pos_intruder_ni(1,2));
                                    this_bin = (this_y-1)*grid_num_y + this_x;
                                    for n=1:1:num_agents
                                        fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + reward_checking_intruder;
                                    end
                                else
                                    inactive_intruder = inactive_intruder + 1;
                                end
                                
                            end
                        else
                            inactive_intruder = inactive_intruder + 1;
                        end
                    end
                    
                    % Spread probability of inactive intruder
                    for g_x=[1 grid_num_y]
                        for g_y=1:1:grid_num_x
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            for n=1:1:num_agents
                                predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                                
                                if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                    fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + predictive_task_reward;
                                elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                    floating_task_grid_reward(this_bin,k,n) = floating_task_grid_reward(this_bin,k,n) +  predictive_task_reward;
                                else
                                    disp('Something is wrong here!')
                                end
                            end
                        end
                    end
                    
                    for g_x=2:1:grid_num_y-1
                        for g_y=[1 grid_num_x]
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            for n=1:1:num_agents
                                predictive_task_reward = probability_create_intruder*inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                                
                                if flag_predictive_task_fixed_floating == 1 % predctive task is fixed task
                                    fixed_task_grid_reward(this_bin,k,n) = fixed_task_grid_reward(this_bin,k,n) + predictive_task_reward;
                                elseif flag_predictive_task_fixed_floating == 2 % predctive task is floating task
                                    floating_task_grid_reward(this_bin,k,n) = floating_task_grid_reward(this_bin,k,n) +  predictive_task_reward;
                                else
                                    disp('Something is wrong here!')
                                end
                            end
                        end
                    end
                    
                    
                end
                
            end
            
            pos_agent_array = zeros(n_variables,num_agents);
            for n=1:1:num_agents
                pos_agent_array(pos_agent(n),n) = 1;
            end
            
            % Solve the LP
            
            f_cost = zeros(num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables,1);
            % for x for n agent this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            % for y for n agent this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        for grid_i=1:1:n_variables
                            this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                            f_cost(this_grid,1) = f_cost(this_grid,1) + fixed_task_grid_reward(grid_j,k,n);
                        end
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j;
                        f_cost(this_grid,1) = f_cost(this_grid,1) + floating_task_grid_reward(grid_j,k,n);
                    end
                end
            end
            
            for k=1:1:time_receding_horizon
                for grid_j=1:1:n_variables
                    for grid_i=1:1:n_variables
                        for n=1:1:num_agents
                            this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                            f_cost(this_grid,1) = f_cost(this_grid,1) - (weight_distance*total_grid_cost(grid_i,grid_j));
                        end
                    end
                end
            end
            
            %             lower_bound = zeros(num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables,1);
            %             upper_bound = ones(num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables,1);
            %
            %
            %             A_inequality_entries = zeros(10,3);
            %             b_inequality = zeros(n_variables,1);
            %             count = 0;
            %             count_b = 0;
            %
            %             for k=1:1:time_receding_horizon
            %                 for grid_j=1:1:n_variables
            %                     count_b = count_b + 1;
            %
            %                     for grid_i=1:1:n_variables
            %                         for n=1:1:num_agents
            %                             this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            %                             count = count + 1;
            %                             A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %                         end
            %                     end
            %                     b_inequality(count_b,1) = 1;
            %                 end
            %             end
            %
            %             for grid_j=1:1:n_variables
            %                 count_b = count_b + 1;
            %
            %                 for k=1:1:time_receding_horizon
            %                     for n=1:1:num_agents
            %                         this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j;
            %                         count = count + 1;
            %                         A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %                     end
            %                 end
            %
            %                 b_inequality(count_b,1) = 1;
            %             end
            %
            %
            %             for k=1:1:time_receding_horizon
            %                 for grid_j=1:1:n_variables
            %                     for n=1:1:num_agents
            %                         count_b = count_b + 1;
            %
            %                         this_grid = num_agents*time_receding_horizon*n_variables*n_variables + (n-1)*time_receding_horizon*n_variables + (k-1)*n_variables + grid_j;
            %                         count = count + 1;
            %                         A_inequality_entries(count,:) = [count_b, this_grid, 1];
            %
            %                         for grid_i=1:1:n_variables
            %                             this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
            %                             count = count + 1;
            %                             A_inequality_entries(count,:) = [count_b, this_grid, -1];
            %                         end
            %
            %                         b_inequality(count_b,1) = 0;
            %                     end
            %                 end
            %             end
            %
            %             A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
            %
            %             A_equality_entries = zeros(10,3);
            b_equality = zeros(n_variables,1);
            %             count = 0;
            count_b = 0;
            %
            for grid_i=1:1:n_variables
                for n=1:1:num_agents
                    count_b = count_b + 1;
                    %
                    %                     k=1;
                    %                     for grid_j=1:1:n_variables
                    %                         this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    %                         count = count + 1;
                    %                         A_equality_entries(count,:) = [count_b, this_grid, 1];
                    %                     end
                    %
                    b_equality(count_b,1) = pos_agent_array(grid_i,n);
                end
            end
            %
            for k=1:1:time_receding_horizon-1
                for grid_j=1:1:n_variables
                    for n=1:1:num_agents
                        count_b = count_b + 1;
                        %
                        %                         for grid_i=1:1:n_variables
                        %                             this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                        %                             count = count + 1;
                        %                             A_equality_entries(count,:) = [count_b, this_grid, 1];
                        %
                        %                             this_grid_2 = (n-1)*time_receding_horizon*n_variables*n_variables + (k+1-1)*n_variables*n_variables + (grid_j-1)*n_variables + grid_i;
                        %                             count = count + 1;
                        %                             A_equality_entries(count,:) = [count_b, this_grid_2, -1];
                        %                         end
                        %
                        b_equality(count_b,1) = 0;
                    end
                end
            end
            %
            %             A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count_b,num_agents*time_receding_horizon*n_variables*n_variables + num_agents*time_receding_horizon*n_variables);
            %
            [x , f_val] = linprog(-f_cost,A_inequality,b_inequality,A_equality,b_equality,lower_bound,upper_bound);
            
            % Hungarian algorithm
            Hungarian_cost_array = zeros(num_agents,n_variables);
            for n=1:1:num_agents
                grid_i = pos_agent(n);
                k = 1;
                for grid_j=1:1:n_variables
                    this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    Hungarian_cost_array(n,grid_j) = x(this_grid,1);
                end
            end
            [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
            
            for n=1:1:num_agents
                new_pos_agent(n) = assignment_agent(n);
            end
            
            if flag_show_probability_cloud == 1
                probability_cloud = zeros(n_variables,time_receding_horizon,num_agents);
                
                for n=1:1:num_agents
                    for k=1:1:time_receding_horizon
                        for grid_j=1:1:n_variables
                            this_prob = 0;
                            for grid_i=1:1:n_variables
                                this_grid = (n-1)*time_receding_horizon*n_variables*n_variables + (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                this_prob = this_prob + x(this_grid,1);
                            end
                            probability_cloud(grid_j,k,n) = floor(this_prob/threshold_probability);
                        end
                    end
                end
            end
            
        case 9
            %% Generate and visualize stored rand solution
            
            if flag_generate_visualize_store_rand_solution == 1 % Generate solution
                flag_intruder_active_array(:,t) = flag_intruder_active;
                bin_intruder_array(:,t) = bin_intruder;
                flag_intruder_type_array(:,t) = flag_intruder_type;
                
                if t == t_final
                    
                    this_reward_array = zeros(max_num_intruder,1);
                    
                    n_variables = grid_num_x*grid_num_y;
                    
                    floating_task_grid_reward = zeros(n_variables,t_final);
                    fixed_task_grid_reward = zeros(n_variables,t_final);
                    
                    %                     for k=1:1:time_receding_horizon
                    %                         floating_task_grid_reward(:,k) = patrol_grid_reward + k*reward_patrol_increment*ones(n_variables,1) ; % at that time step
                    %                     end
                    
                    if flag_patrol_intruder == 1
                        
                        for k=1:1:t_final-1
                            
                            inactive_intruder = 0;
                            
                            for ni=1:1:max_num_intruder
                                if flag_intruder_active_array(ni,k+1) == 1
                                    
                                    if (flag_intruder_type_array(ni,k+1) == 2) || (flag_intruder_type_array(ni,k+1) == 1)
                                        this_bin = bin_intruder_array(ni,k+1);
                                        fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + reward_checking_intruder;
                                        this_reward_array(ni,1) = 0;
                                        
                                    elseif flag_intruder_type_array(ni,k+1) == 1
                                        this_bin = bin_intruder_array(ni,k+1);
                                        this_reward = 0;
                                        if k == 1
                                            this_reward = reward_checking_intruder;
                                        elseif flag_intruder_type_array(ni,k) == 0
                                            this_reward = reward_checking_intruder;
                                        elseif flag_intruder_type_array(ni,k) == 1
                                            this_reward = 0.9*this_reward_array(ni,1);
                                        else
                                            disp('Error with this_reward')
                                        end
                                        this_reward_array(ni,1) = this_reward;
                                        
                                        floating_task_grid_reward(this_bin,k) = floating_task_grid_reward(this_bin,k) + this_reward;
                                    else
                                        disp('Something is wrong here!')
                                    end
                                    
                                else
                                    this_reward_array(ni,1) = 0;
                                end
                            end
                            
                        end
                    end
                    
                    pos_agent_array = zeros(n_variables,1);
                    for n=1:1:num_agents
                        pos_agent_array(pos_agent_original(n)) = 1;
                    end
                    
                    
                    f_cost = zeros(t_final*n_variables*n_variables + t_final*n_variables,1);
                    % for x this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                    % for y this_grid = t_final*n_variables*n_variables + (k-1)*n_variables + grid_j
                    
                    for k=1:1:t_final
                        for grid_j=1:1:n_variables
                            for grid_i=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                f_cost(this_grid,1) = f_cost(this_grid,1) + fixed_task_grid_reward(grid_j,k);
                            end
                        end
                    end
                    
                    for k=1:1:t_final
                        for grid_j=1:1:n_variables
                            this_grid = t_final*n_variables*n_variables + (k-1)*n_variables + grid_j;
                            f_cost(this_grid,1) = f_cost(this_grid,1) + floating_task_grid_reward(grid_j,k);
                        end
                    end
                    
                    for k=1:1:t_final
                        for grid_j=1:1:n_variables
                            for grid_i=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                f_cost(this_grid,1) = f_cost(this_grid,1) - (weight_distance*total_grid_cost(grid_i,grid_j));
                            end
                        end
                    end
                    
                    lower_bound = zeros(t_final*n_variables*n_variables + t_final*n_variables,1);
                    upper_bound = ones(t_final*n_variables*n_variables + t_final*n_variables,1);
                    
                    disp('Writing Inequality Entries!')
                    A_inequality_entries = zeros(n_variables,3);
                    b_inequality = zeros(n_variables,1);
                    count = 0;
                    count_b = 0;
                    for k=1:1:t_final
                        for grid_j=1:1:n_variables
                            count_b = count_b + 1;
                            for grid_i=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                count = count + 1;
                                A_inequality_entries(count,:) = [count_b, this_grid, 1];
                            end
                            b_inequality(count_b,1) = 1;
                        end
                    end
                    
                    for grid_j=1:1:n_variables
                        count_b = count_b + 1;
                        for k=1:1:t_final
                            this_grid = t_final*n_variables*n_variables + (k-1)*n_variables + grid_j;
                            count = count + 1;
                            A_inequality_entries(count,:) = [count_b, this_grid, 1];
                        end
                        b_inequality(count_b,1) = 1;
                    end
                    
                    for k=1:1:t_final
                        for grid_j=1:1:n_variables
                            count_b = count_b + 1;
                            this_grid = t_final*n_variables*n_variables + (k-1)*n_variables + grid_j;
                            count = count + 1;
                            A_inequality_entries(count,:) = [count_b, this_grid, 1];
                            for grid_i=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                count = count + 1;
                                A_inequality_entries(count,:) = [count_b, this_grid, -1];
                            end
                            b_inequality(count_b,1) = 0;
                        end
                    end
                    A_inequality = sparse(A_inequality_entries(:,1),A_inequality_entries(:,2),A_inequality_entries(:,3));
                    
                    disp('Writing Equality Entries!')
                    A_equality_entries = zeros(n_variables,3);
                    b_equality = zeros(n_variables,1);
                    count = 0;
                    count_b = 0;
                    for grid_i=1:1:n_variables
                        count_b = count_b + 1;
                        k = 1;
                        for grid_j=1:1:n_variables
                            this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                            count = count + 1;
                            A_equality_entries(count,:) = [count_b, this_grid, 1];
                        end
                        b_equality(count_b,1) = pos_agent_array(grid_i);
                    end
                    
                    for k=1:1:t_final-1
                        for grid_j=1:1:n_variables
                            count_b = count_b + 1;
                            for grid_i=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                count = count + 1;
                                A_equality_entries(count,:) = [count_b, this_grid, 1];
                                
                                this_grid_2 = (k+1-1)*n_variables*n_variables + (grid_j-1)*n_variables + grid_i;
                                count = count + 1;
                                A_equality_entries(count,:) = [count_b, this_grid_2, -1];
                            end
                            b_equality(count_b,1) = 0;
                        end
                    end
                    A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count_b,t_final*n_variables*n_variables + t_final*n_variables);
                    
                    disp('Solving Big LP!')
                    [x , f_val] = linprog(-f_cost,A_inequality,b_inequality,A_equality,b_equality,lower_bound,upper_bound);
                    disp(f_val)
                    
                    
                    pos_agent_array_omniscient = zeros(num_agents,t_final);
                    pos_agent = pos_agent_original;
                    % Hungarian algorithm
                    for k=1:1:t_final
                        new_pos_agent = zeros(num_agents,1);
                        Hungarian_cost_array = zeros(num_agents,n_variables);
                        for n=1:1:num_agents
                            grid_i = pos_agent(n);
                            for grid_j=1:1:n_variables
                                this_grid = (k-1)*n_variables*n_variables + (grid_i-1)*n_variables + grid_j;
                                Hungarian_cost_array(n,grid_j) = x(this_grid,1);
                            end
                        end
                        [assignment_agent,total_Hungarian_cost] = munkres(-Hungarian_cost_array);
                        
                        for n=1:1:num_agents
                            new_pos_agent(n) = assignment_agent(n);
                        end
                        pos_agent_array_omniscient(:,k) = new_pos_agent;
                        pos_agent = new_pos_agent;
                    end
                    %                     new_pos_agent_array(:,t_final) = new_pos_agent_array(:,t_final-1);
                    
                    save('pos_agent_array_omniscient.mat','pos_agent_array_omniscient')
                    
                    close all
                    save('Big_LP_Sol.mat')
                    
                end
                
                pos_agent = pos_agent_original;
                new_pos_agent = pos_agent_original;
                
            elseif flag_generate_visualize_store_rand_solution == 2 % visulaize solution
                new_pos_agent = pos_agent_array_omniscient(:,t);
                %                 if t < t_final
                %                     new_pos_agent = pos_agent_array(:,t+1);
                %                 else
                %                     new_pos_agent = pos_agent_array(:,t_final);
                %                 end
            else
                disp('Case 9: Something is wrong here!')
            end
            
        otherwise
            disp('Should not reach here!')
            
    end
    
    
    % Plotting
    
    h4=figure(1);
    clf
    set(h4,'Color',[1 1 1]);
    set(h4,'units','normalized','outerposition',[0 0 1 1])
    set(h4,'PaperPositionMode','auto');
    
    subplot(2,4,[1 2 5 6])
    hold on
    
    for gi = 0:1:grid_num_x
        plot([0 grid_num_y],[gi gi],'-k','LineWidth',1.0)
    end
    for gj = 0:1:grid_num_y
        plot([gj gj],[0 grid_num_x],'-k','LineWidth',1.0)
    end
    for g_x=1:1:grid_num_x
        for g_y=1:1:grid_num_y
            if patrol_grid_reward((g_x-1)*grid_num_y+g_y,1) > 0
                plot(g_y-0.5,g_x-0.5,'or','MarkerSize',patrol_grid_reward((g_x-1)*grid_num_y+g_y,1))
            end
        end
    end
    
    if flag_show_probability_cloud == 1
        switch flag_type_task_allocation
            
            case 7
                for k=1:1:time_receding_horizon
                    for grid_j=1:1:n_variables
                        this_grid_show_prob = probability_cloud(grid_j,k);
                        
                        if this_grid_show_prob > 0
                            this_x = mod(grid_j-1,grid_num_y) + 1;
                            this_y = ((grid_j - this_x)/grid_num_y) + 1;
                            rand_pos = rand(this_grid_show_prob,2);
                            plot(rand_pos(:,1)+(this_x-1), rand_pos(:,2)+(this_y-1), 'ok','MarkerSize',2,'MarkerFaceColor','k')
                        end
                    end
                end
                
            case 8
                for n=1:1:num_agents
                    for k=1:1:time_receding_horizon
                        for grid_j=1:1:n_variables
                            this_grid_show_prob = probability_cloud(grid_j,k,n);
                            
                            if this_grid_show_prob > 0
                                this_x = mod(grid_j-1,grid_num_y) + 1;
                                this_y = ((grid_j - this_x)/grid_num_y) + 1;
                                rand_pos = rand(this_grid_show_prob,2);
                                plot(rand_pos(:,1)+(this_x-1), rand_pos(:,2)+(this_y-1), 'o','MarkerSize',2,'MarkerFaceColor',color_array(mod(n,length(color_array))+1),'MarkerEdgeColor',color_array(mod(n,length(color_array))+1))
                            end
                        end
                    end
                end
                
            otherwise
                % Do nothing
        end
    end
    
    for n=1:1:num_agents
        this_agent_pos = pos_agent(n,1);
        this_x = mod(this_agent_pos-1,grid_num_y) + 1;
        this_y = ((this_agent_pos - this_x)/grid_num_y) + 1;
        plot(this_x-0.5, this_y-0.5, 'sk','MarkerSize',20,'MarkerFaceColor',color_array(mod(n,length(color_array))+1))
        
        new_agent_pos = new_pos_agent(n,1);
        new_j = mod(new_agent_pos-1,grid_num_y) + 1;
        new_i = ((new_agent_pos - new_j)/grid_num_y) + 1;
        plot([this_x-0.5 new_j-0.5], [this_y-0.5 new_i-0.5], '-','LineWidth',2.0,'Color',color_array(mod(n,length(color_array))+1))
    end
    
    for ni=1:1:max_num_intruder
        if flag_intruder_active(ni,1) == 1
            if agent_check_intruder_type(ni,1) == 0
                if flag_intruder_type(ni,1) == 1
                    color_face_type = 'k';
                elseif flag_intruder_type(ni,1) == 2
                    color_face_type = 'r';
                else
                    disp('Should not reach here! 4')
                end
                color_edge_type = [0.6 0.6 0.6];
            elseif agent_check_intruder_type(ni,1) == 1
                color_face_type = 'k'; color_edge_type = 'k';
            elseif agent_check_intruder_type(ni,1) == 2
                color_face_type = 'r'; color_edge_type = 'r';
            else
                disp('Should not reach here! 3')
            end
            plot(pos_intruder(ni,1),pos_intruder(ni,2),'o','MarkerSize',20,'MarkerFaceColor',color_edge_type,'MarkerEdgeColor',color_edge_type)
            plot(pos_intruder(ni,1),pos_intruder(ni,2),'o','MarkerSize',5,'MarkerFaceColor',color_face_type,'MarkerEdgeColor',color_face_type)
            plot([pos_intruder(ni,1) pos_intruder(ni,1)+vel_intruder(ni,1)],[pos_intruder(ni,2) pos_intruder(ni,2)+vel_intruder(ni,2)],'-','LineWidth',2,'Color',color_face_type)
        end
    end
    
    
    xlabel('X axis','fontsize',standard_font_size,'FontName','Times New Roman')
    ylabel('Y axis','fontsize',standard_font_size,'FontName','Times New Roman')
    title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    xlim([-1 grid_num_y+1])
    ylim([-1 grid_num_x+1])
    axis equal
    hold off
    
    
    subplot(2,4,3)
    hold on
    
    plot([1:1:t],total_reward_available_array(1:t,1),'-k','LineWidth',2)
    plot([1:1:t],total_reward_earned_array(1:t,1),'-r','LineWidth',2)
    legend('Available','Earned')
    xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
    ylabel('Reward','fontsize',standard_font_size,'FontName','Times New Roman')
    %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(2,4,7)
    hold on
    
    plot([1:1:t],fraction_total_reward_earned_array(1:t,1),'-r','LineWidth',2)
    %     legend('Available','Earned')
    xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
    ylabel('Fraction Reward','fontsize',standard_font_size,'FontName','Times New Roman')
    %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
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
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    hold off
    
    subplot(2,4,8)
    hold on
    
    plot([1:1:t],fraction_cumulative_total_reward_earned_array(1:t,1),'-r','LineWidth',2)
    %     legend('Available','Earned')
    xlabel('Time Index','fontsize',standard_font_size,'FontName','Times New Roman')
    ylabel('Fraction Cumulative Reward','fontsize',standard_font_size,'FontName','Times New Roman')
    %     title(['Grid Status, Time Index = ',num2str(t)],'fontsize',title_font_size,'FontName','Times New Roman')
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    hold off
    
    
    drawnow limitrate
    
    if (flag_store_video == 1)
        F = getframe(h4);
        writeVideo(myVideo1, F);
    end
    
    
    % Update Position
    pos_agent = new_pos_agent;
    
end

if (flag_store_video == 1)
    close(myVideo1);
end



