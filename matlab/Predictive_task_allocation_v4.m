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

flag_type_task_allocation = 3; % 1: Dumb Heuristic , 2: Task Assignment (1 step) Section 4, 3: Task Assignment (multi-step) Section 5,

reward_checking_intruder = 500;

reward_patrol_increment = 0.25;

time_receding_horizon = 2; % integer >= 1

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
            this_location = ceil(rand*grid_num_x*grid_num_y);
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

t_final = 300;
total_reward_available_array = zeros(t_final,1);
total_reward_earned_array = zeros(t_final,1);
cumulative_total_reward_available_array = zeros(t_final,1);
cumulative_total_reward_earned_array = zeros(t_final,1);
fraction_total_reward_earned_array = zeros(t_final,1);
fraction_cumulative_total_reward_earned_array = zeros(t_final,1);

%% Run simple algorithm

color_array = ['y' 'g' 'b' 'c' 'm'];

for t=1:1:t_final
    disp(t)
    
    % Update Reward of All Grid Points
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
            
            if rand < probability_create_intruder % threshold
                % Intiate intruder
                
                flag_intruder_active(ni,1) = 1;
                
                flag_intruder_type(ni,1) = 1+round(rand);
                
                chosen_side = ceil(4*rand);
                
                switch chosen_side
                    
                    case 1
                        pos_intruder(ni,:) = [rand*grid_num_y 0.1];
                        vel_intruder(ni,:) = [rand-0.5 rand];
                        
                    case 2
                        pos_intruder(ni,:) = [grid_num_y-0.1 rand*grid_num_x];
                        vel_intruder(ni,:) = [-rand rand-0.5];
                        
                    case 3
                        pos_intruder(ni,:) = [rand*grid_num_y grid_num_x-0.1];
                        vel_intruder(ni,:) = [rand-0.5 -rand];
                        
                    case 4
                        pos_intruder(ni,:) = [0.1 rand*grid_num_x];
                        vel_intruder(ni,:) = [rand rand-0.5];
                        
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
    
    
    
    
    % Selection of Next Location
    new_pos_agent = zeros(num_agents,1);
    
    switch flag_type_task_allocation
        
        case 1 % dumb hurestic
            
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
            
        case 2 % Sec.4 Task Assignment for Homogeneous Agents to Tasks at One Time Step
            
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
            
        case 3 % Sec.5.A Task Assignment for Homogeneous Agents to Tasks over Multiple Time Step
            
            n_variables = grid_num_x*grid_num_y;
            
            floating_task_grid_reward = zeros(n_variables,time_receding_horizon);
            fixed_task_grid_reward = zeros(n_variables,time_receding_horizon);
            
            for k=1:1:time_receding_horizon
                floating_task_grid_reward(:,k) = patrol_grid_reward + (k-1)*reward_patrol_increment*ones(n_variables,1) ; % at that time step
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
                            fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
                        end
                    end
                    
                    for g_x=2:1:grid_num_y-1
                        for g_y=[1 grid_num_x]
                            this_bin = (g_y-1)*grid_num_y + g_x;
                            fixed_task_grid_reward(this_bin,k) = fixed_task_grid_reward(this_bin,k) + inactive_intruder*reward_checking_intruder/(2*grid_num_x + 2*grid_num_y - 4);
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



