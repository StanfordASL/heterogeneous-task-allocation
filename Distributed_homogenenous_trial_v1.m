% Trial to find error in distributed homogenenous

num_trial_runs = 50;

no_convergence_array = [];
count_no_convergence = 0;

actual_error_array = [];
count_actual_error = 0;

for n_tr = 1:1:num_trial_runs
    
    disp(['n_tr = ',num2str(n_tr)])
    
    %% Initialize by User
    
    % Grid Size
    grid_num_x = 8;
    grid_num_y = 10;
    
    % Number of Heterogeneous Agent Types
    num_agent_type = 2;
    
    % Number of Agents per Type
    num_agents_per_type = zeros(num_agent_type,1);
    for i=1:1:num_agent_type
        num_agents_per_type(i) = 2;
    end
    %     num_agents_per_type(end) = 3;
    
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
    
    t=1;
    
    %     disp(t)
    
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
        
        %                 if flag_type_task_allocation == 4
        [homo_reward_case4, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
        homo_reward_case4
        
        %                 elseif flag_type_task_allocation == 5
        [homo_reward_case5, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
        homo_reward_case5
        
        %                 else
        %                     disp('Shouldnt reach here!')
        %
        %                 end
        
        %                 PF_x_sol = [PF_x_sol; homo_x_sol];
        %                 PF_reward = PF_reward + homo_reward;
        
        if abs(homo_reward_case4-homo_reward_case5) > 1e-6
            disp('Error found between case4 and case 5!')
            
            data = [];
            data.homo_T_f_tau = homo_T_f_tau;
            data.homo_num_agents = homo_num_agents;
            data.homo_agent_bin = homo_agent_bin;
            data.homo_reward_case4_stored = homo_reward_case4;
            data.homo_reward_case5_stored = homo_reward_case5;
            
            if homo_reward_case5 == 0
                count_no_convergence = count_no_convergence + 1;
                no_convergence_array{count_no_convergence} = data;
            else
                count_actual_error = count_actual_error + 1;
                actual_error_array{count_actual_error} = data;
            end
            
        end
        
    end
    
    
    
end

save('find_error_dist_homo.mat')

%% Load and recheck those with actual error

load('find_error_dist_homo_16thSept.mat')

disp(count_actual_error)

counter = 0;

counter_case5_smaller = 0;

for i_c = 1:1:count_actual_error
    disp(i_c)
    sol_val = i_c; %ceil(rand*count_actual_error)
    
    data = actual_error_array{sol_val};
    homo_T_f_tau = data.homo_T_f_tau;
    homo_num_agents = data.homo_num_agents;
    homo_agent_bin = data.homo_agent_bin;
    
    %                 if flag_type_task_allocation == 4
    [homo_reward_case4, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
    homo_reward_case4
    
    %                 elseif flag_type_task_allocation == 5
    [homo_reward_case5, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
    homo_reward_case5
    
    if abs(homo_reward_case4-homo_reward_case5) > 1e-6
        disp('Error found between case4 and case 5!')
        counter = counter + 1;
        
        if homo_reward_case5 < homo_reward_case4
            counter_case5_smaller = counter_case5_smaller + 1;
        end
    end
    
end

disp(counter)


%% Load and recheck those that did not converge

load('find_error_dist_homo_16thSept.mat')

disp(count_no_convergence)

counter = 0;

counter_case5_smaller = 0;

for i_c = 1:1:count_no_convergence
    disp(i_c)
    sol_val = i_c; %ceil(rand*count_actual_error)
    
    data = no_convergence_array{sol_val};
    homo_T_f_tau = data.homo_T_f_tau;
    homo_num_agents = data.homo_num_agents;
    homo_agent_bin = data.homo_agent_bin;
    
    %                 if flag_type_task_allocation == 4
    [homo_reward_case4, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
    homo_reward_case4
    
    %                 elseif flag_type_task_allocation == 5
    [homo_reward_case5, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
    homo_reward_case5
    
    if abs(homo_reward_case4-homo_reward_case5) > 1e-6
        disp('Error found between case4 and case 5!')
        counter = counter + 1;
        
        if homo_reward_case5 < homo_reward_case4
            counter_case5_smaller = counter_case5_smaller + 1;
        end
    end
    
end

disp(counter)

%% Load and recheck randomly

load('find_error_dist_homo_16thSept.mat')

sol_val = ceil(rand*count_actual_error)

data = actual_error_array{sol_val};
homo_T_f_tau = data.homo_T_f_tau;
homo_num_agents = data.homo_num_agents;
homo_agent_bin = data.homo_agent_bin;

%                 if flag_type_task_allocation == 4
[homo_reward_case4, homo_x_sol, homo_z_sol] = func_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
homo_reward_case4

%                 elseif flag_type_task_allocation == 5
[homo_reward_case5, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance);
homo_reward_case5

if abs(homo_reward_case4-homo_reward_case5) > 1e-6
    disp('Error found between case4 and case 5!')
end

