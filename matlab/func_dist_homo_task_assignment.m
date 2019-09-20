function [homo_reward, homo_x_sol, homo_z_sol, epsilon_q] = func_dist_homo_task_assignment(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance,lambda,total_num_agents)

total_x_q_tau = total_grid_bins * total_grid_bins * homo_num_agents * (time_horizon+1);

total_z_q_tau = total_grid_bins * homo_num_agents * (time_horizon+2);

lower_bound = zeros(total_x_q_tau + total_z_q_tau,1);
upper_bound = ones(total_x_q_tau + total_z_q_tau,1);

f_cost = zeros(total_x_q_tau+total_z_q_tau,1);

for tau = 1:1:time_horizon+2
    for g=1:1:total_grid_bins
        for i=1:1:homo_num_agents
            
            culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0);
            f_cost(culmulative_variable_number,1) = f_cost(culmulative_variable_number,1) + homo_T_f_tau(g,tau) - lambda(tau,g);
            
        end
    end
end

A_equality_entries = zeros(10,3);
b_equality = zeros(10,1);
count = 0;
count_Aeq = 0;

% Eq (1d)
tau = 1;
for i=1:1:homo_num_agents
    
    this_agent_bin = homo_agent_bin(i,:);
    culmulative_bin = func_culmulative_bin(this_agent_bin(1),this_agent_bin(2),grid_num_x,grid_num_y);
    
    for g=1:1:total_grid_bins
        count = count + 1;
        
        for g2=1:1:total_grid_bins
            if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                count_Aeq = count_Aeq + 1;
                culmulative_variable_number = func_culmulative_variable_number_homo('x',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,g2);
                A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, 1];
            end
        end
        
        if g == culmulative_bin
            b_equality(count,1) = 1;
        else
            b_equality(count,1) = 0;
        end
    end
end

% Eq (1e)
for tau=1:1:time_horizon
    for i=1:1:homo_num_agents
        for g=1:1:total_grid_bins
            count = count + 1;
            b_equality(count,1) = 0;
            
            for g2=1:1:total_grid_bins
                if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                    count_Aeq = count_Aeq + 1;
                    culmulative_variable_number = func_culmulative_variable_number_homo('x',i,g2,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,g);
                    A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, 1];
                end
            end
            
            for g2=1:1:total_grid_bins
                if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                    count_Aeq = count_Aeq + 1;
                    culmulative_variable_number = func_culmulative_variable_number_homo('x',i,g,tau+1,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,g2);
                    A_equality_entries(count_Aeq,:) = [count, culmulative_variable_number, -1];
                end
            end
            
        end
    end
end

A_equality = sparse(A_equality_entries(:,1),A_equality_entries(:,2),A_equality_entries(:,3),count,total_x_q_tau+total_z_q_tau);

A_in_equality_entries = zeros(10,3);
b_in_equality = zeros(10,1);
count = 0;
count_Aineq = 0;


% Eq (1k)
for tau=1:1:time_horizon+1
    for i=1:1:homo_num_agents
        for g=1:1:total_grid_bins
            count = count + 1;
            b_in_equality(count,1) = 0;
            
            count_Aineq = count_Aineq + 1;
            culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0);
            A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
            
            for g2=1:1:total_grid_bins
                if func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                    count_Aineq = count_Aineq + 1;
                    culmulative_variable_number = func_culmulative_variable_number_homo('x',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,g2);
                    A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
                end
            end
            
        end
    end
end

% Eq (1l)
tau = time_horizon+1;
for i=1:1:homo_num_agents
    for g=1:1:total_grid_bins
        count = count + 1;
        b_in_equality(count,1) = 0;
        
        count_Aineq = count_Aineq + 1;
        culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau+1,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0); 
        A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
        
        for g2=1:1:total_grid_bins
            if func_transition_allowed(g2,g,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins) == 1
                count_Aineq = count_Aineq + 1;
                culmulative_variable_number = func_culmulative_variable_number_homo('x',i,g2,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,g); 
                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, -1];
            end
        end
        
    end
end

% Eq (1m)
for tau=1:1:time_horizon+2
        for g=1:1:total_grid_bins
            count = count + 1;
            b_in_equality(count,1) = 1;
            
            for i=1:1:homo_num_agents
                count_Aineq = count_Aineq + 1;
                culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0); 
                A_in_equality_entries(count_Aineq,:) = [count, culmulative_variable_number, 1];
            end
            
        end
end

A_in_equality = sparse(A_in_equality_entries(:,1),A_in_equality_entries(:,2),A_in_equality_entries(:,3),count,total_x_q_tau+total_z_q_tau);

options = optimset('linprog');
options.Display = 'off';
[x_sol , f_val] = linprog(-f_cost,A_in_equality,b_in_equality,A_equality,b_equality,lower_bound,upper_bound,options);

homo_reward = -f_val;

homo_x_sol = x_sol(1:total_x_q_tau,1);

homo_z_sol = x_sol(total_x_q_tau+1:end,1);

epsilon_q = zeros(time_horizon+2,total_grid_bins);
for tau = 1:1:time_horizon+2
    for g=1:1:total_grid_bins
        for i=1:1:homo_num_agents
            
            culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0);
            epsilon_q(tau,g) = x_sol(culmulative_variable_number,1) - (1/total_num_agents);            
        end
    end
end

f_cost = zeros(total_x_q_tau+total_z_q_tau,1);

for tau = 1:1:time_horizon+2
    for g=1:1:total_grid_bins
        for i=1:1:homo_num_agents
            
            culmulative_variable_number = func_culmulative_variable_number_homo('z',i,g,tau,time_horizon,total_grid_bins,homo_num_agents,total_x_q_tau,0);
            f_cost(culmulative_variable_number,1) = f_cost(culmulative_variable_number,1) + homo_T_f_tau(g,tau);
            
        end
    end
end

homo_reward = f_cost' * x_sol;
