function [homo_reward, homo_x_sol, homo_z_sol] = func_distributed_homo_task_assignment_setup(homo_T_f_tau,homo_num_agents,homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance)

lambda = zeros(time_horizon+2,total_grid_bins);
alpha = 0.1;
epsilon_0 = 0.1;
epsilon_f = ones(time_horizon+2,total_grid_bins);
num_old = homo_num_agents;
old_epsilon_f = zeros(time_horizon+2,total_grid_bins,num_old);

count = 0;
fprintf('\n')

flag_convergence_satisfied = 0; 

while flag_convergence_satisfied == 0
    count = count + 1;
    fprintf('|')
    
    flag_matches_num_old = 0;
    for i_no=1:1:num_old
        flag_matches_num_old = flag_matches_num_old + isequal(old_epsilon_f(:,:,i_no),epsilon_f);
    end
    
    if (flag_matches_num_old > 0) && (count > num_old) 
        fprintf(['o',num2str(flag_matches_num_old)])
        if (flag_matches_num_old <= num_old/2)
            alpha = 0.9*alpha;
        else
            alpha = 1.0*alpha;
        end
    else
        alpha = 0.999*alpha;
    end
    
    %     alpha = 0.99*alpha;
    
    array_total_x_q_tau = total_grid_bins * total_grid_bins * 1 * (time_horizon+1);
    array_total_z_q_tau = total_grid_bins * 1 * (time_horizon+2);

    
    epsilon_q_array = zeros(time_horizon+2,total_grid_bins,homo_num_agents);
    homo_reward_array = zeros(homo_num_agents,1);
    homo_x_sol_array = zeros(array_total_x_q_tau,homo_num_agents);
    homo_z_sol_array = zeros(array_total_z_q_tau,homo_num_agents);
    
    parfor j=1:1:homo_num_agents
        
        this_homo_T_f_tau = homo_T_f_tau;
        this_homo_num_agents = 1;
        this_homo_agent_bin = homo_agent_bin(j,:);
        
        [this_homo_reward, this_homo_x_sol, this_homo_z_sol, epsilon_q] = func_dist_homo_task_assignment(this_homo_T_f_tau,this_homo_num_agents,this_homo_agent_bin,total_grid_bins,time_horizon,grid_num_x,grid_num_y,threshold_max_distance,lambda,homo_num_agents);
        
        epsilon_q_array(:,:,j) = epsilon_q;
        
        homo_reward_array(j,1) = this_homo_reward;
        
        homo_x_sol_array(:,j) = this_homo_x_sol;
        
        homo_z_sol_array(:,j) = this_homo_z_sol;
    end
    
    for i_no=num_old:-1:2
        old_epsilon_f(:,:,i_no) = old_epsilon_f(:,:,i_no-1);
    end
    old_epsilon_f(:,:,1) = epsilon_f;
    
    %     old_old_epsilon_f = old_epsilon_f;
    %     old_epsilon_f = epsilon_f;
    
    homo_reward = sum(homo_reward_array);
    homo_x_sol = zeros(array_total_x_q_tau*homo_num_agents,1);
    homo_z_sol = zeros(array_total_z_q_tau*homo_num_agents,1);
    epsilon_f = zeros(time_horizon+2,total_grid_bins);
    
    for j=1:1:homo_num_agents
        epsilon_f = epsilon_f + epsilon_q_array(:,:,j);
        homo_x_sol((j-1)*array_total_x_q_tau+1:j*array_total_x_q_tau,1) = homo_x_sol_array(:,j);
        homo_z_sol((j-1)*array_total_z_q_tau+1:j*array_total_z_q_tau,1) = homo_z_sol_array(:,j);
    end
    
    
    lambda = subplus(lambda + alpha*epsilon_f);
    
    
    %     lambda
    %
    %     epsilon_f
    %
    %     disp(max(max(epsilon_f)))
    %
    %     pause
    %
    
    if max(max(epsilon_f)) < epsilon_0
        flag_convergence_satisfied = 1;
    end
    
    if (mod(count,50) == 0) || (flag_convergence_satisfied > 0)
        fprintf([' : ',num2str(count),', alpha = ',num2str(alpha),', max epsilon_f = ',num2str(max(max(epsilon_f))),', num epsilon_f >= 1 is ',num2str(sum(sum(epsilon_f>=1))),'\n'])
    end
    
    % added for breaking no conv solutions
    if count == 3000
        homo_reward = 0;
        break
    end
    
end


