function culmulative_variable_number = func_culmulative_variable_number(xyz,j,i,g,tau,time_horizon,total_grid_bins,num_agent_type,num_agents_per_type,total_x_q_tau,total_y_q_tau,g2)

% for tau = 1:1:time_horizon+1
%     for g=1:1:total_grid_bins
%         for i=1:1:num_agent_type
%             for j=1:1:num_agents_per_type(i)

flag_input_values_are_correct = 0;
if (tau >= 1) && (tau <= time_horizon+2)
    if (g >= 1) && (g <= total_grid_bins)
        if (i >= 1) && (i <= num_agent_type)
            if (j >= 1) && (j <= num_agents_per_type(i))
                flag_input_values_are_correct = 1;
                
                if i == 1
                    this_agent_number = j;
                else
                    this_agent_number = sum(num_agents_per_type(1:i-1)) + j;
                end
                
                if (xyz == 'x')
                    if (g2 >= 1) && (g2 <= total_grid_bins) && (tau <= time_horizon+1)
                        flag_input_values_are_correct = 1;
                    else
                        flag_input_values_are_correct = 0;
                    end
                end
                
            end
        end
    end
end

if (xyz == 'x') && (flag_input_values_are_correct == 1)
    
    culmulative_variable_number = (this_agent_number-1)*(time_horizon+1)*total_grid_bins*total_grid_bins + (tau-1)*total_grid_bins*total_grid_bins + (g-1)*total_grid_bins + g2;
    
elseif (xyz == 'y') && (flag_input_values_are_correct == 1)
    
    culmulative_variable_number = total_x_q_tau + (this_agent_number-1)*(time_horizon+2)*total_grid_bins + (tau-1)*total_grid_bins + g;
    
elseif (xyz == 'z') && (flag_input_values_are_correct == 1)

    culmulative_variable_number = total_x_q_tau + total_y_q_tau + (this_agent_number-1)*(time_horizon+2)*total_grid_bins + (tau-1)*total_grid_bins + g;
    
else
    culmulative_variable_number = 0;
    disp('Error in culmulative_variable_number!')
end