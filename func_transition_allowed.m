function transition_allowed = func_transition_allowed(g,g2,grid_num_x,grid_num_y,threshold_max_distance,total_grid_bins)

if (g >= 1) && (g <= total_grid_bins) && (g2 >= 1) && (g2 <= total_grid_bins)
    
    [g_x, g_y] = func_bin_from_culmulative_bin(g,grid_num_x,grid_num_y,total_grid_bins);
    
    [g2_x, g2_y] = func_bin_from_culmulative_bin(g2,grid_num_x,grid_num_y,total_grid_bins);
    
    if norm([g_x, g_y] - [g2_x, g2_y],1) <= threshold_max_distance
        transition_allowed = 1;
    else
        transition_allowed = 0;
    end
    
else
    transition_allowed = 0;
    disp('Error in transition_allowed!')
end