function [culmulative_bin_x, culmulative_bin_y] = func_bin_from_culmulative_bin(culmulative_bin,grid_num_x,grid_num_y,total_grid_bins)

if (culmulative_bin >= 1) && (culmulative_bin <= total_grid_bins) 
    
    culmulative_bin_y = mod(culmulative_bin, grid_num_y);
    culmulative_bin_x = (culmulative_bin-culmulative_bin_y)/grid_num_y + 1;
    
    if culmulative_bin_y == 0
        culmulative_bin_y = grid_num_y;
        culmulative_bin_x = culmulative_bin_x - 1;
    end
       
else
    culmulative_bin_x = 0;
    culmulative_bin_y = 0;
    disp('Error in bin_from_culmulative_bin!')
end


% % Try for entire grid
% 
% for i=1:1:grid_num_x
%     for j=1:1:grid_num_y
%         culmulative_bin = func_culmulative_bin(i,j,grid_num_x,grid_num_y);
%         [culmulative_bin_x, culmulative_bin_y] = func_bin_from_culmulative_bin(culmulative_bin,grid_num_x,grid_num_y,total_grid_bins);
%         if (i == culmulative_bin_x) && (j == culmulative_bin_y)
%             % Do nothing
%         else
%             disp('Error!')
%         end
%     end
% end