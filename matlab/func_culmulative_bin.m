function culmulative_bin = func_culmulative_bin(i,j,grid_num_x,grid_num_y)

if (i >= 1) && (i<=grid_num_x) && (j>=1) && (j<=grid_num_y)
    culmulative_bin = (i-1)*grid_num_y + j;
else
    culmulative_bin = 0;
    % disp('Error in culmulative_bin!') % I use this zero for other things!
end