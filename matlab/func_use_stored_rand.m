function this_rand = func_use_stored_rand(flag_use_stored_random_numbers)

persistent count_rand stored_rand_array

if isempty(count_rand)
    count_rand = 0;
    load('stored_rand_array.mat');
end

if flag_use_stored_random_numbers == 1
    count_rand = count_rand + 1;
    this_rand = stored_rand_array(count_rand,1);
    %     disp(count_rand)
else
    this_rand = rand;
end

end

% stored_rand_array = rand(100000,1);
% save('stored_rand_array.mat','stored_rand_array')
