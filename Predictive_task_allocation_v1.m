% Predictive Task Allocation
close all, clear all, clc

%% Initialize by User
grid_num_x = 8;
grid_num_y = 10;

num_agents = 3;
flag_starting_condition = 2; % Set 1 for starting in a row, 2 for random

max_num_intruder = 3;

flag_patrol_intruder = 1; % Set 0 for only patrol, set 1 for tracking intruder

flag_max_distance_constraint = 1; % Set 0 for no constraint, set 1 to use threshold_max_distance as constraint

threshold_max_distance = 2;

flag_store_video = 0;

%% Initialize Variables
standard_font_size = 25;
title_font_size = 40;

patrol_grid_cost = zeros(grid_num_x*grid_num_y,1);

patrol_grid_location = zeros(grid_num_x*grid_num_y,2);
for i=1:1:grid_num_x
    for j=1:1:grid_num_y
        patrol_grid_location((i-1)*grid_num_y+j,:) = [j-0.5, i-0.5];
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

%% Run simple algorithm

color_array = ['y' 'g' 'b' 'c' 'm'];

for t=0:1:100
    disp(t)
    
    % Remove Cost from Observed Grid Points
    for n=1:1:num_agents
        patrol_grid_cost(pos_agent(n),1) = 0;
    end
    
    % Update Cost of All Grid Points
    patrol_grid_cost = patrol_grid_cost + ones(grid_num_x*grid_num_y,1);
    
    % Update Intruder Motion
    for ni=1:1:max_num_intruder
        if flag_intruder_active(ni,1) == 0
            % Intruder is inactive
            
            if rand < 0.1 % threshold
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
                this_j = ceil(pos_intruder(ni,1));
                this_i = ceil(pos_intruder(ni,2));
                this_bin = (this_i-1)*grid_num_y + this_j;
                bin_intruder(ni,1) = this_bin;
            else
                flag_intruder_active(ni,1) = 0;
                bin_intruder(ni,1) = 0;
            end
        end
        
    end
    
    % Go to Next Location
    new_pos_agent = zeros(num_agents,1);
    
    all_cost = zeros(2,4); %[cost patrol/intruder bin agent]
    
    for n=1:1:num_agents
        
        this_agent_pos = pos_agent(n,1);
        this_j = mod(this_agent_pos-1,grid_num_y) + 1;
        this_i = ((this_agent_pos - this_j)/grid_num_y) + 1;
        this_agent_location = [this_j-0.5, this_i-0.5];
        
        this_agent_distance = vecnorm(patrol_grid_location - this_agent_location,2,2);
        
        this_agent_pgc = patrol_grid_cost - weight_distance*this_agent_distance;
        
        this_agent_patrol_cost = [this_agent_pgc, zeros(grid_num_x*grid_num_y,1), [1:1:grid_num_x*grid_num_y]', n*ones(grid_num_x*grid_num_y,1)];
        
        greater_than_max_distance = logical(this_agent_distance > threshold_max_distance);
        
        this_agent_patrol_cost = this_agent_patrol_cost(~greater_than_max_distance,:);
        
        all_cost = [all_cost; this_agent_patrol_cost];
    end
    
    if flag_patrol_intruder == 1
        
        for ni=1:1:max_num_intruder
            if flag_intruder_active(ni,1) == 1
                if agent_check_intruder_type(ni,1) == 0 || agent_check_intruder_type(ni,1) == 2
                    
                    for n=1:1:num_agents
                        
                        this_agent_pos = pos_agent(n,1);
                        this_j = mod(this_agent_pos-1,grid_num_y) + 1;
                        this_i = ((this_agent_pos - this_j)/grid_num_y) + 1;
                        this_agent_location = [this_j-0.5, this_i-0.5];
                        
                        this_agent_distance_intruder = norm(this_agent_location - pos_intruder(ni,:));
                        
                        if this_agent_distance_intruder < threshold_max_distance
                            
                            this_agent_intruder_cost = 100 - weight_distance*this_agent_distance_intruder;
                            
                            this_agent_intruder_cost_array = [this_agent_intruder_cost, ni, bin_intruder(ni,1), n];
                            
                            all_cost = [all_cost; this_agent_intruder_cost_array];
                            
                        end
                    end
                    
                end
            else
                agent_check_intruder_type(ni,1) = 0;
            end
        end
    end
    
    [B_ac,I_ac] = sort(all_cost(:,1),'descend');
    all_cost = all_cost(I_ac,:);
    
    count_assigned_agent = 0;
    
    while count_assigned_agent < num_agents
        this_agent = all_cost(1,4);
        this_bin = all_cost(1,3);
        new_pos_agent(this_agent,1) = this_bin;
        
        if all_cost(1,2) > 0
            this_intruder = all_cost(1,2);
            agent_check_intruder_type(this_intruder,1) = flag_intruder_type(this_intruder,1);
            
            this_intruder_appears = logical(all_cost(:,2)==this_intruder);
            all_cost = all_cost(~this_intruder_appears,:);
        end
        
        count_assigned_agent = count_assigned_agent + 1;
        this_agent_appears = logical(all_cost(:,4)==this_agent);
        all_cost = all_cost(~this_agent_appears,:);
        
        this_bin_appears = logical(all_cost(:,3)==this_bin);
        all_cost = all_cost(~this_bin_appears,:);
    end
    
    
    % Plotting
    
    h4=figure(1);
    clf
    set(h4,'Color',[1 1 1]);
    set(h4,'units','normalized','outerposition',[0 0 1 1])
    set(h4,'PaperPositionMode','auto');
    hold on
    
    for gi = 0:1:grid_num_x
        plot([0 grid_num_y],[gi gi],'-k','LineWidth',1.0)
    end
    for gj = 0:1:grid_num_y
        plot([gj gj],[0 grid_num_x],'-k','LineWidth',1.0)
    end
    for i=1:1:grid_num_x
        for j=1:1:grid_num_y
            plot(j-0.5,i-0.5,'or','MarkerSize',patrol_grid_cost((i-1)*grid_num_y+j,1))
        end
    end
    
    for n=1:1:num_agents
        this_agent_pos = pos_agent(n,1);
        this_j = mod(this_agent_pos-1,grid_num_y) + 1;
        this_i = ((this_agent_pos - this_j)/grid_num_y) + 1;
        plot(this_j-0.5, this_i-0.5, 'sk','MarkerSize',20,'MarkerFaceColor',color_array(mod(n,length(color_array))+1))
        
        new_agent_pos = new_pos_agent(n,1);
        new_j = mod(new_agent_pos-1,grid_num_y) + 1;
        new_i = ((new_agent_pos - new_j)/grid_num_y) + 1;
        plot([this_j-0.5 new_j-0.5], [this_i-0.5 new_i-0.5], '-','LineWidth',2.0,'Color',color_array(mod(n,length(color_array))+1))
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



