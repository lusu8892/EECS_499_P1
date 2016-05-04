classdef NeedleThruRRT < handle
    properties (SetAccess = private)
        tree                % Array stores position information of states
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        % free_nodes          % Indices of free nodes
        % free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        XY_BOUNDARY         % [min_x max_x min_y max_y]
        goal_point          % Goal position
        % delta_goal_point    % Radius of goal position region
        % delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        obstacle            % Obstacle information
        % dynamic_obstacle    % Dynamic Obstacles Information
        best_path_node      % The index of last node of the best path
        goal_reached
        max_nodes 
        %%% Binning for faster neighbor search
        % bins are square
        % bin_size
        % bin
        % bin_x
        % bin_y
        % bin_offset
        % nbins
        % bin_ind
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
    end
    
    methods
        % class constructor
        function this = NeedleThruRRT(rand_seed, max_nodes, map, conf)
            max_nodes = int32(max_nodes);
            this.max_nodes = max_nodes;
            rng(rand_seed);
            this.tree = zeros(2, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes);
            % this.free_nodes = zeros(1, max_nodes);
            % this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XY_BOUNDARY = zeros(4,1);
            this.tree(:, 1) = map.start_point;
            this.goal_point = map.goal_point;
            % this.delta_goal_point = conf.delta_goal_point;
            % this.delta_near = conf.delta_near;
            this.nodes_added = uint32(1);
            this.max_step = conf.max_step;
            this.best_path_node = -1;
            this.goal_reached = false;
            this.load_map(map.name);
            %%% temp var-s initialization
            this.compare_table = zeros(1, max_nodes);
            this.index = zeros(1, max_nodes);
            this.list = 1:max_nodes;
            this.num_rewired = 0;
            
            this.bin_ind = zeros(10, max_nodes);
            
            this.bin_size = conf.bin_size;
            this.bin_x = ceil((this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1))/this.bin_size);
            this.bin_y = ceil((this.XY_BOUNDARY(4) - this.XY_BOUNDARY(2))/this.bin_size);
            
            delta = this.bin_size/100;
            left_edge = int32((this.XY_BOUNDARY(1) + delta) / this.bin_size - 0.5);
            bottom_edge = int32((this.XY_BOUNDARY(3) + delta) / this.bin_size - 0.5);
            right_edge = int32((this.XY_BOUNDARY(2) - delta) / this.bin_size - 0.5);
            top_edge = int32((this.XY_BOUNDARY(4) - delta) / this.bin_size - 0.5);
            
            this.bin_offset = -(left_edge + bottom_edge*this.bin_x) + 1;
            this.nbins = (right_edge + top_edge*this.bin_x) - (left_edge + bottom_edge*this.bin_x)+ 1;
            this.bin = repmat(struct('nodes', zeros(1, int32(max_nodes/5)), 'last', 0), 1, this.nbins);
            
            % add root node into bin
            x_comp = int32(map.start_point(1) / this.bin_size - 0.5);
            y_comp = int32(map.start_point(2) / this.bin_size - 0.5);
            cur_bin = x_comp + y_comp*this.bin_x + this.bin_offset;
            this.bin(cur_bin).last = this.bin(cur_bin).last + 1;
            this.bin(cur_bin).nodes(this.bin(cur_bin).last) = 1;
            this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
            this.bin_ind(this.bin_ind(end,1),1) = cur_bin;
        
        function [ sample_node ] = collisionFreeRandomConfig(this.map_info)
            sample_node.position = [];
            sample_node.direction = 0;

            while (true)
                % based on map info generate random sample configuration and check collision
                sample_node.position(1,:) = 100 * rand();
                sample_node.position(2,:) = 100 * rand();
                sample_node.position(3,:) = 0;
                sample_node.direction = (2 * pi) * rand();
            %         plot(sample_node.position(1,:),sample_node.position(2,:),'o');
            %         hold on
            %         pause(1);
                
                collision = collisionDetection(map_info, sample_node);
                
                if (collision == true)
                    continue;
                else
                    return;
                end
            end
        end

        function load_map(this, map_name)
            % function loads '.mat' file with obstacle information and the
            % size of the map
            map_path = 'maps/';
            this.obstacle = load([map_path map_name], 'num', 'output', 'x_constraints', 'y_constraints');
            this.obstacle.vert_num = zeros(this.obstacle.num,1);
            this.obstacle.m = cell(this.obstacle.num,1);
            this.obstacle.b = cell(this.obstacle.num,1);
            this.obstacle.r = zeros(this.obstacle.num,1);
            this.obstacle.r_sqr = zeros(this.obstacle.num,1);
            this.obstacle.cir_center = cell(this.obstacle.num,1);
            
            
            this.XY_BOUNDARY = [this.obstacle.x_constraints this.obstacle.y_constraints];
            for obs_ind = 1:this.obstacle.num
                this.obstacle.m{obs_ind} = (this.obstacle.output{obs_ind}(1:end-1,2) - this.obstacle.output{obs_ind}(2:end,2)) ./ (this.obstacle.output{obs_ind}(1:end-1,1) - this.obstacle.output{obs_ind}(2:end,1));
                this.obstacle.b{obs_ind} = this.obstacle.output{obs_ind}(1:end-1,2) - this.obstacle.m{obs_ind} .* this.obstacle.output{obs_ind}(1:end-1,1);
                this.obstacle.vert_num(obs_ind) = size(this.obstacle.output{obs_ind}, 1)-1;
                
                this.obstacle.cir_center{obs_ind} = zeros(2, 1);
                [this.obstacle.cir_center{obs_ind}(1), this.obstacle.cir_center{obs_ind}(2), this.obstacle.r(obs_ind)] = ...
                    SmallestEnclosingCircle(this.obstacle.output{obs_ind}(1:end-1,1)', this.obstacle.output{obs_ind}(1:end-1,2)');
                this.obstacle.r_sqr(obs_ind) = this.obstacle.r(obs_ind) ^ 2;
            end
        end


    end