classdef NeedleThruRRT < handle
    properties (SetAccess = private)
        tree                % Array stores position information of states
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        free_nodes          % Indices of free nodes
        free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        XY_BOUNDARY         % [min_x max_x min_y max_y]
        goal_point          % Goal position
%         delta_goal_point    % Radius of goal position region
%         delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        obstacle            % Obstacle information
%         dynamic_obstacle    % Dynamic Obstacles Information
        best_path_node      % The index of last node of the best path
        goal_reached
        max_nodes 
        %%% Binning for faster neighbor search
        % bins are square
        bin_size
        bin
        bin_x
        bin_y
        bin_offset
        nbins
        bin_ind
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
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.XY_BOUNDARY = zeros(4,1);
            this.tree(:, 1) = map.start_point;
            this.goal_point = map.goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_near = conf.delta_near;
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
            
            %% placing nodes in additional bins
            radius = this.delta_near;
            x_left = x_comp;
            x_right = x_comp;
            y_top = y_comp;
            y_bottom = y_comp;
            if map.start_point(1) - radius >= this.XY_BOUNDARY(1)
                x_left = int32((map.start_point(1) - radius)/this.bin_size - 0.5);
            end
            if map.start_point(1) + radius <= this.XY_BOUNDARY(2)
                x_right = int32((map.start_point(1) + radius)/this.bin_size - 0.5);
            end
            if map.start_point(2) - radius >= this.XY_BOUNDARY(3)
                y_top = int32((map.start_point(2) + radius)/this.bin_size - 0.5);
            end
            if map.start_point(2) + radius <= this.XY_BOUNDARY(4)
                y_bottom = int32((map.start_point(2) - radius)/this.bin_size - 0.5);
            end
            
            if x_comp > x_left && cur_bin - 1 > 0
                this.bin(cur_bin-1).last = this.bin(cur_bin-1).last + 1;
                this.bin(cur_bin-1).nodes(this.bin(cur_bin-1).last) = 1;
                
                this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1;
            end
            
            if x_comp < x_right && cur_bin + 1 < this.nbins
                this.bin(cur_bin+1).last = this.bin(cur_bin+1).last + 1;
                this.bin(cur_bin+1).nodes(this.bin(cur_bin+1).last) = 1;
                
                this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1;
            end
            
            if y_comp < y_top
                if cur_bin+this.bin_x <= this.nbins
                    this.bin(cur_bin+this.bin_x).last = this.bin(cur_bin+this.bin_x).last + 1;
                    this.bin(cur_bin+this.bin_x).nodes(this.bin(cur_bin+this.bin_x).last) = 1;
                    
                    this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                    this.bin_ind(this.bin_ind(end,1),1) = cur_bin+this.bin_x;
                    if x_comp > x_left
                        this.bin(cur_bin-1+this.bin_x).last = this.bin(cur_bin-1+this.bin_x).last + 1;
                        this.bin(cur_bin-1+this.bin_x).nodes(this.bin(cur_bin-1+this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1+this.bin_x;
                    end
                    if x_comp < x_right && cur_bin+this.bin_x+1 <= this.nbins
                        this.bin(cur_bin+1+this.bin_x).last = this.bin(cur_bin+1+this.bin_x).last + 1;
                        this.bin(cur_bin+1+this.bin_x).nodes(this.bin(cur_bin+1+this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1+this.bin_x;
                    end
                end
            end
            
            if y_comp > y_bottom
                if cur_bin-this.bin_x > 0
                    this.bin(cur_bin-this.bin_x).last = this.bin(cur_bin-this.bin_x).last + 1;
                    this.bin(cur_bin-this.bin_x).nodes(this.bin(cur_bin-this.bin_x).last) = 1;
                    
                    this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                    this.bin_ind(this.bin_ind(end,1),1) = cur_bin-this.bin_x;
                    
                    if x_comp > x_left && cur_bin-1-this.bin_x > 0
                        this.bin(cur_bin-1-this.bin_x).last = this.bin(cur_bin-1-this.bin_x).last + 1;
                        this.bin(cur_bin-1-this.bin_x).nodes(this.bin(cur_bin-1-this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin-1-this.bin_x;
                    end
                    if x_comp < x_right
                        this.bin(cur_bin+1-this.bin_x).last = this.bin(cur_bin+1-this.bin_x).last + 1;
                        this.bin(cur_bin+1-this.bin_x).nodes(this.bin(cur_bin+1-this.bin_x).last) = 1;
                        
                        this.bin_ind(end,1) = this.bin_ind(end, 1) + 1;
                        this.bin_ind(this.bin_ind(end,1),1) = cur_bin+1-this.bin_x;
                    end
                end
            end
        end
        
        function position = sample(this)
            % generates and return random point in area defined in
            % this.XY_BOUNDARY
            position = [this.XY_BOUNDARY(2) - this.XY_BOUNDARY(1); this.XY_BOUNDARY(4) - this.XY_BOUNDARY(3)] .* rand(2,1) ...
                + [this.XY_BOUNDARY(1);this.XY_BOUNDARY(3)];
        end
    end