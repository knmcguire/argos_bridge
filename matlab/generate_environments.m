%% Random indoor environment generator
% This matlab script makes random environments within a given boundaries of
% an environment. The corridors are generated by virtual agents, started from
% each location that the robot in the final environment will start from. This
% script ensures that the generated corridors are always connected to
% each other. After the corridor generation, doors and rooms are created at
% random as well.
%%%%%%%%%%%
% Authored by Kimberly McGuire (k.n.mcguire@tudelft.nl)
%%%%%%%%%%%%


close all
clc
clear all

%Defined variables
width_environment = 10;
height_environment = 10;
resolution=2;
starting_location_agents = [1,1;10,10];
chance_agent_gostraight = 0.80;
density_corridor = 0.5;
density_openings = 0.5;
size_opening = 15;
opening_percentage = 0.04;
size_rooms = 2;
iterations = 100;
visualize_agents = true;
create_rooms = true;
show_end_result = true;
make_ARGoS_environment = true;

% Initializing environment grid
grid = cell(width_environment,height_environment);
grid{width_environment,height_environment} = [];
for k = 1:width_environment
    for j = 1:height_environment
        grid{k,j}.agent = 0;
        grid{k,j}.circle_action = zeros(2,4);
        grid{k,j}.corridor = 0;
    end
end

% Initialize starting position per agent
for k=1:size(starting_location_agents,2)
    grid{starting_location_agents(k,1),starting_location_agents(k,2)}.agent = 1;
    grid{starting_location_agents(k,1),starting_location_agents(k,2)}.circle_action = circshift([0 1; 1 0; 0 -1; -1 0],[randi([0,4]) 0]);
end

% Initialize variables for loop
random_number = 0;
next_action = "go_straight";

%For It iterations
for it = 1:iterations
    
    %Find current location of agent(s) in grid
    convertgrid= cellfun( @(sas) sas.agent, grid, 'uni', 0 );
    isyes = cellfun(@(sas)isequal(sas,1),convertgrid);
    [rowagent,colagent] = find(isyes);
    
    % For il agents found.
    for il=1:length(rowagent)
        
        %Randomize action selector for the agent
        %pick random number
        random_number = rand;
        %  percent of the time just go straight
        if random_number<=chance_agent_gostraight
            next_action = "go_straight";
        else % Do something else
            precentage_rest = 1-chance_agent_gostraight;
            if random_number>chance_agent_gostraight&&random_number<=chance_agent_gostraight + precentage_rest/2
                next_action = "go_left";
            end
            if random_number>chance_agent_gostraight + precentage_rest/2&&random_number<=1
                next_action = "go_right";
            end
        end
        
        % Per state, find out the direction the agent want to go next with
        % a circle_action. This ensures that the agent only has choice
        % between up left down right (and their immediate neighbors)
        switch next_action
            case "go_straight"
                %Do nothing
            case "go_left"
                grid{rowagent(il),colagent(il)}.circle_action= circshift(grid{rowagent(il),colagent(il)}.circle_action,[-1 0]);
            case "go_right"
                grid{rowagent(il),colagent(il)}.circle_action= circshift(grid{rowagent(il),colagent(il)}.circle_action,[1 0]);
        end
        
        % Calculate next direction and location of the agent
        next_direction = grid{rowagent(il),colagent(il)}.circle_action(1,:);
        next_location_agent = [rowagent(il)+next_direction(1),colagent(il)+next_direction(2)];
        
        %If next location is outside of environment, relocate the agent to
        %the otherside of the environment (like snake 2!)
        next_location_agent = [1 + mod(next_location_agent(1)-1,width_environment), ...
            1 + mod(next_location_agent(2)-1,height_environment)];
        
        %Place the agent at the next location and leave a corridor trace
        grid{next_location_agent(1),next_location_agent(2)}.agent = 1;
        grid{next_location_agent(1),next_location_agent(2)}.circle_action = grid{rowagent(il),colagent(il)}.circle_action ;
        grid{rowagent(il),colagent(il)}.agent = 0;
        grid{rowagent(il),colagent(il)}.corridor = 1;
    end
    
    %Obtain the resulting corridor from this iteration and check if it
    %surpasses the given density value
    result_corridor= cell2mat(cellfun( @(sas) sas.corridor, grid, 'uni', 0 ));
    if sum(sum(result_corridor))/(width_environment*height_environment) > density_corridor
        break
    end
    % Visualize the agents making the environment
    if visualize_agents
        figure(1),
        result_current_location_agent= cell2mat(cellfun( @(sas) sas.agent, grid, 'uni', 0 ));
        rgb=zeros(width_environment,height_environment,3);
        rgb(:,:,1)=result_corridor;
        rgb(:,:,2)=result_current_location_agent;
        imshow(rgb,[],'InitialMagnification',1000)
    end
end


%% Check connectivity of walls between agents
close all

%Find the clusters of corridors
corridor_bin=imbinarize(result_corridor);
connected_corridors= bwconncomp(corridor_bin,4);

% Go through each cluster and see if the agents position is all in one of them
found_connection=false;
for k = 1:connected_corridors.NumObjects
    [y,x] = ind2sub(size(corridor_bin), connected_corridors.PixelIdxList{k});
    %See per initial position, if the agents sit in the same cluster
    for m = 1:size(starting_location_agents,2)
        clear check_interconnection
        check_interconnection=intersect([y,x],starting_location_agents(m,:),'rows');
        %if just one is empty, immediatly break the loop!
        if isempty(check_interconnection)
            break
        else
            %Do not do anything and continue loop
        end
    end
    %if last check_interconnection is not empty, means it found all agentns
    %initial positions in the one cluster
    if ~isempty(check_interconnection)
        found_connection=true;
        break
    end
end

% Give error messages if the initial position of the agents are not within
% the cluster
if(found_connection==false)
    error("maze is not interconnected!! Run the script again!!")
end


%% Create boundary walls

% Increase binary image for more resolution to create walls
corridor_bin_resize=imresize(corridor_bin,10,'nearest');

% Find bounderies at the clusters of corridors
[boundery_coord,L] = bwboundaries(corridor_bin_resize,4);

%Create binary image for just the walls
boundary_bin=zeros(width_environment*10,height_environment*10);
for k=1:length(boundery_coord)
    boundary = boundery_coord{k};
    for l = 1:size(boundery_coord{k},1)
        boundary_bin(boundery_coord{k}(l,1),boundery_coord{k}(l,2))=1;
    end
end

if create_rooms
    for k=1:size_rooms*10:width_environment*10
        for m=1:height_environment*10
            if (corridor_bin_resize(k,m)==0)
                boundary_bin(k,m)=1;
            end
        end
    end
    for m=1:size_rooms*10:width_environment*10
        for k=1:height_environment*10
            if (corridor_bin_resize(k,m)==0)
                boundary_bin(k,m)=1;
            end
        end
    end
end

boundaries_with_holes = boundary_bin;
for k=1:length(boundery_coord)
    boundary = boundery_coord{k};
    for l = 1:size(boundery_coord{k},1)
        random_number = rand;
        if(random_number<opening_percentage)
            boundaries_with_holes=rgb2gray(insertShape(boundaries_with_holes,'FilledRectangle',[boundery_coord{k}(l,1)-size_opening/2,boundery_coord{k}(l,2)-size_opening/2,size_opening,size_opening],...
                'Color','black','Opacity',1.0));
        end
    end
end

%Create opening like doors, by adding noise to the corridor bin and remove
%it from the created walls
% noise_bin=imbinarize(imnoise(zeros(width_environment,height_environment),'salt & pepper',density_openings));
% noise_bin=imresize(noise_bin,10,'nearest');
% boundaries_with_holes = boundary_bin-noise_bin;
boundery_coord_new=[];
[boundery_coord_new(:,2), boundery_coord_new(:,1)] = find(boundaries_with_holes==1);

if show_end_result

    figure,imshow(boundaries_with_holes)
end

imwrite(boundaries_with_holes,'rand_env_test.png')

%%  Create Argos environment
% Makes an ARGoS environment based on the random agent-based environment
% builder, by making each box a pixel in the following form:

%  <box id="wall_west" size="0.1,20,0.5" movable="false">
%       <body position="x_pos_argos,y_pos_argos,0" orientation="0,0,0"/>
% </box>
% TO DO: put robots on postions
%
%
if make_ARGoS_environment
    
    
    % Start of environment, based on height and width of the generated one
    str_start_arena = sprintf('<arena size="%d, %d, 2" center="0,0,0.5">',width_environment*resolution,height_environment*resolution);
    str_start_arena= strcat(str_start_arena, ...
        sprintf('\n<box id="wall_1" size="%d,0.1,0.5" movable="false"> \n <body position="0,%d,0" orientation="0,0,0"/>\n</box>' ...
        ,width_environment*resolution,width_environment/2*resolution));
    str_start_arena= strcat(str_start_arena, ...
        sprintf('\n<box id="wall_2" size="%d,0.1,0.5" movable="false"> \n <body position="0,%d,0" orientation="0,0,0"/>\n</box>' ...
        ,width_environment*resolution,-width_environment/2*resolution));
    str_start_arena= strcat(str_start_arena, ...
        sprintf('\n<box id="wall_3" size="0.1,%d,0.5" movable="false"> \n <body position="%d,0,0" orientation="0,0,0"/>\n</box>' ...
        ,width_environment*resolution,width_environment/2*resolution));
    str_start_arena= strcat(str_start_arena, ...
        sprintf('\n<box id="wall_4" size="0.1,%d,0.5" movable="false"> \n <body position="%d,0,0" orientation="0,0,0"/>\n</box>' ...
        ,width_environment*resolution,-width_environment/2*resolution));
    
    str_loc_bot = "";
    rotation=[-270, -90];
    for k = 1:size(starting_location_agents,1)
        str_loc_bot = strcat(str_loc_bot,sprintf('\n<foot-bot id="bot%d">',k-1));
        str_loc_bot = strcat(str_loc_bot,sprintf('\n<body position="%d,%d,0" orientation="%d,0,0" />',...
            starting_location_agents(k,1)*resolution - 1 - width_environment/2*resolution,...
            starting_location_agents(k,2)*resolution -1- height_environment/2*resolution,...
            rotation(k)));
        str_loc_bot = strcat(str_loc_bot,sprintf('\n<controller config="argos_ros_bot"/>\n</foot-bot>'));
    end

    
    
    x_pos_argos = size(boundery_coord_new,1);
    y_pos_argos = size(boundery_coord_new,1);
    id = zeros(size(boundery_coord_new,1),1);
    
    
    % Transform each boundary coordinate to a coordinate in ARGoS
    h=1;
    for k=1:size(boundery_coord_new,1)
        id(h) = h;
        y_pos_argos(h)=boundery_coord_new(k,1) / 10*resolution - width_environment/2*resolution;
        x_pos_argos(h)=boundery_coord_new(k,2) / 10*resolution - height_environment/2*resolution;
        h=h+1;
    end
    list= zeros(length(id),3);
    list(:,1)=id;
    list(:,2)=x_pos_argos;
    list(:,3)=y_pos_argos;
    
    % Put a box as a pixel on where the boundaries are generated
    formatspec = '\n<box id="%d" size="0.2,0.2,0.5" movable="false"> <body position="%4.2f,%4.2f,0" orientation="0,0,0"/> </box>';
    str_random_env = sprintf(formatspec,list');
    
    %specify end of environment area
    str_end_arena = sprintf('\n</arena>');
    
    %Concatenate the strings together
    str_total = strcat(str_start_arena,str_loc_bot, str_random_env, str_end_arena);
    
    % Read out the template file, use it to find the trigger string (which is
    % <arena></arena> in here). Then replace it with str_total with the random
    % generated environment
    fid = fopen(fullfile('..', 'argos_worlds', 'rand_env_template.argos'),'rt');
    f=fread(fid);
    f=char(f.');
    f = strrep(f,'<arena></arena>',str_total);
    fid  = fopen(fullfile('..', 'argos_worlds', 'rand_env_test.argos'),'wt');
    fwrite(fid,f) ;
    fclose(fid);
end
%And you are finished :):):)
disp('Environment generated and saved in ../argos_world/rand_env_test.argos !!!')
