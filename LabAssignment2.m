classdef LabAssignment2 < handle
    properties

        Dobot_A2 %Robot class
        workspace = [-5 5 -3 3 -0.5 2];
        pill_height = {};        
        pillMatrix = {};

        end
    
    methods
function obj = LabAssignment2(tablepose,pill_1,pill_2,pill_3,pill_4,pill_5,pill_6,pill_7,pill_8,pill_9, box) %constructor

startpose =transl(0.4,-0.3,0);
tablepose = transl(0,0,0); 


    pill_1 = [0.25;-0.2;0]; % initial position of each bottle
    pill_2 = [0.25;-0.3;0];
    pill_3 = [0.25;-0.4;0];
    pill_4 = [0;-0.2;0];
    pill_5 = [0;-0.3;0];
    pill_6 = [0;-0.4;0];
    pill_7 = [-0.25;-0.2;0];
    pill_8 = [-0.25;-0.3;0]; 
    pill_9 = [-0.25;-0.4;0];
%     box = ;


    obj.Locationforpills(pill_1,pill_2,pill_3,pill_4,pill_5,pill_6,pill_7,pill_8,pill_9); %call placement of pills fuction
    obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction

    disp('Press Enter to Continue');

    pause();
end

function GenerateEnvironment(obj,centre,tablepose)    
    [f,v,data] = plyread('table.ply','tri');
    
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    x_table = tablepose(1,4);
    y_table = tablepose(2,4);
    Offset = centre(3,4) - 0.554; % table is 0.495m tall, and so this is to asjust the offset of the table so that the robot can sit on top of it
    
    trisurf(f,v(:,1) + x_table, v(:,2) + y_table, v(:,3) + Offset,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    hold on
    camlight;
    axis equal;
    view(3);
    
    
    for i = 1:9
        [f,v,data] = plyread('pill_bottle.ply','tri');
        % find corner size (vertex) of the pills
        determinesize = size(v,1);
        %moving the origin of the pill to the top of the table
        Originpoint = (sum(v)/determinesize); 
        pillVertex = v - repmat(Originpoint,determinesize,1);
        
        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        % Then plot the trisurf
        obj.pillMatrix{i} = trisurf(f,pillVertex(:,1) + obj.pill_height{i}(1,4), ...
            pillVertex(:,2) + obj.pill_height{i}(2,4), pillVertex(:,3) + (0.06671/2), ...
            'FaceVertexCData',vertexColours,'EdgeColor', 'interp','EdgeLighting','flat');
    end
    
    
    
    [f,v,data] = plyread('fire_extinguisher.ply','tri');
    % Scale the colours to be 0-to-1 (they are originally 0-to-255)
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    % Then plot the trisurf
    trisurf(f,v(:,1) - 1, v(:,2) - 1, v(:,3) + 0 ,'FaceVertexCData',...
    vertexColours,'EdgeColor','interp','EdgeLighting','flat');

       surf([-2.5,-2.5;2.5,2.5],[-2,2;-2,2],[-0.5,-0.5;-0.5,-0.5],...
       'CData',imread('concrete.jpg'),'FaceColor','texturemap');
       
       surf([0.1,-0.1;0.1,-0.1],[0.1,0.1;0.2,0.2],[0,0;0,0],...
       'CData',imread('pick_up_sign.jpg'),'FaceColor','texturemap');
  
       disp('Generating Dobot');
       dobot = Dobot_A2();                                                       
       dobot.PlotModel3d();
       disp('lifting');
       dobot.lift(true);
       pause(0.5); 
       disp('lowering');
       dobot.lift(false); 
            
end

function Locationforpills(object,pill_1,pill_2,pill_3,pill_4,pill_5,pill_6,pill_7,pill_8,pill_9)
    
    object.pill_height{1} = pill_1; %create instance of each pill as an object
    object.pill_height{2} = pill_2;
    object.pill_height{3} = pill_3;
    object.pill_height{4} = pill_4;
    object.pill_height{5} = pill_5;
    object.pill_height{6} = pill_6;
    object.pill_height{7} = pill_7;
    object.pill_height{8} = pill_8;
    object.pill_height{9} = pill_9;
    
    position = eye(4); %create positions as a 4x4 matrix
    position = position(1:4,1:3); 

    for i= 1:9
        object.pill_height{i} = [[object.pill_height{i}]; 1];
        object.pill_height{i} = [[position] [object.pill_height{i}]];
    end
    clc;
    
end

    end
end