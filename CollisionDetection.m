
classdef CollisionDetection < handle
    properties

        Dobot_A2 %Robot class
        workspace = [-2 2 -2 2 -0.555 3];
        pill_height = {};
        pillMatrix = {};
    end

    methods
        function obj = CollisionDetection() %constructor

            rad = (pi/3);
            startpose = transl(0.4,-0.3,0);
            tablepose = transl(0,0,0);
            obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction
            obj.detection(rad);
            disp("END");
        end

        function GenerateEnvironment(obj,centre,tablepose)

            [f,v,data] = plyread('table.ply','tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            x_table = tablepose(1,4);
            y_table = tablepose(2,4);
            Offset = centre(3,4) - 0.554; % table is 0.554m tall, and so this is to asjust the offset of the table so that the robot can sit on top of it

            trisurf(f,v(:,1) + x_table, v(:,2) + y_table, v(:,3) + Offset,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            camlight;
            axis equal;
            view(3);


            [f,v,data] = plyread('fire_extinguisher.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            trisurf(f,v(:,1) - 1, v(:,2) - 0.6, v(:,3) - 0.1 ,'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            [f,v,data] = plyread('estop.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            trisurf(f,v(:,1) - 0.52, v(:,2) + 0.54, v(:,3) -0.09,'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            [f,v,data] = plyread('lightcurtain.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            trisurf(f,v(:,1) - 0.55, v(:,2) + 0.32, v(:,3),'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            [f,v,data] = plyread('lightcurtain.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            trisurf(f,v(:,1) + 0.52, v(:,2) + 0.32, v(:,3) ,'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            [f,v,data] = plyread('railing.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255)
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            trisurf(f,v(:,1) - 0.6, v(:,2) +5.25, v(:,3) -0.33,'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            surf([-1.5,-1.5;1.5,1.5],[-1,1;-1,1],[-0.55,-0.55;-0.555,-0.55],...
                'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            surf([0.3,0.1;0.3,0.1],[0.526,0.526;0.526,0.526],[0,0;-0.1,-0.1],...
                'CData',imread('pick_up_sign.jpg'),'FaceColor','texturemap');

            camlight;
        end

        function detection(obj,rad)

            dobot = Dobot_A2;
            %dobot.model.base = trans(0,0,0.4)
            qhome = [0 0.7854 0.7854 0 0];

           Green1 = GreenPills(transl(0.2,-0.1,0.03));  %0.03 is the height of the pill bottle, translating so that they sit on top of the table
            Green2 = GreenPills(transl(0.2,-0.175,0.03));
            Green3 = GreenPills(transl(0.2,-0.25,0.03));
            Purple1 = PurplePills(transl(0.1,-0.1,0.03));
            Purple2 = PurplePills(transl(0.1,-0.175,0.03));
            Purple3 = PurplePills(transl(0.1,-0.25,0.03));
            Orange1 = OrangePills(transl(0.01,-0.1,0.03));
            Orange2 = OrangePills(transl(0.01,-0.175,0.03));
            Orange3 = OrangePills(transl(0.01,-0.25,0.03));

            side = 0.1;
            centerpnt = [0.1,0.2,(side/2)];

            plotOptions.plotFaces = true;
            [v,f,fn] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

            steps = 50;
            q1 = dobot.model.getpos()
            q2 = dobot.model.ikcon(transl(0.2,0.25,0.03) )
            hold on
            qMatrix = jtraj(q1,q2,steps);
            fps = 40;
            for i = 1:steps
                dobot.model.plot(qMatrix(i,:),'fps',fps , 'nojoints', 'noname', 'noarrow', 'notiles' , 'nobase', 'nowrist' , 'noshading' , 'ortho'    );%,'noarrow','workspace',[-1 1 -1 1 -0.555 1], 'tile1color',[1 1 1 ]);
                result = IsCollision(dobot,qMatrix(i,:),f,v,fn);
                 if result == 1
                    qMatrix(i,:);
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                    break
                end
            end
        end
    end
end

function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(dobot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), dobot);
    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                disp('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, dobot)

links = dobot.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = dobot.model.base;

for i = 1:length(links)
    L = links(1,i);

    current_transform = transforms(:,:, i);

    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end