
classdef CollisionDetection < handle
    properties

        Dobot_A2_ %Robot class
        workspace = [-5 5 -3 3 -0.5 ];
        pill_height = {};
        pillMatrix = {};

        Greenselect1 = 0;
        Purpleselect1 = 0;
        Orangeselect1 = 0;

    end

    methods
        function obj = CollisionDetection(tablepose) %constructor

            startpose =transl(0.4,-0.3,0);
            tablepose = transl(0,0,0);
            obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction
            obj.detection;

            pause();
        end

        function GenerateEnvironment(obj,centre,tablepose)

            %ENVIRONMENT
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

            surf([0.1,-0.1;0.1,-0.1],[0.1,0.1;0.2,0.2],[0,0;0,0],...
                'CData',imread('pick_up_sign.jpg'),'FaceColor','texturemap');

            % dobot = Dobot_A2;

        end

        function detection(obj)
            %initlaising the robot
            disp('Generating Dobot'); %generating robot
            dobot = Dobot_A2;
            q = dobot.model.getpos;

            % initilaising the box to collide with
            centerpnt = [0.27,0,0];
            side = 0.1;
            plotOptions.plotFaces = true;
            [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
            axis equal
            camlight


            tr =zeros(4,4,6); % initialise the transform as a 4x4, and 6 -> number of joints +1
            tr(:,:,1) =dobot.model.base;
            L = dobot.model.links
            for i = 1 : 5 % between the links --> has 5 links
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha)
            end

            
            for i = 1 : size(tr,5)-1    % Go through each link and also each triangle face
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                        disp('Intersection');
                    end
                end
            end

           % Going through until there are no step sizes larger than 1 degree 
            q1 = dobot.model.getpos;
            q2 = dobot.model.ikcon(transl(0.2,0.1,0.03) );
            steps = 2;
            while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
                steps = steps + 1;
            end
            qMatrix = jtraj(q1,q2,steps)

            % Checking each of the joint states in the trajectory to work out which ones are in collision
            result = true(steps,1);
            for i = 1: steps
                result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
                robot.animate(qMatrix(i,:));
            end
        end


        %% GetLinkPoses
        % q - robot joint angles
        % robot -  seriallink robot model
        % transforms - list of transforms
        function [ transforms ] = GetLinkPoses( q, robot)

            links = robot.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.base;

            for i = 1:length(links)
                L = links(1,i);

                current_transform = transforms(:,:, i);

                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end


        %% IsCollision
        % This is based upon the output of questions 2.5 and 2.6
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)
        function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;

            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = GetLinkPoses(qMatrix(qIndex,:), robot);

                % Go through each link and also each triangle face
                for i = 1 : size(tr,5)-1
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

        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
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
    end
end

% % % Check each of the cone end rays for intersection with a UFO
% % function ufoHitIndex = CheckIntersections(endEffectorTr,coneEnds,ufoFleet)
% % ufoHitIndex = [];
% % for rayIndex = 1:size(coneEnds,1)
% %     for ufoIndex = 1:size(ufoFleet.model,2)
% %         % Ray from this part of the cone
% %         rayStart = endEffectorTr(1:3,4)';
% %         rayEnd = coneEnds(rayIndex,:);
% %
% %         % Disk representing the UFO
% %         ufoPoint = ufoFleet.model{ufoIndex}.base(1:3,4)';
% %         ufoNormal = ufoFleet.model{ufoIndex}.base(3,1:3);
% %
% %         % Check intersection of the line with the plane.
% %         [intersectionPoint,check] = LinePlaneIntersection(ufoNormal,ufoPoint,rayStart,rayEnd);
% %         % Check for an intersection which is also close to the ufoCenter
% %         if check ~= 1 || ufoFleet.shipRadius < DistanceBetweenTwoPoints(intersectionPoint,ufoPoint)
% %             continue;
% %         end
% %         ufoHitIndex = [ufoHitIndex,ufoIndex];          %#ok<AGROW>
% %     end
% % end
% % end