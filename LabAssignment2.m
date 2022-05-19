classdef LabAssignment2 < handle
    properties
        Dobot_A2 %Robot class
        workspace = [-5 5 -3 3 -0.5 2];
        pill_height = {};
        pillMatrix = {};

    end

    methods
        function obj = LabAssignment2(traj) %constructor

            startpose =transl(0.4,-0.3,0);
            tablepose = transl(0,0,0);
            obj.GenerateEnvironment(startpose,tablepose); %call generate environment fuction
            obj.PickUpTrajectory(traj); %call trajectory fuction
            % disp('Press Enter to Continue');
            % pause();
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
            trisurf(f,v(:,1) - 1, v(:,2) - 1, v(:,3) + 0 ,'FaceVertexCData',...
                vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            surf([-1.5,-1.5;1.5,1.5],[-1,1;-1,1],[-0.5,-0.5;-0.5,-0.5],...
                'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            surf([0.3,0.1;0.3,0.1],[0.55,0.55;0.55,0.55],[0,0;-0.1,-0.1],...
                'CData',imread('pick_up_sign.jpg'),'FaceColor','texturemap');

        end

        function PickUpTrajectory(obj,traj)

            disp('Generating Dobot'); %generating robot
            dobot = Dobot_A2;
            qHome1 = [0 0.7854 0.7854 0 0];

            Boxinitpos = BoxClass(transl(0.25,0.1,0.09));


            Green1 = GreenPills(transl(0.2,-0.1,0.03));  %0.03 is the height of the pill bottle, translating so that they sit on top of the table
            Green2 = GreenPills(transl(0.2,-0.175,0.03));
            Green3 = GreenPills(transl(0.2,-0.25,0.03));


            Purple1 = PurplePills(transl(0.1,-0.1,0.03));
            Purple2 = PurplePills(transl(0.1,-0.175,0.03));
            Purple3 = PurplePills(transl(0.1,-0.25,0.03));

            Orange1 = OrangePills(transl(0.01,-0.1,0.03));
            Orange2 = OrangePills(transl(0.01,-0.175,0.03));
            Orange3 = OrangePills(transl(0.01,-0.25,0.03));


            if traj == 1 %1 green
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Green1.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end


                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green1.move(newPose1);
                    drawnow();
                end

                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Green1.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end

                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end


            end

            if traj == 2 % 2 green

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Green1.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Green2.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green2.move(newPose1);
                    drawnow();
                end %end pill bottle 2

                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Green1.move(newPose1*transl(0,0.075,0));
                    Green2.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 3 % 3 green

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Green1.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Green2.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(-0.075,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green2.move(newPose1);
                    drawnow();
                end %end pill bottle 2

                q1 = dobot.model.getpos; %pill bottle 3
                q2 = dobot.model.ikcon(Green3.GreenPillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Green3.move(newPose1);
                    drawnow();
                end %end pill bottle 3

                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Green1.move(newPose1*transl(0,0.075,0));
                    Green2.move(newPose1*transl(-0.075,0.075,0));
                    Green3.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 4 % 1 Purple

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Purple1.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple1.move(newPose1);
                    drawnow();
                end %end pill bottle 1
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Purple1.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end

            end
            if traj == 5 % 2 Purple

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Purple1.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Purple2.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple2.move(newPose1);
                    drawnow();
                end %end pill bottle 2
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Purple1.move(newPose1*transl(0,0.075,0));
                    Purple2.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 6 % 3 Purple

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Purple1.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Purple2.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0.075,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple2.move(newPose1);
                    drawnow();
                end %end pill bottle 2

                q1 = dobot.model.getpos; %pill bottle 3
                q2 = dobot.model.ikcon(Purple3.PurplePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Purple3.move(newPose1);
                    drawnow();
                end %end pill bottle 3
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Purple1.move(newPose1*transl(0,0.075,0));
                    Purple2.move(newPose1*transl(0.075,0.075,0));
                    Purple3.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 7 % 1 Orange

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Orange1.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange1.move(newPose1);
                    drawnow();
                end %end pill bottle 1
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Orange1.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 8 % 2 Orange

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Orange1.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Orange2.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange2.move(newPose1);
                    drawnow();
                end %end pill bottle 2
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Orange1.move(newPose1*transl(0,0.075,0));
                    Orange2.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
            if traj == 9 % 3 Orange

                q1 = dobot.model.getpos; %pill bottle 1
                q2 = dobot.model.ikcon(Orange1.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange1.move(newPose1);
                    drawnow();
                end %end pill bottle 1

                q1 = dobot.model.getpos; %pill bottle 2
                q2 = dobot.model.ikcon(Orange2.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose*transl(0.075,0.075,0));
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange2.move(newPose1);
                    drawnow();
                end %end pill bottle 2

                q1 = dobot.model.getpos; %pill bottle 3
                q2 = dobot.model.ikcon(Orange3.OrangePillsPose);
                qMatrix = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix(i,:)); % Moving the robot to pill bottle
                    drawnow();
                end
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(Boxinitpos.BoxPose);
                qMatrix5 = jtraj(q1,q2,50);
                for i = 1:50
                    dobot.model.animate(qMatrix5(i,:));
                    newPose1 = dobot.model.fkine(qMatrix5(i,:));
                    Orange3.move(newPose1);
                    drawnow();
                end %end pill bottle 3
                finalpos = transl(0.25,0.34,0.09);
                q1 = dobot.model.getpos;
                q2 = dobot.model.ikcon(finalpos);
                qMatrix6 = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix6(i,:));
                    newPose1 = dobot.model.fkine(qMatrix6(i,:));
                    Orange1.move(newPose1*transl(0,0.075,0));
                    Orange2.move(newPose1*transl(0.075,0.075,0));
                    Orange3.move(newPose1);
                    Boxinitpos.move(newPose1);
                    drawnow();
                end
                %Return back to home once finished
                q1 = dobot.model.getpos;
                q2 = qHome1;
                disp(q1)
                disp(q2)
                qMatrix = jtraj(q1,q2,50);

                for i = 1:50
                    dobot.model.animate(qMatrix(i,:));
                    drawnow();
                end
            end
        end
    end
end



































