classdef GreenPills < handle
    properties
        GreenPillsPose;
        updatedPoints;
        GreenPillsVertexCount;
        GreenPillsVerts;
        midPoint;
        GreenPills_h;

    end
   methods
       function self = GreenPills(GreenPillsPose)
                self.GreenPillsPose = GreenPillsPose;
                self.location(self.GreenPillsPose);
                
       end
       function location(self,GreenPillsPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('GreenPills.ply','tri');

        % Get vertex count
        self.GreenPillsVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.GreenPillsVertexCount;
        self.GreenPillsVerts = v - repmat(self.midPoint,self.GreenPillsVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.GreenPills_h = trisurf(f,self.GreenPillsVerts(:,1),self.GreenPillsVerts(:,2), self.GreenPillsVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.GreenPillsPose = GreenPillsPose;
        self.updatedPoints = [self.GreenPillsPose * [self.GreenPillsVerts,ones(self.GreenPillsVertexCount,1)]']';  

        % Now update the Vertices
        self.GreenPills_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)

                self.GreenPillsPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.GreenPillsPose * [self.GreenPillsVerts,ones(self.GreenPillsVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.GreenPills_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
