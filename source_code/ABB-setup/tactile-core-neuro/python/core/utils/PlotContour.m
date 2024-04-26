classdef PlotContour < handle
    properties  
        hfig
        hvid
        pos = 400*[1.5 0.1 1.5 1];
        frameRate = 5; %2.25;
    end
    
    methods
        function self = PlotContour(traj)               
            self.hfig = figure('Name','Contour', 'color','w', 'MenuBar','None', 'Position',self.pos); 
            clf; hold; axis('equal', 'off') 
            try % 3D surface 
                for i = 1:3; S{i} = reshape(traj(:,i), sqrt(size(traj,1)), []); end
                surf(S{1}, S{2}, S{3}, 'edgecolor','none', 'facealpha',0.9, ...
                    'specularstrength',1, 'specularexponent',10); colormap(bone); 
                h = light; h.Position = [1 0 -1]; view([-1 0 0.5]);                         
            catch % 2D or 3D no fit
                plot3(traj(:,1), traj(:,2), traj(:,3), ':k'); 
                % view([0 0 1]); camroll(90)   % 2D  
                % view([-0.1 -0.5 0.5]) % brain 
                view([-1 0 0.5]) % lid disk 
                % view([1 0.25 -0.25]) % head 
            end 
            axis(reshape([min(traj(:,1:3))-20; max(traj(:,1:3))+20], [], 6))        
            self.magnifyFig(gca, 2); 
            
            self.hvid = VideoWriter(fullfile(getenv('TEMPPATH'), [self.hfig.Name '.avi']));
            self.hvid.FrameRate = self.frameRate;
            open(self.hvid);

            for i = 1:10; writeVideo(self.hvid, getframe(self.hfig)); end
        end

        function update(self, v)
            u = [v(end,:); self.transFrame([0 0 -5 0 0 0], v(end,:))];
            w = [self.transFrame([5 0 0 0 0 0], v(end,:)); self.transFrame([-5 0 0 0 0 0], v(end,:))];        
            
            figure(self.hfig); set(gca,'zdir','reverse','ydir','reverse')
            try plot3(v(end-1:end,1), v(end-1:end,2), v(end-1:end,3), '-r'); end
            plot3(u(:,1), u(:,2), u(:,3), '-r', 'markersize', 1);
            plot3(w(:,1), w(:,2), w(:,3), '-b', 'markersize', 1);

            writeVideo(self.hvid, getframe(self.hfig));
        end
        
        function finish(self, filename)
            savefig(self.hfig, [filename '.fig']);
            close(self.hvid);
            movefile(fullfile(self.hvid.Path,self.hvid.Filename), [filename '.avi'])
        end
        
        function magnifyFig(self, ax, n)
            for i = 1:n
                o = ax.OuterPosition; ti = ax.TightInset;
                ax.Position = [o(1)+ti(1) o(2)+ti(2) o(3)-ti(1)-ti(3) o(4)-ti(2)-ti(4)];
            end
        end
        
        function vecFrame = transFrame(self, vec, frame)
            vecQ = self.etr2qtr(vec);       % wrt frame [0 0 1 0 0 0]
            frameQ = self.etr2qtr(frame);   % transform (translate,rotate) to this frame
            vecPosQ = frameQ(1:3) + quatrotate(quatinv(frameQ(4:7)), vecQ(1:3));
            vecRotQ = quatmultiply(frameQ(4:7), vecQ(4:7));
            vecFrame = self.qtr2etr([vecPosQ, vecRotQ]);
        end
        
        function qtr = etr2qtr(self, etr) 
            pos = etr(:, 1:3);
            quat = eul2quat(deg2rad(etr(:, 4:6)), 'XYZ');
            qtr = [pos, quat];
        end
        
        function etr = qtr2etr(self, qtr)
            pos = qtr(:, 1:3);
            eul = quat2eul(qtr(:, 4:7), 'XYZ');
            etr = [pos, rad2deg(eul)];    
        end

    end 
end

