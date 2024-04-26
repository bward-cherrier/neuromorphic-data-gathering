classdef PlotControl < handle
    properties  
        hfig
        hvid
        pos = 400*[2.5 1.1 1 1];
        frameRate = 10;
        inds
    end  

    methods
        function self = PlotControl(inds)               
            self.hfig = figure('Name','Control', 'color','w', 'MenuBar','None', 'Position',self.pos); clf;
            self.inds = inds;
            
            self.hvid = VideoWriter(fullfile(getenv('TEMPPATH'), [self.hfig.Name '.avi']));
            self.hvid.FrameRate = self.frameRate;
            open(self.hvid);
        end

        function update(self, i, y, e, ei)
            figure(self.hfig); 
            nj = length(self.inds);
            name = {'pose_1', 'pose_2', 'pose_3', 'pose_4', 'pose_5', 'pose_6'};
              
            for j = 1:nj
                subplot(3, nj, j); grid on; hold on;
                plot(max(0,i-2):i-1, y(max(1,i-1):i,self.inds(j)), 'r'); xlim([0 i]);
                if j==1; ylabel('prediction'); end
                title(name{self.inds(j)})

                subplot(3, nj, nj+j); grid on; hold on;
                plot(max(0,i-2):i-1, e(max(1,i-1):i,self.inds(j)), 'r'); xlim([0 i]); 
                yl = 1+max(abs(e(:,self.inds(j)))); ylim([-yl yl])
                if j==1; ylabel('error'); end

                subplot(3, nj, 2*nj+j); grid on; hold on;
                plot(max(0,i-2):i-1, ei(max(1,i-1):i,self.inds(j)), 'r'); xlim([0 i]); 
                yl = 1+max(abs(ei(:,self.inds(j)))); ylim([-yl yl])
                if j==1; ylabel('integrated error'); end
                xlabel('time steps');
            end

            writeVideo(self.hvid, getframe(self.hfig));
        end
        
        function finish(self, filename)
            savefig(self.hfig, [filename '.fig']);
            close(self.hvid);
            movefile(fullfile(self.hvid.Path,self.hvid.Filename), [filename '.avi'])
        end
        
    end 
end

