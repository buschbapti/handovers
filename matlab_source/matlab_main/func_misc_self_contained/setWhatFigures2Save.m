classdef setWhatFigures2Save
% This function saves figures. Good if you are running experiments overnight
% It is useful if you call this from within the directory to save experiment
% The directory can be created by using the index of the iteration
% The figure must be already positioned and sized according to what you want to save. You should call f_FigPosSet if available
%
% 10.2009 Guilherme Maeda: file created
% 
% this class receives as input
%     fig_num  =     [n1 n2 n3 ... nn]; <= figure number in matlab
%     fig_name = eg: [{'rrt1'} {'timeresp1'} {'timeresp2'} {'param_est'} {'covariances'} ... {'runtime'}]; <= corresponding names of the figures
%     optionally: 
%         jpeg = 1/0; % save jpeg or not
%         ps   = 1/0;
%         fig  = 1/0;
%         resolution = eg: '-r200'
%         
%         if only fig_num and fig_name are given, it will set to save jpeg and fig, skipping ps figures and resolution to 200 dpi
   
    properties
        fig_num;
        fig_name
        jpeg;
        ps;
        fig;
        emf;
        pdf;
        png;
        resolution;   
    end
       
methods %=====================================================================================
        
    function obj = setWhatFigures2Save( fig_num, fig_name, opt)
        obj.fig_num    = fig_num;
        obj.fig_name   = fig_name;
        obj.jpeg       = opt.jpeg;
        obj.resolution = opt.jpegRes;
        obj.ps         = opt.ps;
        obj.fig        = opt.fig;
        obj.emf        = opt.emf;
        obj.pdf        = opt.pdf;
        obj.png        = opt.png;
    end
    
    function [] = saveFiguresNow(obj)
        N = length(obj.fig_num);
        res = obj.resolution;
        
        if size(obj.fig_name,2) ~= size(obj.fig_num,2)
             error('figure numbers and figure names do not have a correct correspondence');
        end
        
        for i= 1:N
            clear h;
            % get figure handle
            j = obj.fig_num(i);
            figure(j); 
            h = gcf; %call figure(1) and get its handle
            tmp1 = get(gcf); 
            tmp2 = get(gca); 
            tmp3 = get(h);
            set(gcf,'paperpositionmode','auto');  % save figures with aspect ration shown in screen
            if obj.jpeg
                %print(obj.fig_name{i}, res, '-djpeg', h)
                print([obj.fig_name{i} '.jpeg'], res, '-djpeg', h)
            end
            if obj.emf
                print([obj.fig_name{i} '.emf'], res, '-dmeta', h)
            end
            if obj.pdf
                print([obj.fig_name{i} '.pdf'], res, '-dpdf', h)
            end
            if obj.png
                print([obj.fig_name{i} '.png'], res, '-dpng', h)
            end            
            if obj.ps
                print(obj.fig_name{i}, res, '-dpsc', h)
            end
            if obj.fig
                saveas(h,obj.fig_name{i},'fig')
            end
         
        end % end of for
    end
    
end% methods

end % end of class