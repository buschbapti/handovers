classdef DataSaver < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
       
    properties
        instance;
        sys    =  [];
        initT =[];
        initTresampled  = [];
        param  = [];
        fileName ;
    end

methods

    function obj = DataSaver(description, sys, initTresampled, initT,  param )

        obj.instance = description;
        % Not sure if I can inherit the class Vrep directly here because it
        % will be used by multiple instances of VrepAgent. So I prefer to
        % simply copy the properties of the "parent object".
        obj.sys       = sys;
        obj.initT          = initT; % the original trajectory without resample
        obj.initTresampled = initTresampled;
        obj.param = param;       
        
        % save solution
        obj.fileName = [description '_' my_time];       
    end


    
end
   
methods(Static)
    
    function [] = saveCurrent(obj)       
        
        try mkdir(obj.fileName); end        
        obj.param.hfig = []; 
        save( [obj.fileName '/' obj.fileName '.mat'], 'obj' );
        
        
    end   
    
end

end

