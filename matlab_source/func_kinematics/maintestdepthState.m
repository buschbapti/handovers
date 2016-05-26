
function [] = maintestdepthState()

    clear
    clc
    close all
    initialize_dropbox_path(1, 1);
  
    load('Texample.mat');
    T0 = T(:,:,end);
    
    h_ =  figurewe('debugAngles');
    xlabel 'x'; ylabel 'y'; zlabel 'z';
    angles = homogTransfMatrixProjectedAngles(T0, h_ );
    
    angles.vectorX.aroundWorldXYZ
    angles.vectorY.aroundWorldXYZ
    angles.vectorZ.aroundWorldXYZ
    
    [~,~,~, T_] =  moveStateOnCartesianPlane( 'xy', angles.vectorZ.aroundWorldXYZ(3) , 0.3 );
    T0 = T0 + T_;
    angles = homogTransfMatrixProjectedAngles( T0 , h_ );
    
    [~,~,~, T_]  =  moveStateOnCartesianPlane( 'xz', pi/2 , 0.3 );
    T0 = T0 + T_;
    angles = homogTransfMatrixProjectedAngles( T0 , h_ );
    
    [~,~,~, T_] =  moveStateOnCartesianPlane( 'yz', pi/4 , 0.5 );
    T0 = T0 + T_;
    angles = homogTransfMatrixProjectedAngles( T0 , h_ );
    
end





