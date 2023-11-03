clc
clear all
close all



obj = Lab_assignment_2();

% obj.rob1.model.teach();
% test_pos_1 = obj.rob1.model.getpos();
% test_pos_2 = obj.transformation2Q_rob1(obj.test_pos);
% obj.transform(test_pos_1, test_pos_2, test_pos_2, test_pos_2);
start_q = deg2rad([0 -90 0 0 0 0]);
final_pos = transl(0,0,1);
final_q = obj.transformation2Q_rob1(final_pos, start_q);
% obj.rmrc(start_q,final_q,start_q,start_q);

% obj.pickApples()


% obj.GraphicalUserInterface();

% obj.pickApples()
% start_q = obj.rob1.model.getpos();
% final_pos = transl(0,0,1);
% final_q = obj.transformation2Q_rob1(final_pos);
% disp("Does it get to here?");

% obj.transform_interpolation(start_q,final_q,start_q,start_q,1);

i = 0;
while(i < 100)
    obj.RMRC_Jogging([0.01,0.01,0.01],[0.5,0,0]);
    
    disp(i);
    i = i+1;

end
disp(i);
