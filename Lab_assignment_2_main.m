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
final_q = obj.transformation2Q_rob1(final_pos);
disp("Does it get to here?");
% obj.rmrc(start_q,final_q,start_q,start_q);




check_coll_start_cube = transl(0.5,-0.5,0.4);
check_coll_end_cube = transl(0.5,0.5,0.4);
check_coll_startq_cube = obj.transformation2Q_rob1(check_coll_start_cube);
check_coll_endq_cube = obj.transformation2Q_rob1(check_coll_end_cube);


% obj.pickApples()
% start_q = obj.rob1.model.getpos();
% final_pos = transl(0,0,1);
% final_q = obj.transformation2Q_rob1(final_pos);
% disp("Does it get to here?");

obj.transform_interpolation(start_q,final_q,start_q,start_q);
