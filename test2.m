close all; clear; clear classes; clc;


Handler_IK_Solution = SRD_get('Handler_IK_Solution');

qva = Handler_IK_Solution.get_position_velocity_acceleration(0);

Handler_State = SRDHandler_State(...
    'InitialPosition', qva(:, 1) + 0*randn(size(qva, 1), 1), ...
    'InitialVelocity', qva(:, 2) + 0*randn(size(qva, 1), 1));
Handler_State_StateSpace = SRDHandler_StateConverter_GenCoord2StateSpace(...
    'Handler_State', Handler_State);
% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Handler_dynamics_generalized_coordinates_model = SRD_get('Handler_dynamics_generalized_coordinates_model');
Handler_dynamics_Linearized_Model = SRD_get('Handler_dynamics_Linearized_Model');
Handler_Constraints_Model = SRD_get('Handler_Constraints_Model');
 
Handler_dynamics_GC_model_evaluator = SRDHandler_dynamics_GC_model_evaluator(...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_generalized_coordinates_model, ...
    'Handler_State', Handler_State, ...
    'UsePinv', true);

Handler_Linear_model = SRDHandler_Linear_model_finite_dif_constrained2(...
     'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_GC_model_evaluator, ...
    'Handler_Constraints_Model', Handler_Constraints_Model, ...
    'Handler_State', Handler_State, ...
    'Handler_Controller', [], ...
    'finite_dif_step_q', 0.000001, 'finite_dif_step_v', 0.000001);


% Handler_Linear_model = SRDHandler_Linear_model_finite_dif_constrained_explicit(...
%      'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_GC_model_evaluator, ...
%     'Handler_Constraints_Model', Handler_Constraints_Model, ...
%     'Handler_State', Handler_State, ...
%     'Handler_Controller', [], ...
%     'finite_dif_step_q', 0.000001, 'finite_dif_step_v', 0.000001);

Handler_Constraints_Model.Handler_dynamics_generalized_coordinates_model = ...
    Handler_dynamics_generalized_coordinates_model;

C = [1,0,0,0,0,0;
     0,1,0,0,0,0;
     0,0,1,0,0,0;
     0,0,0,1,1,1];
% C = randn(4, 6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %

dt = 0.001;
tf = Handler_IK_Solution.TimeExpiration;
% tf = 1.0;

Handler_Time = SRDHandler_Time('TimeLog', 0:dt:tf);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Handler_ObserverState = SRDHandler_StateSpace(...
    'InitialState', [qva(:, 1); ...
                     qva(:, 2)]);
Handler_ObserverState_genCoord = SRDHandler_StateConverter_StateSpace2GenCoord(...
    'Handler_StateSpace', Handler_ObserverState); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %

Handler_MeasuredOutput = SRDHandler_LinearMeasuredOutput(...
    'Handler_State_StateSpace', Handler_State_StateSpace, ...
    'C', C);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Handler_Desired_State = SRD_get_handler__desired_state(...
    'Handler_ControlInput', Handler_IK_Solution, ...
    'Handler_Time',         Handler_Time);

Handler_InverseDynamics = SRD_get_handler__InverseDynamicsConstrained_QR(...
    'Handler_ControlInput', Handler_Desired_State, ...
    'Handler_Constraints_Model', Handler_Constraints_Model, ...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_GC_model_evaluator, ...
    'Handler_Time', Handler_Time);

Handler_nominal_trajectory_state = SRDHandler_nominal_trajectory_state(...
    'Handler_Time', Handler_Time, ...
    'Handler_ControlInput_qva', Handler_IK_Solution, ...
    'Handler_InverseDynamics',  Handler_InverseDynamics, ...
    'function_Original_Model',  Handler_Constraints_Model.get_FirstOrderSystem_qv_handle, ...
    'ToEvaluateOriginalFunction', true);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %

dummy_controller.u = zeros(Handler_Linear_model.dof_control, 1);
Handler_Linear_model.Handler_Controller = dummy_controller;






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Handler_Updater = SRDHandler_Updater({...
    Handler_Desired_State, ...
    Handler_State_StateSpace, ...
    Handler_ObserverState_genCoord, ...
    Handler_MeasuredOutput, ...
    Handler_dynamics_GC_model_evaluator,...
    Handler_InverseDynamics, ...
    Handler_nominal_trajectory_state, ...
    Handler_Linear_model
    });


Handler_Updater.Update();

A = Handler_Linear_model.A;
% G = Handler_Linear_model.linear_G;
disp("eig(A)")
eig(A)

% x = [q, v]
k = Handler_Constraints_Model.dof_Constraint;
n = Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
m = Handler_dynamics_generalized_coordinates_model.dof_control;

q = Handler_State.q;    v = Handler_State.v;    u = zeros(m, 1);

% a = get_acceleration(q, v, u, ...
%     Handler_dynamics_generalized_coordinates_model, ...
%     Handler_Constraints_Model);

F = Handler_Constraints_Model.get_Jacobian(q);
dF = Handler_Constraints_Model.get_Jacobian_derivative(q, v);
% P = eye(n) - pinv(F)*F;

Fss = [zeros(size(F')); F'];
Gss = [dF, F];
Pss = eye(2*n) - Fss*pinv(Gss*Fss)*Gss;
G = [ F, zeros(size(F));
     dF, F];
N = null(G);    



disp("N'*A*N   N'*Pss*A*N")
N'*A*N
N'*Pss*A*N

disp("N'*A*N")
eig(N'*A*N)


[Handler_State.q, Handler_State.v]
[Handler_Linear_model.last_update_q, Handler_Linear_model.last_update_v]
[q, v]

%     0.7854         0
%    -2.0944         0
%     2.1991         0














