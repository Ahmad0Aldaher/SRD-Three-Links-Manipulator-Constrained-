close all; clear classes; clc;


Handler_IK_Solution = SRD_get('Handler_IK_Solution');

qva = Handler_IK_Solution.get_position_velocity_acceleration(0);

Handler_State = SRDHandler_State(...
    'InitialPosition', qva(:, 1) + 0.1*randn(size(qva, 1), 1), ...
    'InitialVelocity', qva(:, 2) + 0.0*randn(size(qva, 1), 1));
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

Handler_dynamics_Linear_model_evaluator = SRDHandler_dynamics_Linear_model_evaluator_c(...
    'Handler_dynamics_generalized_coordinates_model', Handler_dynamics_GC_model_evaluator, ...
    'Handler_dynamics_Linearized_Model', Handler_dynamics_Linearized_Model, ...
    'Handler_Constraints_Model', Handler_Constraints_Model, ...
    'Handler_State', Handler_State, ...
    'Handler_Controller', []);

Handler_Constraints_Model.Handler_dynamics_generalized_coordinates_model = Handler_dynamics_generalized_coordinates_model;

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

dummy_controller.u = zeros(Handler_dynamics_Linear_model_evaluator.dof_control, 1);
Handler_dynamics_Linear_model_evaluator.Handler_Controller = dummy_controller;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Handler_Updater = SRDHandler_Updater({...
    Handler_Desired_State, ...
    Handler_State_StateSpace, ...
    Handler_ObserverState_genCoord, ...
    Handler_MeasuredOutput, ...
    Handler_dynamics_GC_model_evaluator,...
    Handler_InverseDynamics, ...
    Handler_nominal_trajectory_state, ...
    Handler_dynamics_Linear_model_evaluator
    });


Handler_Updater.Update();



F = Handler_Constraints_Model.get_Jacobian(Handler_State.q);
dF = Handler_Constraints_Model.get_Jacobian_derivative(Handler_State.q, Handler_State.v);

A=Handler_dynamics_Linear_model_evaluator.A;

B=Handler_dynamics_Linear_model_evaluator.B;
J = Handler_Constraints_Model.get_Jacobian(Handler_State.q);
dJ = Handler_Constraints_Model.get_Jacobian_derivative(Handler_State.q, Handler_State.v);

save('A','A')
save('B','B')
save('J','J')
save('dJ','dJ')

