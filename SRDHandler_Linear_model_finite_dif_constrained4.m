classdef SRDHandler_Linear_model_finite_dif_constrained4 < SRDHandler
    properties
        dof_state_space_robot;
        dof_configuration_space_robot;
        dof_control;
        dof_Constraint;
        
        last_update_q;
        last_update_v;
        last_update_u;
        
        Handler_dynamics_generalized_coordinates_model;
        Handler_dynamics_Linearized_Model;
        Handler_Constraints_Model;
        
        Handler_State;
        Handler_Controller;
        
        finite_dif_step_q;
        finite_dif_step_v;
        finite_dif_step_u;

        LinearizationType;
        TemporalType;
        
        %dx/dt = A*x + B*u + c
        A;
        B;
    end
    methods
        
        function obj = SRDHandler_Linear_model_finite_dif_constrained4(varargin)
            Parser = inputParser;
            Parser.FunctionName = 'SRDHandler_Linear_model_finite_dif_constrained';
            Parser.addOptional('Handler_dynamics_generalized_coordinates_model', []);
            Parser.addOptional('Handler_Constraints_Model', []);
            Parser.addOptional('Handler_State', []);
            Parser.addOptional('Handler_Controller', []);
            Parser.addOptional('finite_dif_step_q', 0.0001);
            Parser.addOptional('finite_dif_step_v', 0.0001);
            Parser.addOptional('finite_dif_step_u', 0.0001);
            Parser.parse(varargin{:});
            
            obj.Handler_dynamics_generalized_coordinates_model = Parser.Results.Handler_dynamics_generalized_coordinates_model;
            obj.Handler_Constraints_Model                      = Parser.Results.Handler_Constraints_Model;
            obj.Handler_State                                  = Parser.Results.Handler_State;
            obj.Handler_Controller                             = Parser.Results.Handler_Controller;
            
            
            obj.LinearizationType             = "FiniteDifferenceConstrained";
            obj.TemporalType                  = "ContiniousTime";
            obj.dof_control                   = obj.Handler_dynamics_generalized_coordinates_model.dof_control;
            obj.dof_configuration_space_robot = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            obj.dof_state_space_robot         = 2 * obj.dof_configuration_space_robot;
            obj.dof_Constraint                = obj.Handler_Constraints_Model.dof_Constraint;
            
            
            obj.finite_dif_step_q                = Parser.Results.finite_dif_step_q;
            obj.finite_dif_step_v                = Parser.Results.finite_dif_step_v;
            
            obj.finite_dif_step_u                = Parser.Results.finite_dif_step_u;

            
            if isscalar(obj.finite_dif_step_q) 
                obj.finite_dif_step_q = eye(obj.dof_configuration_space_robot) * obj.finite_dif_step_q;
            end
            if isscalar(obj.finite_dif_step_v) 
                obj.finite_dif_step_v = eye(obj.dof_configuration_space_robot) * obj.finite_dif_step_v;
            end
            
            
            if isscalar(obj.finite_dif_step_u) 
                obj.finite_dif_step_u = eye(obj.dof_control) * obj.finite_dif_step_u;
            end


            %implementing serialization for arbitrary cell arrays of handlers seems to
            %be more pain than it is worth
            obj.SerializationPrepNeeded = true;
            obj.PreSerializationPrepFunction = @PreSerializationPrepFunction;
            function PreSerializationPrepFunction(~)
                error('do not attempt to save this function; create a new one on the fly instead')
            end
        end
        
        function Update(obj, ~)
            dof = obj.dof_configuration_space_robot;
            dof_ctrl=obj.dof_control;
            squized = obj.Handler_State.get_position_velocity_acceleration();
            q = squized(:, 1);
            v = squized(:, 2);
            u = obj.Handler_Controller.u;
            a = obj.get_acceleration(q, v, u);
            
            
            F = obj.Handler_Constraints_Model.get_Jacobian(q);
            %dF = obj.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
            
            %P = eye(dof) - pinv(F)*F;
            
            %delta_Q = P * obj.finite_dif_step_q;
            %delta_V = P * obj.finite_dif_step_v;
           
            delta_Q = obj.finite_dif_step_q;
            delta_V = obj.finite_dif_step_v;
           

            %delta_U= P * obj.finite_dif_step_u;
            delta_U= obj.finite_dif_step_u;


            delta_A_q = zeros(dof, dof);  %delta_A_q = A21 * delta_Q 
            delta_A_v = zeros(dof, dof);  %delta_A_v = A22 * delta_V
            
            delta_B_u = zeros(dof, dof_ctrl);  

            for i = 1:dof
                qi = q + delta_Q(:, i);
                
                ai = obj.get_acceleration(qi, v, u);

                delta_A_q(:, i) = (ai - a)/ norm(obj.finite_dif_step_q);
                % A21(:, i) = (ai - a) / norm(delta_q);

                F = obj.Handler_Constraints_Model.get_Jacobian(qi);
                dF = obj.Handler_Constraints_Model.get_Jacobian_derivative(qi, v);
                
                constraint=F*ai+dF*v;

                if norm(constraint)>0.000001
                delta_A_q(:, i)=zeros(dof,1);
                end

            end
            A21 = delta_A_q;% * pinv(delta_Q);
            
            for i = 1:dof
                vi = v + delta_V(:, i);
                ai = obj.get_acceleration(q, vi, u);
                
                delta_A_v(:, i) = (ai - a)/norm(obj.finite_dif_step_v);
                % A22(:, i) = (ai - a) / norm(delta_v);
                F = obj.Handler_Constraints_Model.get_Jacobian(q);
                dF = obj.Handler_Constraints_Model.get_Jacobian_derivative(q, vi);
                
                constraint=F*ai+dF*vi;
                if norm(constraint)>0.000001
                    delta_A_v(:, i)=zeros(dof,1);
                end


            end
            A22 = delta_A_v; %* pinv(delta_V);
            


            for i =1:dof_ctrl
                ui=u+delta_U(:,i);
                ai = obj.get_acceleration(q, v, ui);
                delta_B_u(:, i) = (ai - a)/norm(obj.finite_dif_step_u);
                
                constraint=F*ai+dF*v;
                if norm(constraint)>0.000001
                    delta_B_u(:, i)=zeros(dof,1);
                end
            end
            B2 = delta_B_u; %* pinv(delta_U);



            obj.A = [zeros(dof, dof), eye(dof);
                     A21,             A22];
            
            obj.B = [zeros(dof,dof_ctrl);
                    B2];

            obj.last_update_q = q;
            obj.last_update_v = v;
            obj.last_update_u = u;
        end
        
        
        function a = get_acceleration(obj, q, v, u)
            k = obj.Handler_Constraints_Model.dof_Constraint;
            n = obj.Handler_dynamics_generalized_coordinates_model.dof_configuration_space_robot;
            %m = obj.Handler_dynamics_generalized_coordinates_model.dof_control;
            
            H = obj.Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q);
            c = obj.Handler_dynamics_generalized_coordinates_model.get_bias_vector(q, v);
            T = obj.Handler_dynamics_generalized_coordinates_model.get_control_map(q);
            F = obj.Handler_Constraints_Model.get_Jacobian(q);
            dF = obj.Handler_Constraints_Model.get_Jacobian_derivative(q, v);
            
            M = [H, -F'; F, zeros(k, k)];
            vec = pinv(M) * [(T*u - c); -dF*v];
            a = vec(1:n);
        end
        
        
        
        function A = get_A(obj, ~, ~, ~, ~)
            A = obj.A;
        end
        function B = get_B(obj, ~, ~, ~)
            B = obj.B;
        end
    end
end