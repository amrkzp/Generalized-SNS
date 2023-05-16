classdef Planar_Robot<handle
    % Planar_Robot class 
    properties
        q; % 7x1 vector with the current joint configuration
        dq; % 7x1 vector with the current joint velocity
    end
    
    methods
        function obj=Planar_Robot(q,dq)
            obj.q=q;
            obj.dq=dq;
        end
        
        function obj=set_q_and_qdot(obj,q,dq)
            obj.q=q;
            obj.dq=dq;
        end
        
        function p_EE=computeEEPosition(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            p_EE = [cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5);
                sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)];
        end
        
        function J_EE=computeEEJacobian(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            J_EE = [- sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3 + q4 + q5 + q6) - sin(q1 + q2 + q3 + q4), - sin(q1 + q2 + q3 + q4 + q5) - sin(q1 + q2 + q3 + q4 + q5 + q6), -sin(q1 + q2 + q3 + q4 + q5 + q6);
                cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5),   cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5),  cos(q1 + q2 + q3 + q4 + q5 + q6)];
        end
        
        function Jdot_EE=computeEEJacobianDot(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            dq1 = obj.dq(1);dq2 = obj.dq(2);dq3 = obj.dq(3);
            dq4 = obj.dq(4);dq5 = obj.dq(5);dq6 = obj.dq(6);
            Jdot_EE = [- dq4*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq6*cos(q1 + q2 + q3 + q4 + q5 + q6) - dq5*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5)) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5)), - dq4*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq6*cos(q1 + q2 + q3 + q4 + q5 + q6) - dq5*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5)) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1 + q2 + q3 + q4 + q5)), - dq4*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq6*cos(q1 + q2 + q3 + q4 + q5 + q6) - dq5*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)), - dq1*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq2*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq3*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq4*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2 + q3 + q4 + q5)) - dq6*cos(q1 + q2 + q3 + q4 + q5 + q6) - dq5*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)), - dq6*cos(q1 + q2 + q3 + q4 + q5 + q6) - dq1*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq2*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq3*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq4*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)) - dq5*(cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4 + q5)), -cos(q1 + q2 + q3 + q4 + q5 + q6)*(dq1 + dq2 + dq3 + dq4 + dq5 + dq6);
                - dq2*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq4*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq6*sin(q1 + q2 + q3 + q4 + q5 + q6) - dq3*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq5*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)) - dq1*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)), - dq1*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq2*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq4*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq6*sin(q1 + q2 + q3 + q4 + q5 + q6) - dq3*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq5*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)), - dq4*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq6*sin(q1 + q2 + q3 + q4 + q5 + q6) - dq1*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq2*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq3*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq5*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)), - dq1*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq2*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq3*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq4*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4)) - dq6*sin(q1 + q2 + q3 + q4 + q5 + q6) - dq5*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)), - dq6*sin(q1 + q2 + q3 + q4 + q5 + q6) - dq1*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)) - dq2*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)) - dq3*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)) - dq4*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)) - dq5*(sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3 + q4 + q5 + q6)), -sin(q1 + q2 + q3 + q4 + q5 + q6)*(dq1 + dq2 + dq3 + dq4 + dq5 + dq6)];
        end
        
        function v_EE=computeEEVelocity(obj)
            J=obj.computeEEJacobian();
            v_EE=J*obj.dq;
        end
        
        function p_Ctr=computeCtrPosition(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            p_Ctr = [cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1);
                sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)];
        end

        function p_Ctrs=computeCtrsPosition(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            p_Ctrs = [cos(q1), cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5);
                sin(q1), sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)];
        end        
        
        function J_Ctr = computeCtrJacobian(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            J_Ctr = [- sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2) - sin(q1), - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4) - sin(q1 + q2), - sin(q1 + q2 + q3) - sin(q1 + q2 + q3 + q4), -sin(q1 + q2 + q3 + q4), 0, 0;
                cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2),   cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4),  cos(q1 + q2 + q3 + q4), 0, 0];
        end
        
        function J_ctrPnts = computeCtrsJacobian(obj)
            J = JcontrolPoints(obj.q(1),obj.q(2),obj.q(3),obj.q(4),obj.q(5),obj.q(6));
            J_ctrPnts = [];
            for i=1:size(J,3)
                J_ctrPnts = cat(1,J_ctrPnts,squeeze(J(:,:,i)));
            end
        end        
        
        function Jdot_Ctr = computeCtrJacobianDot(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            dq1 = obj.dq(1);dq2 = obj.dq(2);dq3 = obj.dq(3);
            dq4 = obj.dq(4);dq5 = obj.dq(5);dq6 = obj.dq(6);
            Jdot_Ctr = [- dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1)) - dq4*cos(q1 + q2 + q3 + q4) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4)), - dq4*cos(q1 + q2 + q3 + q4) - dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2)) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4)), - dq4*cos(q1 + q2 + q3 + q4) - dq1*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4)) - dq2*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4)) - dq3*(cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4)), -cos(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4), 0, 0;
                - dq4*sin(q1 + q2 + q3 + q4) - dq1*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)) - dq2*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq3*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4)), - dq4*sin(q1 + q2 + q3 + q4) - dq1*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq2*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2)) - dq3*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4)), - dq4*sin(q1 + q2 + q3 + q4) - dq1*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4)) - dq2*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4)) - dq3*(sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4)), -sin(q1 + q2 + q3 + q4)*(dq1 + dq2 + dq3 + dq4), 0, 0];
        end

        function Jdot_ctrPnts = computeCtrsJacobianDot(obj)
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            dq1 = obj.dq(1);dq2 = obj.dq(2);dq3 = obj.dq(3);
            dq4 = obj.dq(4);dq5 = obj.dq(5);dq6 = obj.dq(6);
            
            J = JdotcontrolPoints(q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6);
            
            Jdot_ctrPnts = [];
            for i=1:size(J,3)
                Jdot_ctrPnts = cat(1,Jdot_ctrPnts,squeeze(J(:,:,i)));
            end
            
        end        
        
        function v_Ctrs=computeCtrVelocity(obj)
            J=obj.computeCtrJacobian();
            v_Ctrs=J*obj.dq;
        end
        
        function v_Ctrs=computeCtrsVelocity(obj)
            J=obj.computeCtrsJacobian();
            v_Ctrs = J*obj.dq;
        end        
        
        %         function controlPoints=compute_control_points_position(obj)
        %             %             controlPoints=zeros(3,8);
        %             ARM_CONTROL_POINTS=4;
        %             FOREARM_CONTROL_POINTS=4;
        %             LINK2_LENGHT=obj.d3;
        %             LINK3_LENGHT=obj.d5;
        %
        %             p0=[0;0;0;1];
        %             %W1
        %             A=obj.transMat(obj.q(1),0,1,0,obj.d1);
        %             kineWToFrame0=A;
        %             %M2
        %             A=A*obj.transMat(obj.q(2),0,-1,0,0);
        %             %M3
        %             A=A*obj.transMat(obj.q(3),0,-1,0,obj.d3);
        %             kineWToFrame1=A;
        %             %M4
        %             A=A*obj.transMat(obj.q(4),0,1,0,0);
        %             %M5
        %             A=A*obj.transMat(obj.q(5),0,1,0,obj.d5);
        %             kineWToFrame2=A;
        %             %M6
        %             A=A*obj.transMat(obj.q(6),0,-1,0,0);
        %             %M7
        %             A=A*obj.transMat(0,1,0,0,obj.d7);
        %             kineWToFrame3=A;
        %
        %             for i=1:1:ARM_CONTROL_POINTS
        %                 l=(LINK2_LENGHT)*(i-1)/ARM_CONTROL_POINTS;
        %                 P=[0;l;0;1];
        %                 Ph=kineWToFrame1*P;
        %                 controlPoints(:,i)=Ph(1:3);
        %             end
        %             for i=1:1:FOREARM_CONTROL_POINTS
        %                 l=LINK3_LENGHT*(i-1)/FOREARM_CONTROL_POINTS;
        %                 P=[0;-l;0;1];
        %                 Ph=kineWToFrame2*P;
        %                 controlPoints(:,i+ARM_CONTROL_POINTS)=Ph(1:3);
        %             end
        %
        %             p_ee = obj.computeEnd_effectorPosition;
        %             controlPoints = cat(2,controlPoints,p_ee(1:3));
        %         end
        %
        %         function J_ctrPnts = compute_control_points_jacobian(obj)
        %             J = JcontrolPoint(obj.q(1),obj.q(2),obj.q(3),...
        %                 obj.q(4),obj.q(5),obj.q(6),obj.q(7),obj.d1,obj.d3,obj.d5,obj.d7);
        %
        %             J_ctrPnts = [];
        %             for i=1:size(J,3)
        %                 J_ctrPnts = cat(1,J_ctrPnts,squeeze(J(:,:,i)));
        %             end
        %             J_ctrPnts = cat(1,J_ctrPnts,obj.computeJacobianPos);
        %         end
        %
        %         function Jdot_ctrPnts = compute_control_points_jacobiandot(obj)
        %             J = JdotcontrolPoint(obj.q(1),obj.q(2),obj.q(3),...
        %                 obj.q(4),obj.q(5),obj.q(6),obj.q(7),...
        %                 obj.qdot(1),obj.qdot(2),obj.qdot(3),...
        %                 obj.qdot(4),obj.qdot(5),obj.qdot(6),obj.qdot(7),...
        %                 obj.d1,obj.d3,obj.d5,obj.d7);
        %
        %             Jdot_ctrPnts = [];
        %             for i=1:size(J,3)
        %                 Jdot_ctrPnts = cat(1,Jdot_ctrPnts,squeeze(J(:,:,i)));
        %             end
        %             Jdot_ctrPnts = cat(1,Jdot_ctrPnts,obj.computeJacobianDot);
        %         end
        %
        %         function [Ori,Ori_z]=computeEnd_effectorOrientation(obj)
        %             A=obj.transMat(obj.q(1),0,1,0,obj.d1);
        %             A=A*obj.transMat(obj.q(2),0,-1,0,0);
        %             A=A*obj.transMat(obj.q(3),0,-1,0,obj.d3);
        %             A=A*obj.transMat(obj.q(4),0,1,0,0);
        %             A=A*obj.transMat(obj.q(5),0,1,0,obj.d5);
        %             A=A*obj.transMat(obj.q(6),0,-1,0,0);
        %             A=A*obj.transMat(0,1,0,0,obj.d7);
        %             Ori=A(1:3,1:3);
        %             Ori_z=A(1:3,3);
        %         end
        
        %         function V_CtrPnt=compute_control_points_velocity(obj)
        %             J=obj.compute_control_points_jacobian();
        %             V_CtrPnt = J*obj.qdot;
        %         end
        
        %         function A=transMat(~,q,ca,sa,a,d)
        %             sq=sin(q);
        %             cq=cos(q);
        %             A=([cq, -sq*ca, sq*sa, a*cq; sq, cq*ca, -cq*sa, a*sq; 0, sa, ca, d; 0 0 0 1]);
        %         end
        
        function plotarm(obj)
            Alpha = 1; %ransparensy
%             Alpha = 0.4; %ransparensy
            q1 = obj.q(1);q2 = obj.q(2);q3 = obj.q(3);
            q4 = obj.q(4);q5 = obj.q(5);q6 = obj.q(6);
            p = [0,cos(q1), cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5), cos(q1 + q2 + q3) + cos(q1 + q2 + q3 + q4 + q5 + q6) + cos(q1 + q2 + q3 + q4) + cos(q1 + q2) + cos(q1) + cos(q1 + q2 + q3 + q4 + q5);
                 0,sin(q1), sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1), sin(q1 + q2 + q3 + q4 + q5) + sin(q1 + q2 + q3) + sin(q1 + q2 + q3 + q4 + q5 + q6) + sin(q1 + q2 + q3 + q4) + sin(q1 + q2) + sin(q1)];
            hold on;
            for i=1:size(p,2)-1
                hline = plot([p(1,i),p(1,i+1)],[p(2,i),p(2,i+1)],'Color','k','linewidth',3);
                hline.Color = [hline.Color Alpha];  % alpha=0.1
                hold on;
            end
%             for i=1:length(hline)
%             hline(i).Color = [hline(i).Color 0.3];  % alpha=0.1
%             end
            hline = scatter(p(1,:), p(2,:), 'r', 'MarkerFaceColor', 'r','LineWidth',3),hold on
            for i=1:length(hline)
            alpha(hline,Alpha);
            end
            grid on;
            %axis([-3 3 -3 3]);
        end
    end
end