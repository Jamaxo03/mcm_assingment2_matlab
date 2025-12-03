%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            
            %TO DO
            bJi = zeros(6,i);

            for j = 1:i

                if (self.gm.jointType(j) == 1)

                    bJi(:,j) = [0; 0; 0; 0; 0; 1];

                end

                if (self.gm.jointType(j) == 0)

                    bTn = self.gm.getTransformWrtBase(self.gm.jointNumber);
                    brn = bTn(1:3,4);
                    bTj = self.gm.getTransformWrtBase(j);
                    brj = bTj(1:3,4);
                    jrn = brn - brj;

                    SM = [0 -1  0;
                          1  0  0;
                          0  0  0];

                    JL = SM * jrn;

                    bJi(:,j) = [0; 0; 1; JL];
                end
            end

        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO

            
        end
    end
end

