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
            
            FIXME: Il Jacobiano richiesto è quello del LINK i rispetto alla base (Q1.5). Nel codice viene usata la posizione dell’end-effector invece della posizione del link i.

            bJi = zeros(6,i);

            TODO: Verificare se il Jacobiano deve essere 6×i o 6×n per coerenza con il resto del codice.

            for j = 1:i

                if (self.gm.jointType(j) == 1)
                
                    FIXME: Contributo del giunto prismatico errato. Per un giunto prismatico J_v = z_j e J_omega = 0.

                    bJi(:,j) = [0; 0; 0; 0; 0; 1];

                end

                if (self.gm.jointType(j) == 0)

                    FIXME: Per il Jacobiano del link i non deve essere usata la trasformazione dell’end-effector. Serve usare la trasformazione del link i (bTi).

                    bTn = self.gm.getTransformWrtBase(self.gm.jointNumber);
                    brn = bTn(1:3,4);
                    bTj = self.gm.getTransformWrtBase(j);
                    brj = bTj(1:3,4);
                    jrn = brn - brj;
                    
                    FIXME: Asse di rotazione hard-coded a [0 0 1]^T. L’asse z_j va estratto dal frame j espresso nella base.

                    SM = [0 -1  0;
                          1  0  0;
                          0  0  0];
                    
                    FIXME: Skew-symmetric matrix fissa non coerente con l’asse reale del giunto. Il termine corretto è z_j x (r_i - r_j).

                    JL = SM * jrn;

                    TODO: Specificare nel report la formula del Jacobiano geometrico usata per giunti rotativi e prismatici.
                    FIXME: La colonna del Jacobiano assume implicitamente un asse di rotazione fisso.

                    bJi(:,j) = [0; 0; 1; JL];
                end
            end

        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO

            self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);

        end
    end
end

