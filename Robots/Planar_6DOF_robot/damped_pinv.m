function invJ = damped_pinv(J, lambdaMax, epsilon, epsilonQ)
% damped_pinv   Calculate the damped pseudoinverse of a matrix.
%
%   invJ = damped_pinv(J, lambdaMax, epsilon, epsilonQ) calculates the damped pseudoinverse
%   of the input matrix J, using damping parameters lambdaMax, epsilon, and epsilonQ.
%
%   Inputs:
%   - J: Input matrix (m x n)
%   - lambdaMax: Maximum damping parameter (scalar) [Optional]
%   - epsilon: Small value to check for singularity (scalar) [Optional]
%   - epsilonQ: Small value to check for zero singular values (scalar) [Optional]
%
%   Output:
%   - invJ: Damped pseudoinverse of the input matrix J (n x m)
%
%   Written by Fabrizio Flacco
%   Edited by Amirhossein Kazemipour
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

if nargin < 4
    epsilonQ = 1e-10;    
    if nargin < 3
        epsilon = 1e-8;
        if nargin < 2
            lambdaMax = 1e-12;
            if nargin < 1
                error(message('MATLAB:qrinsert:NotEnoughInputs'))
            end
        end
    end
end

r = 0; % initialization of the rank counter
matrixRows = size(J, 1);
matrixCols = size(J, 2);
minDim = min(matrixRows, matrixCols);

if (minDim == 0)
    invJ = J' * 0;
else
    % Singular Value Decomposition
    [U, S, V] = svd(J', 0);
    singularValues = diag(S);

    if (singularValues(minDim) > epsilon)
        % Calculate the pseudoinverse if the smallest singular value is larger than epsilon
        invS = inv(S);     
        invJ = U * invS * V';
    else
        % Otherwise, apply damping factor
        lambdaSquared = (1 - (singularValues(minDim)/epsilon)^2) * lambdaMax^2;
        for i = 1:minDim
            if (singularValues(i) > epsilonQ) 
                r = r + 1;
            end
            singularValues(i) = singularValues(i) / (singularValues(i)^2 + lambdaSquared);
        end
        invJ = U(:,1:r) * diag(singularValues(1:r)) * V(:,1:r)';
    end
end

end
