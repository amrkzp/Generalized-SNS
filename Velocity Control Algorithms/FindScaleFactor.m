function taskScale = FindScaleFactor(low, upp, a)
% findScaleFactor   Find the scaling factor for a given range and value.
%
%   taskScale = findScaleFactor(low, upp, a) finds the scaling factor for a given range of values [low, upp]
%   and a value a. The scaling factor is used to determine the feasibility of the task.
%
%   Inputs:
%   - low: Lower bound of the range (scalar)
%   - upp: Upper bound of the range (scalar)
%   - a: Value to be compared (scalar)
%
%   Output:
%   - taskScale: Scaling factor (scalar)
%       - taskScale = low / a if a <= 0 and low <= 0 and a < low
%       - taskScale = 1.0 if a <= 0 and low <= 0 and a >= low
%       - taskScale = upp / a if a >= 0 and upp >= 0 and upp < a
%       - taskScale = 1.0 if a >= 0 and upp >= 0 and upp >= a
%       - taskScale = 0.0 if a is not within the specified range or if the task is infeasible
%
%   Contact email: amrkzp@gmail.com (Amirhossein Kazemipour)

if (a < 1e10 && a > -1e10)
    
    if a <= 0 && low <= 0
        
        if a < low
            taskScale = low / a; % task is feasible with scaling
        else
            taskScale = 1.0; % task is feasible without scaling
        end
        
    elseif a >= 0 && upp >= 0
        
        if upp < a
            taskScale = upp / a; % task is feasible with scaling
        else
            taskScale = 1.0; % task is feasible without scaling
        end
        
    else
        taskScale = 0.0; % task is infeasible
    end
    
else
    taskScale = 0.0; % task is infeasible
end

end
