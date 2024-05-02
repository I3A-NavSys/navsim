classdef InterpolationModes
   enumeration
      % PV        % INPUT {position velocity} OUTPUT {time=CTE}
      TP        % INPUT {time position} OUTPUT {velocity=CTE}
      TPV       % INPUT {time position velocity} OUTPUT {aceleration0 jerk=CTE}
      TPV0      % INPUT {time position velocity aceleration=0} OUTPUT {jerk0 jolt=CTE}
   end
end