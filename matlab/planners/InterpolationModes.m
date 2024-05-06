classdef InterpolationModes
   enumeration
      TP        % INPUT {time position} 
                % OUTPUT {velocity=CTE}

      TPV0      % INPUT {time position velocity aceleration=0} 
                % OUTPUT {aceleration_dot_init aceleration_dot2_init aceleration_dot3=CTE}
   end
end