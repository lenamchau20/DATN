classdef VerticalEllipsoid < Shapes
   properties
      ellipseAxes

   end

   methods
    
       function self = VerticalEllipsoid(axes, position, velocity_function, rho)

           param = struct();
           param.ellipseAxes = axes; % [a; b; c]
           
           %% ------ Write your code below ------
           %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

           % Fill the gamma functions and the gradient
           % The parameter param.ellipseAxes is a 3D vector with the three
           % semi-axes of an ellipse
           
           mu = @(x,y,z,param) 1 ./ sqrt( ...
            (x ./ param.ellipseAxes(1)).^2 + ...
            (y ./ param.ellipseAxes(2)).^2 + ...
            (z ./ param.ellipseAxes(3)).^2 );

           gamma = @(x, y, z, param) 1 ./ mu(x,y,z,param);
           gradientGamma = @(x, y, z, param) [x ./ (param.ellipseAxes(1).^2).* gamma(x, y, z, param);... 
               y ./ (param.ellipseAxes(2).^2) .* gamma(x, y, z, param);...
               z ./ (param.ellipseAxes(3).^2) .* gamma(x, y, z, param)];

           % To complete in TASK 4
           gammaDistance = @(x, y, z, param) sqrt(x.^2 + y.^2 + z.^2) .*(1 - mu(x,y,z,param));

           %  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
           %% ------ Write your code above ------


           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.ellipseAxes = axes;
       end

   end

end
