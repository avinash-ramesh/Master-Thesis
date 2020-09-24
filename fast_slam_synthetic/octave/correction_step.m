function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
	Q_t = [0.1, 0   ; 
		   0  , 0.1];

% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
		landx = robot(1,1) + z(j).range * cos(robot(3,1) + z(j).bearing);
        landy = robot(2,1) + z(j).range * sin(robot(3,1) + z(j).bearing);
		
		
      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
		particles(i).landmarks(l).mu = [landx; landy];
		particles(i).landmarks(l).sigma = H^(-1) * Q_t * (H^-1)';

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else

      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));		

      % TODO: compute the measurement covariance
		S = H * particles(i).landmarks(l).sigma * H' + Q_t;

      % TODO: calculate the Kalman gain
		K = particles(i).landmarks(l).sigma * H' * S^(-1);
		
      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
		Y = [z(j).range; z(j).bearing] - expectedZ;
		Y(2,1) = normalize_angle(Y(2,1));

      % TODO: update the mean and covariance of the EKF for this landmark
		particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K * Y;
		particles(i).landmarks(l).sigma = particles(i).landmarks(l).sigma - (K * H * particles(i).landmarks(l).sigma);

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
		particles(i).weight = particles(i).weight * exp(-0.5*Y'*S^(-1)*Y)/sqrt(2 * pi * det(S));
	
    end

  end % measurement loop
end % particle loop

end
