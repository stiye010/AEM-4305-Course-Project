function C = euler3212DCM(ea)

% Extract Euler angles from input vector
psi = ea(1); % Yaw
theta = ea(2); % Pitch
phi = ea(3); % Roll

% Construct individual DCMs for 3-2-1 (yaw-pitch-roll) sequence
R3 = [ cos(psi), sin(psi), 0;
        -sin(psi),  cos(psi), 0;
             0  ,       0  , 1];

R2 = [ cos(theta), 0, -sin(theta);
              0   , 1,     0    ;
       sin(theta), 0, cos(theta)];

R1 = [1,     0     ,      0    ;
       0, cos(phi), sin(phi);
       0, -sin(phi),  cos(phi)];

% Multiply right DCMs together for 3-2-1 (apply yaw, then pitch, then roll)
C = R1 * R2 * R3;