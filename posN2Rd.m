function [rd] = posN2Rd(r,v,params)
% posN2Rd takes in the position and velocity from the simulation from Part
% 2 of the project and takes the paramaters struct. It outputs the position
% in the nadiar-pointing frame

N = size(r,1);
rd = zeros(N,3);

for i = 1:N
ri = r(i,:)';
vi = v(i,:)';
rd(i,:) = dcmRd2N(ri,vi) * ri;
end
end