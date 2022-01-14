function v2 =  seperation(detected_Point,r)
dist = pdist2([0 0 0],detected_Point);
dist(dist > r) = 0;
e = exp((-5*(dist)-r));
dist(dist >= r) = 1;
e = e.*dist;
v2 = e*detected_Point;
v2(isnan(v2)) = 0;
v2(v2 == Inf) = 0;

v2 = -v2;
v2(1) = 0;
v2(v2 == Inf) = 0;



% v2 = dist*detected_Point;
% v2(v2 == Inf) = 0;
% v2(isnan(v2)) = 0;
% v2 =-v2;
