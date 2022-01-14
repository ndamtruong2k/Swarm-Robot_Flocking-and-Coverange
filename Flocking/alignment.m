function v4 = alignment(data,ori)

x = zeros(size(data,1),1);
y = zeros(size(data,1),1);

for i = 1:size(data,1)
    x(i) = data(i,1)*cos(data(i,2));
    y(i) = data(i,1)*sin(data(i,2));
end
if size(data,1)==1
    vec = [x y 0];
    vec = vec/(norm(vec));
    vec(3) = 0;
else
    vec = sum([x y zeros(size(data,1),1)]);
    vec = vec/(norm(vec));
    vec(3) = 0;
end
vec(isnan(vec)) = 0;
v4 = (inv(Rotation(ori))*vec')';
v4(1) = 0;

