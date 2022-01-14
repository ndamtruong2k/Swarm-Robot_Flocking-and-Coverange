function v1 = cohension(detected_point)
if size(detected_point,1)==1
    v1 = detected_point;
else
    v1 = mean(detected_point);
end
