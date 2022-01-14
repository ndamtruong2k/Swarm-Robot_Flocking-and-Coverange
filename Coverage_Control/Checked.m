function checked_on = Checked(target_point,assignedo)
    checked_on = 0;
    for i = 1:size(assignedo,1)
        if  sqrt(sum((target_point - assignedo(i,:)).^2)) <= 0.1
            checked_on = 1;
        end
    end

end