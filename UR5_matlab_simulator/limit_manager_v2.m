function AxeRes=limit_manager_v2(LimitationActive,Axe)
    LimitationActive;
    Axe;
    if isempty(LimitationActive)
        AxeRes = Axe;
    else
        vect_ok = gram_schmidth(LimitationActive);
        AxeRes = [0, 0, 0, 0, 0, 0]';
        AxeRes(1:3) = project_vector(vect_ok(1:3, :), Axe(1:3));
        AxeRes(4:6) = project_vector(vect_ok(4:6, :), Axe(4:6));
        % AxeRes = AxeRes + project_vector(vect_ok, Axe);
        if norm(AxeRes)>1
            AxeRes = AxeRes / norm(AxeRes);
        end
    end
%     if any(isnan(AxeRes))
%         AxeRes = Axe;
%     end
end