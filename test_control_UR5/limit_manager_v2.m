function AxeRes=limit_manager_v2(LimitationActive,Axe)
    if isempty(LimitationActive)
        AxeRes = Axe;
    else
        vect_ok = gram_schmidth(LimitationActive);
        AxeRes = [0, 0, 0, 0, 0, 0]';
        AxeRes = AxeRes + project_vector(vect_ok, Axe);
        if norm(AxeRes)>1
            AxeRes = AxeRes / norm(AxeRes);
        end
    end
%     if any(isnan(AxeRes))
%         AxeRes = Axe;
%     end
end