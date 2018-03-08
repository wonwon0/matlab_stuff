function AxeRes=limit_manager_v3(LimitationActive,Axe)
    LimitationActive;
    Axe;
    if isempty(LimitationActive)
        AxeRes = Axe;
    else
        vect_ok = gram_schmidth(LimitationActive);
        AxeRes = zeros(length(Axe),1);
        AxeRes = project_vector(vect_ok, Axe);
        % AxeRes = AxeRes + project_vector(vect_ok, Axe);
        if norm(AxeRes)>1
            AxeRes = AxeRes / norm(AxeRes);
        end
    end
%     if any(isnan(AxeRes))
%         AxeRes = Axe;
%     end
end