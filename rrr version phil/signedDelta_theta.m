function signed_delta_theta   = signedDelta_theta(theta_next,theta_act)

diff_theta=theta_next-theta_act;
for i=1:3
    if diff_theta(i)>pi
        diff_theta(i)=theta_next(i)-(theta_act(i)+2*pi);
    elseif diff_theta(i)<-pi
        diff_theta(i)=theta_next(i)+2*pi-theta_act(i);
    end
end

signed_delta_theta=diff_theta; %on traduit en vitesse angulaire rad/s

