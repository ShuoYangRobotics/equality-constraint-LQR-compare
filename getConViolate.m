function vio = getConViolate(N, param, x_list, u_list, C_list, D_list, G_list, r_list, h_list)
vio = 0;

for i=1:N
    
    if (ismember(i,param.Cxu))
        term = C_list(:,:,i)*x_list(:,i) + D_list(:,:,i)*u_list(:,i) + r_list(:,i);
        vio = vio + term'*term;
    end
    
    if (ismember(i,param.Cx))
        term = G_list(:,:,i)*x_list(:,i) + h_list(:,i);
        vio = vio + term'*term;
    end
end


end