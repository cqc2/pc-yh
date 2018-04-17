[a b c]=size(xx);
vv = [];v = [];
for k1 = 1:a
    for k2 = 1:b
        for k3 = 1:c
           v =  xx{k1,k2,k3};
           if v(1)~=0
               vv = [vv;v(1:6)'];
           end
        end
    end
end
quiver3(vv);