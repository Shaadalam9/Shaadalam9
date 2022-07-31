function x = saturate(x,a)

if size(x) == 1
    if abs(x) > a
        x = sign(x)*a;
    end
else
    x(abs(x)>a) = sign(x(abs(x)>a))*a;
end