function f = sig(z)
eps = 0.01;
f = (1/eps)*(sqrt(1+eps*norm(z)^2)-1);
end