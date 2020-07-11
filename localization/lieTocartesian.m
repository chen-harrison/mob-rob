function lieTocartesian(filter)


end

function y = func(x)
G1 = [0     0     1;...
      0     0     0;
      0     0     0];

G2 = [0     0     0;...
      0     0     1;
      0     0     0];

G3 = [0    -1     0;...
      1     0     0;
      0     0     0];
  
X = expm(x(1)*G1 + x(2)*G2 + x(3)*G3);
y = [];
y(1,1) = X(1,3);
y(2,1) = X(2,3);
y(3,1) = atan2(X(2,1), X(1,1));
end
  