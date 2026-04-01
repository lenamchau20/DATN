
function T = DH_transform(a,alpha,d,theta)

ct = cos(theta);
st = sin(theta);

ca = cos(alpha);
sa = sin(alpha);

T = [ ct   -st    0    a;
      st*ca ct*ca -sa -d*sa;
      st*sa ct*sa  ca  d*ca;
      0     0      0   1];

end