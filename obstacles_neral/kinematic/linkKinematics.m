function [p,Jp] = linkKinematics(q)

d1=0.1519;
d2=0.1198;
a3=0.24365;
a4=0.21325;
d3=-0.0925;
d4=0.08535;
d5=0.08535;
d6=0.0819;

T01 = DH_transform(0,0,d1,q(1));
T12 = DH_transform(0,-pi/2,d2,q(2));
T23 = DH_transform(a3,0,d3,q(3));
T34 = DH_transform(a4,0,d4,q(4));
T45 = DH_transform(0,pi/2,d5,q(5));
T56 = DH_transform(0,-pi/2,d6,q(6));

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

z = zeros(3,6);
p = zeros(3,6);

z(:,1) = [0;0;1];
z(:,2) = T01(1:3,3);
z(:,3) = T02(1:3,3);
z(:,4) = T03(1:3,3);
z(:,5) = T04(1:3,3);
z(:,6) = T05(1:3,3);

p(:,1) = T01(1:3,4);
p(:,2) = T02(1:3,4);
p(:,3) = T03(1:3,4);
p(:,4) = T04(1:3,4);
p(:,5) = T05(1:3,4);
p(:,6) = T06(1:3,4);

p0=[0;0;0];

Jp = zeros(3,6,6); 
% Jp(:,:,i) = Jacobian của link i

for i=1:6

    for j=1:i

        if j==1
            pj=p0;
        else
            pj=p(:,j-1);
        end

        Jp(:,j,i)=cross(z(:,j),p(:,i)-pj);

    end

end

end