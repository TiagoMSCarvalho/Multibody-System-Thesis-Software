% Transformation Vector on body frame to the Earth frame
function Earth_vector = BodytoEarth(Body_vector,theta)
e0 = theta(1);
e1 = theta(2);
e2 = theta(3);
e3 = theta(4);

Rbe = 2.*[e0^2+e1^2-(1/2), e1*e2-e0*e3, e1*e3+e0*e2;
         e1*e2+e0*e3, e0^2+e2^2-(1/2), e2*e3-e0*e1;
         e1*e3-e0*e2, e2*e3+e0*e1, e0^2+e3^2-(1/2)];

Earth_vector = Rbe.'*Impose_Column(Body_vector);
end