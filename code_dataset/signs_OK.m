% Checks sign consistence of F and x
function OK = signs_OK(F,x1,x2)
[u,s,v] = svd(F');
e1 = v(:,3);
l1 = vgg_contreps(e1)*x1;
s = sum( (F*x2) .* l1 );
OK = all(s>0) | all(s<0);
return