function F2 = compress_features(F,n)
%F2 = compress_features(F,n)

[V,D] = eigs(cov(F),n);
F2 = F*V;
