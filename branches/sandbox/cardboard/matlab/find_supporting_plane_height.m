function h = find_supporting_plane_height(cloud, h_values)
%h = find_supporting_plane_height(cloud, h_values)

if nargin < 2
    h_values = .95:.001:1.05;
end
H = h_values;  % values to try

A = zeros(size(H));
for j=1:length(H)
    Z = cloud(cloud(:,3)>H(j), 1:2);
    A(j) = prod(max(Z) - min(Z)) + j/1000;
end
%figure(4);
%plot(H, A);
[amin jmin] = min(A);
h = H(jmin);
