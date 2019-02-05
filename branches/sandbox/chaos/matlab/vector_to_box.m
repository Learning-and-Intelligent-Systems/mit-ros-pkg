function box = vector_to_box(w)
%box = vector_to_box(w)

box.pmin = w(1:3);
box.dims = w(4:6);
box.theta = w(7);

