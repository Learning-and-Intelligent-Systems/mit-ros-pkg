function fprintf_formula(f,s)
%fprintf_formula(f,s)

idx = [1, find(s==' ')];
pos = 1000;
for j=2:length(idx)
    fprintf(f, s(idx(j-1):idx(j)-1));
    if idx(j) > pos
        fprintf(f, ' ...\n        ');
        pos = pos + 1000;
    end
end
fprintf(f, s(idx(end):end));
