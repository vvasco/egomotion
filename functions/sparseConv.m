function val = sparseConv(w, f)

height = size(w, 1);
width = size(w, 2);

val = 0;
norm = 0;
for j = 1:height
    for i = 1:width
%         if(w(j, i) ~= 0)
            val = (val + (w(j, i) * f(j, i)));
            norm = norm + abs(f(j, i));
%         end
    end
end

if norm ~= 0
    val = val/norm;
end
