function y = v2_bound(x,limit)
%#codegen
y = min(max(x,-limit(:)),limit(:));
end
