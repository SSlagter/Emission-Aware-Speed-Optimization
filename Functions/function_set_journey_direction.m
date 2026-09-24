function [U_c, d, w_d] = function_set_journey_direction(U_c, d, w_d, downstream)
% The journey initializer supplies segment values in upstream travel order.
% Current is signed relative to upstream travel: negative is adverse.

assert(ismember(downstream, [0, 1]), ...
    'downstream must be 0 or 1.');

U_c = U_c(:).';
d = d(:).';
w_d = w_d(:).';

assert(numel(U_c) == numel(d) && numel(d) == numel(w_d), ...
    'U_c, d, and w_d must have the same number of segments.');

if downstream == 1
    d = fliplr(d);
    w_d = fliplr(w_d);
    U_c = -fliplr(U_c);
end
end