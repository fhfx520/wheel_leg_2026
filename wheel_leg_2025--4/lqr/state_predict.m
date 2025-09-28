function P = state_predict(A, B, Rw, Rl, t)
    P = zeros(2, 8);
    F = A .* t + eye(10);
    C = B .* t;
    temp = F(2, :) ./ Rw - F(4, :) .* Rl ./ Rw;
    P(1, 1:4) = temp(temp ~= 0);
    temp = C(2, :) ./ Rw - C(4, :) .* Rl ./ Rw;
    P(1, 5:8) = temp(temp ~= 0);
    temp = F(2, :) ./ Rw + F(4, :) .* Rl ./ Rw;
    P(2, 1:4) = temp(temp ~= 0);
    temp = C(2, :) ./ Rw + C(4, :) .* Rl ./ Rw;
    P(2, 5:8) = temp(temp ~= 0);

end