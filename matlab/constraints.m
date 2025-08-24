function [c, ceq] = constraints(x)
    A = [x(1) x(2) x(3);
         x(2) x(4) x(5);
         x(3) x(5) x(6)];
    % Optional: enforce positive-definite A
    eigsA = eig(A);
    c = -eigsA + 1e-5;  % c <= 0 → all eig > 0
    ceq = [];
end
