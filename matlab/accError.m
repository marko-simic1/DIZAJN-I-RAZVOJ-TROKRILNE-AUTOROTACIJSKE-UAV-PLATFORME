function err = accError(x, inputs)
    % Error funkcija za akcelerometar
    A = [x(1) x(2) x(3);
         x(2) x(4) x(5);
         x(3) x(5) x(6)];
    b = [x(7) x(8) x(9)];

    acc_corr = (inputs - b) * A';
    norms = vecnorm(acc_corr, 2, 2);
    err = mean((norms - 1).^2); % Normirano na 1g
end
