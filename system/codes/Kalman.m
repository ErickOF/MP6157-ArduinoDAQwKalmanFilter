function [xk1, Pk1] = Kalman(xk, Pk, y, H, R, Phi, Gamma, Q, uk)
    % Compute the Identity Matrix
    n = size(xk, 1);
    I = eye(n);

    % Transpose
    H_t = transpose(H);
    Phi_t = transpose(Phi);

    % Gain compuation
    K = Pk * H_t * inv(H * Pk * H_t + R);

    % Update values
    xk_plus = xk + K * (y - H * xk);
    Pk_plus = (I - K * H * Pk);

    % Propagation
    xk1 = Phi * xk_plus + Gamma * uk;
    Pk1 = Phi * Pk_plus * Phi_t + Q;
end
