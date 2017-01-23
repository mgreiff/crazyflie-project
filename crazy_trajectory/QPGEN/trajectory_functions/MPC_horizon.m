function R = MPC_horizon( P, times, N, currT, predH, predN)
    R = zeros(3, predN);
    predT = currT + predH*(0:predN-1);

    for ii = 1:predN
        R(:,ii) = evaluate_splines( P, times, N, predT(ii));
    end
end

