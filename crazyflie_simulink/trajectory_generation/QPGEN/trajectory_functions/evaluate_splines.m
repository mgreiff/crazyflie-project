function val = evaluate_splines( P, times, N, T)
    %% function R = evaluate_splines(...)

    index = sum(times <=  T);
    % Treats the case when the time is outside the defined interval [splineT(1),splineT(end)]
    if index == 0
        T = times(1);
        index = 1;
    elseif T >= times(end)
        T = times(end);
        index = length(P)/(N + 1);
    end
    tp = T - times(index);
    
    coeff = P((index-1)*(N+1)+1:index*(N+1));
    coeff = coeff(length(coeff):-1:1);
    val = zeros(3,1);
    val(1) = polyval(coeff, tp);
    val(2) = polyval(polyder(coeff), tp);
    val(3) = polyval(polyder(polyder(coeff)), tp);
end

