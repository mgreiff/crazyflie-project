function val = eval_traj( P, times, N, T)
    %% function val = evaluate_splines(...)
    % Evaluates the first five derivatives of a polynomial spline
    % trajectory on the form [p(T), p'(T), p''(T), p'''(T), p''''(T)]
    index = sum(times <=  T);
    mult = ones(5,1);
    
    % Treats the case when the time is outside the defined interval
    if index == 0
        T = times(1);
        index = 1;
        mult = [1;0;0;0;0];
    elseif T >= times(end)
        T = times(end);
        index = length(P)/(N + 1);
        mult = [1;0;0;0;0];
    end
    tp = T - times(index);
    coeff = P((index-1)*(N+1)+1:index*(N+1));
    coeff = coeff(length(coeff):-1:1);
    
    c0 = coeff;
    c1 = polyder(c0);
    c2 = polyder(c1);
    c3 = polyder(c2);
    c4 = polyder(c3);
    val = [polyval(c0, tp);
           polyval(c1, tp);
           polyval(c2, tp);
           polyval(c3, tp);
           polyval(c4, tp)].*mult;
end

