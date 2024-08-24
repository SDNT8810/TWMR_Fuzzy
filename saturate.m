function G = saturate(T,T_Maximum)

    % saturartion 
    if (T(1) >  T_Maximum) 
        T(1) =  T_Maximum;
    end

    if (T(1) < -T_Maximum) 
        T(1) = -T_Maximum;
    end

    if (T(2) >  T_Maximum) 
        T(2) =  T_Maximum;
    end

    if (T(2) < -T_Maximum) 
        T(2) = -T_Maximum;
    end
    
    G = T;
end
