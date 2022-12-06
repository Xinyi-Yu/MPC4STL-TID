
function onestepset(X, U, X_next,times)
    temp=[];
    Lx =(L^(times)-1)/(L-1)
    for i = 1:length(X_next)
        X_next_tmp = convert(HPolytope, minkowski_difference(X_next[i],Lx*w))
        X_previous = convert(HPolytope, inv(A)*intersection(X, minkowski_sum(X_next_tmp, convert(Zonotope,B*U))));
        if X_previous.constraints != []
            push!(temp, intersection(X, X_previous));
        end
    end
    results = UnionSetArray([temp[i] for i in 1: length(temp)]);
    return results
end


function onestepset(X, U, X_next::EmptySet)
    return LazySets.∅(4)
end


