
function case0(X_next::Vector,times)
    X_k_0 = delete_emptyset(intersection(X,onestepset(X, U, X_next[1],times)))
    return X_k_0
end

function case1(X_next::Vector,times)
    X_k_a_0 = delete_emptyset(intersection(X,onestepset(X, U, X_next[2],times)))
    X_k_a_1 = delete_emptyset(intersection(goal[1], onestepset(X, U, X_next[1],times)))
    X_k_a = union_unionsetarray([X_k_a_0, X_k_a_1])
    return X_k_a
end

function case2(X_next::Vector,times)
    X_k_0 = delete_emptyset(intersection(goal[2],onestepset(X, U, X_next[1],times)))
    return X_k_0
end

function com22_25(X_next::Vector,times)
    X_k_1 = case1(X_next,times);
    X_k = [delete_emptyset(UnionSetArray([X])), X_k_1]
    return X_k
end

function com_21(X_next::Vector,times)
    X_k = delete_emptyset(intersection(X,onestepset(X, U, X_next[2],times)))
    return [X_k]
end

function comp_t()
    times = 10
    X_25 = [delete_emptyset(UnionSetArray([X])), delete_emptyset(UnionSetArray([goal[1]]))];
    X_24 = com22_25(X_25,times)
    

    times = times - 1;
    X_23 = com22_25(X_24,times);

    times = times - 1;
    X_22 = com22_25(X_23,times);

    times = times - 1;
    X_21 = com_21(X_22,times);

    times = times - 1;
    X_20 = [case0(X_21,times)];

    times = times - 1;
    X_19 = [case0(X_20,times)];

    times = times - 1;
    X_18 = [case0(X_19,times)];
    
    times = times - 1;
    X_17 = [case0(X_18,times)];

    times = times - 1;
    X_16 = [case0(X_17,times)];
   
    times = 3;
    X_15 = [case2(X_16,times)];

    times = times - 1;
    X_14 = [case2(X_15,times)];

    times = times - 1;
    X_13 = [case0(X_14,times)];

    times = times - 1;
    X_12 = [case0(X_13,times)];

    X_fz = [X_12,X_13,X_14,X_15,X_16,X_17,X_18,X_19,X_20,X_21,X_22,X_23,X_24,X_25];
    return X_fz
end
#= 
function save_data(X_fz)
    jldopen("comp_results.jld", "w") do file
        write(file, "X_fz", X_fz)
    end
end =#