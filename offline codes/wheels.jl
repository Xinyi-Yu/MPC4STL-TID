function delete_emptyset(union::UnionSetArray)
    nonempty_list=[]
    for i in 1:length(union)
        if union[i] != LazySets.∅(4)
            push!(nonempty_list,i)
        end
    end
    nonempty_vector = [union[nonempty_list[i]] for i in 1: length(nonempty_list)]
    if nonempty_vector != []
        nonempty = UnionSetArray(nonempty_vector)
        return nonempty
    else
        return LazySets.∅(4)
    end
end
function delete_emptyset(vector::EmptySet)
    return LazySets.∅(4)
end



function union_unionsetarray(working::Vector)
    temp = []
    if working != []
        for i in 1:length(working)
            if working[i] != LazySets.∅(4)
                for j in 1:length(working[i])
                        push!(temp, working[i][j])
                end
            end
        end
    end
    if temp != []
        vector = [temp[i] for i in 1: length(temp)]
        union_unionsetarray = UnionSetArray(vector)
        return union_unionsetarray
    else
        return LazySets.∅(4)
    end
end