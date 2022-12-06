using Plots.PlotMeasures
function draw_map()
    plot(LazySets.project(X, [1,3]), color=:white, showaxis = false, size = (700, 700), linealpha=1, linewidth = 3,  left_margin = 4mm)
end

function draw_goal()
    plot!(LazySets.project(goal[1], [1,3]), color=green, alpha = 0.6, linealpha=1, linewidth = 3)
    plot!(LazySets.project(goal[2], [1,3]), color=green, alpha = 0.6, linealpha=1, linewidth = 3)
    plot!(LazySets.project(goal[3], [1,3]), color=green, alpha = 0.6, linealpha=1, linewidth = 3)
    annotate!([(9,1, text("A3", 23))])
    annotate!([(1,1, text("A2", 23))])
    annotate!([(9,9, text("A1", 23))])
end

function draw_prepare(X_fz)
    X_fz_p = []
    for i in 1:length(X_fz)  
        X_fz_i = []
        for j in 1:length(X_fz[i])
            if X_fz[i][j] != LazySets.∅(4)
                X_fz_i_j_temp = []
                for k in 1:length(X_fz[i][j])
                    temp = convert(VPolytope, X_fz[i][j][k]) # there is something wrong if we directly project X_fz[i][j][k] to [1,3] (refer to as https://github.com/JuliaReach/LazySets.jl/issues/3039)
                    push!(X_fz_i_j_temp, LazySets.project(temp, [1,3]))
                end
                X_fz_i_j = UnionSetArray([X_fz_i_j_temp[k] for k in 1:length(X_fz[i][j])])
                push!(X_fz_i, X_fz_i_j)
            else
                push!(X_fz_i, LazySets.∅(2))
            end
        end
        push!(X_fz_p, X_fz_i)
    end
    return X_fz_p
end
function draw_results(X_fz_p)
    draw_map()
    plot!(X_fz_p[1][1], color=:red, alpha = 1, linealpha = 0, label = L"X_{12}^{\{2,3\}}", legendfontsize = 18,legend_position=:topleft, fillstyle = :\)
    draw_goal()
    Plots.png("terminalSetFigure")
    anim = @animate for i=1:1:length(X_fz_p)
        j = length(X_fz_p)+1-i
        if j>10
            p1 = draw_map()
            if X_fz_p[j][2] != LazySets.∅(2)
                p1 = plot!(X_fz_p[j][2], color=blue, alpha = 1, linealpha = 0, xlim = (0,10), ylim = (0,10),title = L"X_k^{\{3\}}", tickfontsize = 25)
            end
            p1 = draw_goal()
        end
        if j <=10
            p1 = draw_map()
            if X_fz_p[j][1] != LazySets.∅(2)
                if j<=4
                    p1 = plot!(X_fz_p[j][1], color=blue, alpha = 1, linealpha = 0, xlim = (0,10), ylim = (0,10), title = L"X_k^{\{2,3\}}",tickfontsize = 25)
                end
                if j>4
                    p1 = plot!(X_fz_p[j][1], color=blue, alpha = 1, linealpha = 0, xlim = (0,10), ylim = (0,10), title = L"X_k^{\{3\}}",tickfontsize = 25)
                end
            end
            p1 = draw_goal()
        end
        plot(p1, legend = false, plot_title = text("k=$(11+j)").str, plot_titlefontsize = 20, titlefontsize = 20, size = (1000, 1000), guidefontsize = 25,  left_margin = 20mm, bottom_margin = 20mm)
    end
    gif(anim, "animation.gif", fps=1)
end