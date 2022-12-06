using IntervalArithmetic
using LazySets
using Plots
using Polyhedra, CDDLib
using LaTeXStrings
using JLD, HDF5
include("parameter.jl")
include("wheels.jl")
include("comp_details.jl")
include("onestep.jl")
include("draw.jl")


X_fz = comp_t();
X_fz_p = draw_prepare(X_fz);
draw_results(X_fz_p);


