using LightGraphs, GraphUtils
using LDrawParser
using HierarchicalGeometry
using LazySets
using Polyhedra
using LightGraphs, GraphUtils, Test
using GeometryBasics
using CoordinateTransformations
using Rotations
using StaticArrays
using MeshCat
using LinearAlgebra
# using Meshing
using Colors
using Test

filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","ATTEWalker.mpd")
model = parse_ldraw_file(filename)
LDrawParser.populate_part_geometry!(model)
model

# construct model graph
model_graph = construct_assembly_graph(model)
model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
print(model_tree,v->summary(v.val),"\t")

sched = LDrawParser.construct_model_schedule(model)
msched = LDrawParser.extract_single_model(sched,"20009 - AT-TE Walker.mpd")

GraphUtils.validate_graph(msched)
