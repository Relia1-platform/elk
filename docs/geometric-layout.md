# Geometric implementation

The `org.eclipse.elk.alg.geometric` plugin registers `org.eclipse.elk.geometric` through generated metadata and the Java service loader. It depends on common layout utilities, MrTree, radial, and layered. The algorithms feature includes it; elkjs compiles the same sources and registers all dependencies when only `geometric` is requested.

## Components

| Component | Responsibility |
| --- | --- |
| `GeometryGraph` | Footprints, structural adjacency, connected components, deterministic roots, and iterative BFS backbones. |
| `BalancedTreeLayout` | Two-pass subtree contours, envelope centering, uniform levels, and direction transforms. |
| `BalancedRadialLayout` | Equal root sectors, nested sectors, common level radii, and global clearance expansion. |
| `TopologyAnalysis` | Iterative Tarjan blocks, articulation points, bridges, and bounded fundamental-cycle recognition. |
| `RingLayoutKernel` | Cyclic order, anchor, orientation, equal-angle placement, clearance, and interactive rotation. |
| `MixedLayoutKernel` | Unique ownership of nodes, outward ring branches, local generic regions, and rigid region composition. |
| `TreeBusRouter` | Org-chart connectors for placed trees (`tree.routing = BUS`): one bus per parent halfway through the level gap, perpendicular drops into the children, junction points where drops leave the bus. |
| `EdgeLabelReservation` | Room for edge labels during placement: footprint extensions for tree contours, and the smallest uniform scale at which every label rectangle beside its segment clears nodes and other labels for radial and ring placement. |
| `FixedNodeRouter` | Adaptive visibility routing around fixed footprints, hierarchy waypoints, ports, and labels. Labels are placed on the finished route first; a bounded local detour and finally an exterior corridor are used only when no segment can hold them. |
| `GeometryBounds` | Complete extents, padding, root centering, fixed-size errors, and synchronized translation. |
| `GeometryTransaction` | Work on an internal copy, then transfer only geometry to the original graph objects. |

The JS-facing option reference and build commands are in the matching elkjs checkout's `docs/geometric-layout.md`. New options are declared in `Geometric.melk`; generated metadata is produced by the normal build. MrTree and radial retain their existing defaults and expose `nodePlacement = BALANCED` as an opt-in.

## Verification and acceptance

`test/geometry` contains canonical JSON fixtures shared with elkjs. `GeometricLayoutTest` verifies preservation of Java node/edge objects, bounds, spacing, centered roots, exact circles, transaction failure, and Tarjan articulation handling. It exports JVM geometry for cross-runtime comparison. `GeometricRoutingTest` independently checks fixed-port loops, labels against prior routes, and stale junction removal.

The `*-labels.json` fixtures cover a symmetric tree, an unequal tree, a six-leaf star with mixed label widths, a ring with unequal labels, mixed graphs with ring and tree regions plus a chord and a bridge, a dense single block, parallel edges with self-loops, nested hierarchy with cross-hierarchy edges, and fixed ports. `GeometricLabelTest` checks every one of them for preserved structure, finite deterministic coordinates, labels that are adjacent to their own edge, clear of nodes and other labels, and not crossed by other edges. For the symmetric tree, the star and the ring it also compares against the unlabeled twin: labels add at most one bend per labeled edge, the per-edge detour factor (route length over endpoint distance) grows by at most 1.5, and the six-leaf star keeps at most six bends in total. The detour factor rather than the raw length is compared because reserving room for labels scales the drawing.

## Connector styles and route-only mode

`elk.geometric.tree.routing = BUS` replaces the straight parent-child connectors of a TREE component by an org chart: the parent leaves the center of the side facing its children, a bus runs halfway through the level gap, and each child is entered through a perpendicular drop into the center of its facing side. A child whose center lies within a quarter of its width from the parent's axis gets a straight drop that enters it slightly off-center instead of a jog. Tree edges of one parent share stub and bus; the first edge by identifier between a parent and a child takes the bus, parallel duplicates, edges through ports or nested endpoints, self-loops and cross-links go through the general router with the bus segments as crossings and obstacles. Labels of bus edges are placed on the child's own drop, and the tree contour reserves twice the label extent in front of a labeled child so that the half-gap drop always holds it. `DIRECT`, the default, is unchanged.

`elk.geometric.mode = FIXED` keeps every node where the caller put it, up to the uniform translation that establishes the padding, and only routes edges, places labels and computes bounds. Edges that cannot be routed around the given footprints, for example from an endpoint enclosed by another node, fall back to their direct segment instead of failing; labels that find no valid position are placed beside the longest segment of their route. Nested scopes read their own mode.

## Edge labels

Placement reserves room before routing. Tree contours extend the child of a labeled tree edge in front, by the label extent along the edge plus a slant allowance, and on one side across the edge; the visible envelope used to center a parent excludes these extensions. Radial and ring placement solve for the smallest uniform scale at which each label rectangle, beside the middle of its segment, keeps the label-node spacing from every footprint and the label-label spacing from other labels, and at which the segment between the two footprint borders is long enough for the label. Layered regions of a mixed graph receive proxy labels of the same size, and inter-region edges carry their labels onto the region proxies.

The router places labels after an edge has its shortest route. Stage one tries every segment, longest first, at the preferred position (`edgeLabels.placement` selects the center, tail or head) and then sliding outward, on both sides of the segment, and accepts the first position with a feasible candidate; among the two sides it prefers the one that crosses fewer direct lines of edges still to be routed and then the one with more clearance. With `edgeLabels.inline` the label is centered on the segment instead. Stage two builds a bounded set of carrier segments parallel to the direct line, horizontal and vertical, at increasing offsets and shifts, plus single-bend kinks at the endpoints, and takes the cheapest feasible one by route length plus one clearance per bend; obstructed legs are completed with the visibility search for a few best candidates only. Stage three routes an exterior corridor beyond the nearest side of the drawing. Multiple labels of one edge form a composite: side by side along horizontal segments, stacked along vertical ones. Placed labels, including those of already laid out nested scopes, are obstacles for the edges routed afterwards but never for their own edge.

The elkjs suite adds all directions, unequal visible subtree envelopes, forests, seeded cyclic graphs in all five modes, stable-ID input permutations, hierarchy routing, wide labels, fixed-size fallback, interactive additions, multi-ring anchors, and both worker variants. A separate parity command compares every exported coordinate with the freshly built JS bundle.

Use the CI full-reactor `clean verify` with JDK 17 and `elk-models`; selecting only a Maven subset does not resolve all Tycho bundle dependencies. GWT compilation is a required gate because JVM-only success does not establish browser compatibility.

## Design boundaries

Geometry is prioritized over compactness. Tree contours center parents over visible descendants; radial placement and rings can grow substantially for unequal footprints. Mixed placement expands or translates regions, preserving the local geometric family. Ring recognition uses a deterministic fundamental cycle basis rather than unbounded cycle enumeration. Overlapping motifs have one real owner per node; conflicting remainders are classified again.

New routes are polylines. Explicit splines, orthogonal routing, hyperedges, or excluded nodes trigger layered fallback with a logged reason. Infeasible fixed bounds produce errors, including in fallback. No new clusters, reversed edges, or duplicated articulation nodes are committed to the caller's graph.

Legacy-output changes are isolated to the MrTree/radial bounds corrections, the Eades tangent-cone correction, and removal of stale JSON junction geometry. New placement strategies are opt-in.
