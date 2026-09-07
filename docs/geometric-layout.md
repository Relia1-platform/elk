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
| `FixedNodeRouter` | Adaptive visibility routing around fixed footprints, hierarchy waypoints, ports, and labels. |
| `GeometryBounds` | Complete extents, padding, root centering, fixed-size errors, and synchronized translation. |
| `GeometryTransaction` | Work on an internal copy, then transfer only geometry to the original graph objects. |

The JS-facing option reference and build commands are in the matching elkjs checkout's `docs/geometric-layout.md`. New options are declared in `Geometric.melk`; generated metadata is produced by the normal build. MrTree and radial retain their existing defaults and expose `nodePlacement = BALANCED` as an opt-in.

## Verification and acceptance

`test/geometry` contains canonical JSON fixtures shared with elkjs. `GeometricLayoutTest` verifies preservation of Java node/edge objects, bounds, spacing, centered roots, exact circles, transaction failure, and Tarjan articulation handling. It exports JVM geometry for cross-runtime comparison. `GeometricRoutingTest` independently checks fixed-port loops, labels against prior routes, and stale junction removal.

The elkjs suite adds all directions, unequal visible subtree envelopes, forests, seeded cyclic graphs in all five modes, stable-ID input permutations, hierarchy routing, wide labels, fixed-size fallback, interactive additions, multi-ring anchors, and both worker variants. A separate parity command compares every exported coordinate with the freshly built JS bundle.

Use the CI full-reactor `clean verify` with JDK 17 and `elk-models`; selecting only a Maven subset does not resolve all Tycho bundle dependencies. GWT compilation is a required gate because JVM-only success does not establish browser compatibility.

## Design boundaries

Geometry is prioritized over compactness. Tree contours center parents over visible descendants; radial placement and rings can grow substantially for unequal footprints. Mixed placement expands or translates regions, preserving the local geometric family. Ring recognition uses a deterministic fundamental cycle basis rather than unbounded cycle enumeration. Overlapping motifs have one real owner per node; conflicting remainders are classified again.

New routes are polylines. Explicit splines, orthogonal routing, hyperedges, or excluded nodes trigger layered fallback with a logged reason. Infeasible fixed bounds produce errors, including in fallback. No new clusters, reversed edges, or duplicated articulation nodes are committed to the caller's graph.

Legacy-output changes are isolated to the MrTree/radial bounds corrections, the Eades tangent-cone correction, and removal of stale JSON junction geometry. New placement strategies are opt-in.
