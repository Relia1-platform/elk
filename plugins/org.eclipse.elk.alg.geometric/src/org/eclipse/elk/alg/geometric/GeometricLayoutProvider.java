/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.common.FixedNodeRouter;
import org.eclipse.elk.alg.common.GeometryBounds;
import org.eclipse.elk.alg.common.GeometryClearance;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.alg.common.GeometryPacking;
import org.eclipse.elk.alg.common.LayoutRefinement;
import org.eclipse.elk.alg.common.NodeMicroLayout;
import org.eclipse.elk.alg.common.TreeBusRouter;
import org.eclipse.elk.alg.geometric.options.GeometricMode;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.alg.geometric.options.GeometricOrder;
import org.eclipse.elk.alg.geometric.options.GeometricRouting;
import org.eclipse.elk.alg.geometric.options.TreeRouting;
import org.eclipse.elk.alg.layered.LayeredLayoutProvider;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.alg.mrtree.BalancedTreeLayout;
import org.eclipse.elk.alg.radial.BalancedRadialLayout;
import org.eclipse.elk.core.AbstractLayoutProvider;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.options.EdgeRouting;
import org.eclipse.elk.core.util.IElkProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkNode;

/** Geometry-first layout of declared hierarchy and automatically detected structural regions. */
public final class GeometricLayoutProvider extends AbstractLayoutProvider {
    @Override
    public void layout(final ElkNode layoutGraph, final IElkProgressMonitor monitor) {
        monitor.begin("Geometric layout", 1);
        GeometryTransaction transaction = new GeometryTransaction(layoutGraph);
        List<ElkNode> scopes = new ArrayList<>();
        scopes.add(transaction.working);
        for (int i = 0; i < scopes.size(); i++) {
            for (ElkNode child : scopes.get(i).getChildren()) {
                String algorithm = child.getProperty(CoreOptions.ALGORITHM);
                if (!child.getChildren().isEmpty() && (algorithm == null || algorithm.isEmpty()
                        || algorithm.equals("org.eclipse.elk.geometric") || algorithm.equals("geometric"))) {
                    scopes.add(child);
                }
            }
        }
        for (int i = scopes.size() - 1; i >= 0; i--) {
            if (monitor.isCanceled()) { monitor.done(); return; }
            layoutScope(scopes.get(i), monitor.subTask(1.0f / scopes.size()));
        }
        transaction.commit();
        monitor.done();
    }

    private void layoutScope(final ElkNode scope, final IElkProgressMonitor monitor) {
        monitor.begin("Geometric scope " + GeometryGraph.identifier(scope), 5);
        if (!scope.hasProperty(CoreOptions.PADDING)) {
            scope.setProperty(CoreOptions.PADDING, GeometricOptions.PADDING.getDefault());
        }
        if (!scope.hasProperty(CoreOptions.DIRECTION)) {
            scope.setProperty(CoreOptions.DIRECTION, GeometricOptions.DIRECTION.getDefault());
        }
        if (!scope.hasProperty(CoreOptions.SPACING_EDGE_NODE)) {
            scope.setProperty(CoreOptions.SPACING_EDGE_NODE, GeometricOptions.SPACING_EDGE_NODE.getDefault());
        }
        if (!scope.hasProperty(CoreOptions.SPACING_EDGE_EDGE)) {
            scope.setProperty(CoreOptions.SPACING_EDGE_EDGE, GeometricOptions.SPACING_EDGE_EDGE.getDefault());
        }
        if (!scope.hasProperty(CoreOptions.SPACING_COMPONENT_COMPONENT)) {
            scope.setProperty(CoreOptions.SPACING_COMPONENT_COMPONENT, GeometricOptions.SPACING_COMPONENT_COMPONENT.getDefault());
        }
        scope.setProperty(CoreOptions.SPACING_NODE_NODE,
                GeometryClearance.nodeSpacing(scope, scope.getProperty(GeometricOptions.SPACING_NODE_NODE)));
        String fallback = unsupported(scope);
        if (fallback != null) {
            monitor.log("Geometric fallback to layered: " + fallback);
            boolean fixed = scope.getProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE);
            double fixedWidth = scope.getWidth();
            double fixedHeight = scope.getHeight();
            // Measure the natural fallback result before enforcing the caller's outer dimensions.
            scope.setProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE, false);
            scope.setProperty(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED);
            new LayeredLayoutProvider().layout(scope, monitor.subTask(4));
            if (fixed) {
                GeometryBounds bounds = GeometryBounds.measure(scope, true);
                double requiredWidth = Math.max(scope.getWidth(), bounds.maxX + scope.getProperty(CoreOptions.PADDING).right);
                double requiredHeight = Math.max(scope.getHeight(), bounds.maxY + scope.getProperty(CoreOptions.PADDING).bottom);
                if (requiredWidth > fixedWidth + 1e-6 || requiredHeight > fixedHeight + 1e-6) {
                    throw new IllegalArgumentException("Geometric fallback cannot fit the fixed graph size: requires "
                            + requiredWidth + " x " + requiredHeight);
                }
                scope.setDimensions(fixedWidth, fixedHeight);
                scope.setProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE, true);
            }
            monitor.done();
            return;
        }
        IElkProgressMonitor stage = monitor.subTask(1);
        stage.begin("Footprint", 1);
        if (!scope.getProperty(CoreOptions.OMIT_NODE_MICRO_LAYOUT)) { NodeMicroLayout.forGraph(scope).execute(); }
        stage.done();
        stage = monitor.subTask(1);
        stage.begin("Topology", 1);
        GeometryGraph graph = new GeometryGraph(scope,
                scope.getProperty(GeometricOptions.ORDER) == GeometricOrder.STABLE_ID);
        List<List<Vertex>> components = graph.components();
        stage.done();
        stage = monitor.subTask(1);
        stage.begin("Placement", 1);
        boolean routeOnly = scope.getProperty(GeometricOptions.MODE) == GeometricMode.FIXED;
        boolean bus = scope.getProperty(GeometricOptions.TREE_ROUTING) == TreeRouting.BUS;
        List<List<Vertex>> busComponents = new ArrayList<>();
        Map<Vertex, Double> hangingTrunks = new HashMap<>();
        ElkNode center = null;
        if (routeOnly) {
            // Positions are the caller's; only connectors, labels and bounds are computed.
            monitor.log("Geometric FIXED: " + graph.vertices.size() + " nodes kept in place");
        } else {
            double spacing = scope.getProperty(CoreOptions.SPACING_NODE_NODE);
            double angle = scope.getProperty(GeometricOptions.START_ANGLE);
            if (!Double.isFinite(spacing) || spacing < 0 || !Double.isFinite(angle)) {
                throw new IllegalArgumentException("Geometric spacing must be nonnegative and startAngle must be finite");
            }
            boolean clockwise = scope.getProperty(GeometricOptions.CLOCKWISE);
            String anchorId = scope.getProperty(GeometricOptions.RING_ANCHOR_ID);
            boolean anchorFound = anchorId.isEmpty();
            for (Vertex vertex : graph.vertices) { anchorFound |= GeometryGraph.identifier(vertex.node).equals(anchorId); }
            if (!anchorFound) { throw new IllegalArgumentException("Ring anchor does not exist: " + anchorId); }
            boolean interactive = scope.getProperty(CoreOptions.INTERACTIVE);
            for (List<Vertex> component : components) {
                Vertex root = graph.chooseRoot(component, interactive);
                GeometricMode mode = scope.getProperty(GeometricOptions.MODE);
                int hanging = Math.max(0, scope.getProperty(GeometricOptions.TREE_HANGING));
                if (mode == GeometricMode.AUTO) { mode = chooseMode(component, root, hanging); }
                monitor.log("Geometric " + mode + ": " + component.size() + " nodes, root="
                        + GeometryGraph.identifier(root.node));
                switch (mode) {
                case TREE:
                    Map<Vertex, Double> trunks = BalancedTreeLayout.place(graph, component, root,
                            scope.getProperty(CoreOptions.DIRECTION), spacing, bus, hanging, interactive);
                    hangingTrunks.putAll(trunks);
                    if (bus || !trunks.isEmpty()) { busComponents.add(component); }
                    break;
                case RADIAL:
                    BalancedRadialLayout.place(graph, component, root, spacing, 0, angle, clockwise, interactive);
                    if (components.size() == 1) { center = root.node; }
                    break;
                case RING:
                    String componentAnchor = "";
                    for (Vertex vertex : component) {
                        if (GeometryGraph.identifier(vertex.node).equals(anchorId)) { componentAnchor = anchorId; break; }
                    }
                    List<Vertex> order = RingLayoutKernel.order(component, componentAnchor);
                    RingLayoutKernel.place(order, spacing, angle, clockwise,
                            scope.getProperty(CoreOptions.INTERACTIVE) && !scope.hasProperty(GeometricOptions.START_ANGLE), 0,
                            graph);
                    break;
                default:
                    MixedLayoutKernel.place(graph, component, stage);
                    break;
                }
            }
            GeometryPacking.pack(components, scope.getProperty(CoreOptions.SPACING_COMPONENT_COMPONENT),
                    scope.getProperty(CoreOptions.ASPECT_RATIO));
            graph.applyPositions();
        }
        boolean refine = scope.getProperty(GeometricOptions.REFINE);
        if (refine) {
            // Placed components keep their component spacing; given positions promise only the node spacing.
            double nodeSpacing = Math.max(0, scope.getProperty(CoreOptions.SPACING_NODE_NODE));
            int moved = LayoutRefinement.refine(graph, components, nodeSpacing,
                    routeOnly ? nodeSpacing : Math.max(0, scope.getProperty(CoreOptions.SPACING_COMPONENT_COMPONENT)),
                    scope.getProperty(GeometricOptions.REFINE_GRID),
                    scope.getProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE));
            graph.applyPositions();
            monitor.log("Geometric refine: " + moved + " moves");
        }
        stage.done();
        stage = monitor.subTask(1);
        stage.begin("Routing", 1);
        Runnable reroute = () -> route(scope, graph, busComponents, hangingTrunks, bus, routeOnly);
        reroute.run();
        stage.done();
        stage = monitor.subTask(1);
        stage.begin("Bounds", 1);
        GeometryBounds.normalize(scope, center, true, reroute,
                refine ? Math.max(0, scope.getProperty(GeometricOptions.REFINE_GRID)) : 0);
        stage.done();
        monitor.done();
    }

    /** Bus connectors for placed trees first, then every remaining edge through the general router. */
    private void route(final ElkNode scope, final GeometryGraph graph, final List<List<Vertex>> busComponents,
            final Map<Vertex, Double> hangingTrunks, final boolean rowBus, final boolean lenient) {
        FixedNodeRouter router = new FixedNodeRouter(scope);
        router.setLenient(lenient);
        router.setOrthogonal(scope.getProperty(GeometricOptions.ROUTING) == GeometricRouting.ORTHOGONAL);
        Direction direction = scope.getProperty(CoreOptions.DIRECTION);
        for (List<Vertex> component : busComponents) {
            for (Map.Entry<ElkEdge, TreeBusRouter.Connector> entry
                    : TreeBusRouter.route(graph, component, direction, hangingTrunks, rowBus).entrySet()) {
                router.prerouted(entry.getKey(), entry.getValue().points, entry.getValue().labelSegment);
            }
        }
        router.route();
    }

    private GeometricMode chooseMode(final List<Vertex> component, final Vertex root, final int hanging) {
        if (root.node.getProperty(GeometryGraph.ROOT_HINT)) { return GeometricMode.RADIAL; }
        int edges = TopologyAnalysis.edgeCount(component);
        if (edges == component.size() - 1) {
            // A star hangs its leaves as a tree once it has more of them than the hanging threshold.
            boolean star = component.size() > 3 && root.neighbors.size() == component.size() - 1;
            boolean hangs = hanging > 0 && root.neighbors.size() > hanging;
            return star && !hangs ? GeometricMode.RADIAL : GeometricMode.TREE;
        }
        List<Vertex> cycle = TopologyAnalysis.longestBasisCycle(component);
        if (TopologyAnalysis.qualifiesAsRing(component, cycle)) { return GeometricMode.RING; }
        if (root.neighbors.size() >= Math.max(3, (component.size() + 1) / 2)) { return GeometricMode.RADIAL; }
        return GeometricMode.CLUSTER;
    }

    private String unsupported(final ElkNode scope) {
        if (scope.hasProperty(CoreOptions.EDGE_ROUTING) && scope.getProperty(CoreOptions.EDGE_ROUTING) == EdgeRouting.SPLINES) {
            return "spline routing requested";
        }
        if (scope.hasProperty(CoreOptions.EDGE_ROUTING) && scope.getProperty(CoreOptions.EDGE_ROUTING) == EdgeRouting.ORTHOGONAL) {
            return "orthogonal routing requested";
        }
        for (ElkEdge edge : scope.getContainedEdges()) {
            if (edge.getSources().size() != 1 || edge.getTargets().size() != 1) { return "hyperedge endpoints"; }
        }
        for (ElkNode node : scope.getChildren()) {
            if (node.getProperty(CoreOptions.NO_LAYOUT)) { return "excluded node constraint"; }
        }
        return null;
    }
}
