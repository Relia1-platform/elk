/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.ElkUtil;
import org.eclipse.elk.graph.ElkBendPoint;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;

/** Measures and translates complete layout results using top-left shape coordinates. */
public final class GeometryBounds {
    public double minX = Double.POSITIVE_INFINITY;
    public double minY = Double.POSITIVE_INFINITY;
    public double maxX = Double.NEGATIVE_INFINITY;
    public double maxY = Double.NEGATIVE_INFINITY;

    public void include(final double x, final double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Non-finite geometric layout coordinate");
        }
        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
    }

    public void include(final double x, final double y, final double width, final double height) {
        include(x, y);
        include(x + width, y + height);
    }

    public static GeometryBounds measure(final ElkNode graph, final boolean edges) {
        GeometryBounds bounds = new GeometryBounds();
        GeometryGraph structural = new GeometryGraph(graph, false);
        for (GeometryGraph.Vertex vertex : structural.vertices) {
            bounds.include(vertex.x + vertex.left, vertex.y + vertex.top);
            bounds.include(vertex.x + vertex.right, vertex.y + vertex.bottom);
        }
        if (edges) {
            for (ElkEdge edge : graph.getContainedEdges()) {
                for (ElkEdgeSection section : edge.getSections()) {
                    bounds.include(section.getStartX(), section.getStartY());
                    bounds.include(section.getEndX(), section.getEndY());
                    for (ElkBendPoint point : section.getBendPoints()) {
                        bounds.include(point.getX(), point.getY());
                    }
                }
                for (ElkLabel label : edge.getLabels()) {
                    bounds.include(label.getX(), label.getY(), label.getWidth(), label.getHeight());
                }
            }
        }
        if (bounds.minX == Double.POSITIVE_INFINITY) {
            bounds.include(0, 0);
        }
        return bounds;
    }

    public static void normalize(final ElkNode graph, final ElkNode center, final boolean includeEdges) {
        normalize(graph, center, includeEdges, null);
    }

    /**
     * Normalizes and, when edges attach to the scope's own ports, reroutes them with the given
     * action so that pre-routed connectors and routing modes survive the translation.
     */
    public static void normalize(final ElkNode graph, final ElkNode center, final boolean includeEdges,
            final Runnable reroute) {
        GeometryBounds bounds = measure(graph, includeEdges);
        ElkPadding padding = graph.getProperty(CoreOptions.PADDING);
        double width = bounds.maxX - bounds.minX + padding.getHorizontal();
        double height = bounds.maxY - bounds.minY + padding.getVertical();
        double dx = padding.left - bounds.minX;
        double dy = padding.top - bounds.minY;
        if (center != null) {
            double cx = center.getX() + center.getWidth() / 2;
            double cy = center.getY() + center.getHeight() / 2;
            width = 2 * Math.max(cx - bounds.minX + padding.left, bounds.maxX - cx + padding.right);
            height = 2 * Math.max(cy - bounds.minY + padding.top, bounds.maxY - cy + padding.bottom);
            dx = width / 2 - cx;
            dy = height / 2 - cy;
        }
        if (graph.getProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE)) {
            double epsilon = 1e-6 * Math.max(1, Math.max(width, height));
            if (width > graph.getWidth() + epsilon || height > graph.getHeight() + epsilon) {
                throw new IllegalArgumentException("Geometric layout cannot fit the fixed graph size: requires "
                        + width + " x " + height);
            }
            if (center != null) {
                dx += (graph.getWidth() - width) / 2;
                dy += (graph.getHeight() - height) / 2;
            }
        } else {
            graph.setDimensions(width, height);
        }
        ElkUtil.translate(graph, dx, dy);
        boolean boundaryEndpoints = false;
        for (ElkEdge edge : graph.getContainedEdges()) {
            for (org.eclipse.elk.graph.ElkConnectableShape endpoint : edge.getSources()) {
                boundaryEndpoints |= GeometryGraph.endpointNode(endpoint) == graph;
            }
            for (org.eclipse.elk.graph.ElkConnectableShape endpoint : edge.getTargets()) {
                boundaryEndpoints |= GeometryGraph.endpointNode(endpoint) == graph;
            }
        }
        if (boundaryEndpoints) {
            // Parent ports live in the scope's frame and must not move with its children.
            // Rerouting after translation reconnects them without changing any port position.
            if (reroute != null) { reroute.run(); } else { new FixedNodeRouter(graph).route(); }
            GeometryBounds routed = measure(graph, true);
            double epsilon = 1e-6 * Math.max(1, Math.max(graph.getWidth(), graph.getHeight()));
            if (routed.minX < -epsilon || routed.minY < -epsilon
                    || routed.maxX > graph.getWidth() + epsilon || routed.maxY > graph.getHeight() + epsilon) {
                throw new IllegalArgumentException("Scope boundary ports cannot fit the geometric bounds");
            }
        }
        graph.setProperty(CoreOptions.CHILD_AREA_WIDTH, graph.getWidth() - padding.getHorizontal());
        graph.setProperty(CoreOptions.CHILD_AREA_HEIGHT, graph.getHeight() - padding.getVertical());
    }
}
