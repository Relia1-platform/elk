/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.math.KVectorChain;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;

/**
 * Org-chart connectors for a placed tree: the parent leaves the center of the side facing its
 * children, one bus runs halfway through the level gap, and every child is entered through a
 * perpendicular drop into the center of its facing side. Tree edges of one parent share the stub
 * and the bus, so a family of edges reads as one connector without any crossing.
 *
 * Only simple edges whose endpoints are the two nodes themselves take the bus; edges through ports
 * or nested endpoints, parallel duplicates, and cross-links are left to the general router.
 */
public final class TreeBusRouter {
    /** A child whose center is this close to the parent's axis gets a straight drop instead of a jog. */
    private static final double STRAIGHT_FRACTION = 0.25;

    /** A bus connector: its polyline and the segment that belongs to this edge alone, for labels. */
    public static final class Connector {
        public final List<KVector> points;
        /** Index of the drop segment, counting the segment that ends at point i as i. */
        public final int labelSegment;
        Connector(final List<KVector> points, final int labelSegment) {
            this.points = points;
            this.labelSegment = labelSegment;
        }
    }

    private TreeBusRouter() { }

    /**
     * Routes every tree edge of the component and returns the connectors in scope coordinates,
     * ordered like the spanning tree. Junction points are stored on the edges.
     */
    public static Map<ElkEdge, Connector> route(final GeometryGraph graph, final List<Vertex> component,
            final Direction direction) {
        Map<ElkEdge, Connector> result = new LinkedHashMap<>();
        Map<String, List<ElkEdge>> edgesByPair = new HashMap<>();
        List<ElkEdge> edges = new ArrayList<>(graph.graph.getContainedEdges());
        Collections.sort(edges, Comparator.comparing(e -> e.getIdentifier() == null ? "" : e.getIdentifier()));
        for (ElkEdge edge : edges) {
            if (edge.getSources().size() != 1 || edge.getTargets().size() != 1) { continue; }
            if (edge.getSources().get(0) instanceof ElkPort || edge.getTargets().get(0) instanceof ElkPort) { continue; }
            ElkNode source = (ElkNode) edge.getSources().get(0);
            ElkNode target = (ElkNode) edge.getTargets().get(0);
            if (source.getParent() != graph.graph || target.getParent() != graph.graph || source == target) { continue; }
            Vertex a = graph.endpoint(source);
            Vertex b = graph.endpoint(target);
            if (a == null || b == null) { continue; }
            String key = Math.min(a.index, b.index) + ":" + Math.max(a.index, b.index);
            edgesByPair.computeIfAbsent(key, k -> new ArrayList<>()).add(edge);
        }
        for (Vertex parent : component) {
            if (parent.children.isEmpty()) { continue; }
            double exitAxis = exit(parent, direction);
            double nearestEntry = Double.NaN;
            for (Vertex child : parent.children) {
                double entry = entry(child, direction);
                nearestEntry = Double.isNaN(nearestEntry) ? entry : nearer(nearestEntry, entry, exitAxis);
            }
            if (Double.isNaN(nearestEntry) || (nearestEntry - exitAxis) * sign(direction) <= 0) { continue; }
            double bus = (exitAxis + nearestEntry) / 2;
            for (Vertex child : parent.children) {
                String key = Math.min(parent.index, child.index) + ":" + Math.max(parent.index, child.index);
                List<ElkEdge> pair = edgesByPair.get(key);
                if (pair == null || pair.isEmpty()) { continue; }
                // The first edge by identifier takes the bus; parallel duplicates keep their own lane.
                ElkEdge edge = pair.get(0);
                double axis = across(parent, direction);
                double childAxis = across(child, direction);
                double extent = horizontal(direction) ? child.node.getHeight() : child.node.getWidth();
                // A child almost under its parent takes a straight drop that enters it slightly off-center.
                boolean straight = Math.abs(axis - childAxis) <= STRAIGHT_FRACTION * extent;
                List<KVector> points = new ArrayList<>();
                points.add(point(axis, exitAxis, direction));
                if (straight) {
                    points.add(point(axis, entry(child, direction), direction));
                } else {
                    points.add(point(axis, bus, direction));
                    points.add(point(childAxis, bus, direction));
                    points.add(point(childAxis, entry(child, direction), direction));
                }
                boolean forward = edge.getSources().get(0) == parent.node;
                if (!forward) { Collections.reverse(points); }
                KVectorChain junctions = new KVectorChain();
                if (!straight && parent.children.size() > 1) {
                    junctions.add(point(childAxis, bus, direction));
                }
                edge.setProperty(CoreOptions.JUNCTION_POINTS, junctions.isEmpty() ? null : junctions);
                result.put(edge, new Connector(points, forward ? points.size() - 1 : 1));
            }
        }
        return result;
    }

    private static double sign(final Direction direction) {
        return direction == Direction.UP || direction == Direction.LEFT ? -1 : 1;
    }

    private static boolean horizontal(final Direction direction) {
        return direction == Direction.LEFT || direction == Direction.RIGHT;
    }

    /** Coordinate across the tree axis, at the node center. */
    private static double across(final Vertex vertex, final Direction direction) {
        return horizontal(direction) ? vertex.node.getY() + vertex.node.getHeight() / 2
                : vertex.node.getX() + vertex.node.getWidth() / 2;
    }

    /** Depth coordinate of the parent's border on the side facing the children. */
    private static double exit(final Vertex vertex, final Direction direction) {
        ElkNode node = vertex.node;
        switch (direction) {
        case UP: return node.getY();
        case LEFT: return node.getX();
        case RIGHT: return node.getX() + node.getWidth();
        default: return node.getY() + node.getHeight();
        }
    }

    /** Depth coordinate of the child's border on the side facing the parent. */
    private static double entry(final Vertex vertex, final Direction direction) {
        ElkNode node = vertex.node;
        switch (direction) {
        case UP: return node.getY() + node.getHeight();
        case LEFT: return node.getX() + node.getWidth();
        case RIGHT: return node.getX();
        default: return node.getY();
        }
    }

    private static double nearer(final double a, final double b, final double reference) {
        return Math.abs(a - reference) <= Math.abs(b - reference) ? a : b;
    }

    private static KVector point(final double across, final double depth, final Direction direction) {
        return horizontal(direction) ? new KVector(depth, across) : new KVector(across, depth);
    }
}
