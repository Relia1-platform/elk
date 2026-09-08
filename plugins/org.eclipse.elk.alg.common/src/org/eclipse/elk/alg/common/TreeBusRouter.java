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
 *
 * Hanging leaves, placed in columns beside trunks by the tree kernel, are connected through the
 * bus, down their trunk, and across a stub into the side facing the trunk; the stub is the
 * segment that carries the label. Junction points mark where a trunk leaves the bus and where a
 * stub leaves a trunk that continues or serves both sides.
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
        return route(graph, component, direction, Collections.<Vertex, Double>emptyMap(), true);
    }

    /**
     * Routes bus connectors; hanging leaves are given by the offset of their trunk from the parent's
     * cross-axis coordinate. With {@code rowBus} false only hanging leaves get connectors and the
     * other tree edges keep their straight lines.
     */
    public static Map<ElkEdge, Connector> route(final GeometryGraph graph, final List<Vertex> component,
            final Direction direction, final Map<Vertex, Double> trunks, final boolean rowBus) {
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
            double axis = across(parent, direction);
            // Trunk coordinates of this parent's hanging leaves, and the bus span they need.
            Map<Vertex, Double> trunkOf = new HashMap<>();
            double busMin = axis;
            double busMax = axis;
            for (Vertex child : parent.children) {
                Double offset = trunks.get(child);
                if (offset == null) { continue; }
                double trunk = axis + offset;
                trunkOf.put(child, trunk);
                busMin = Math.min(busMin, trunk);
                busMax = Math.max(busMax, trunk);
            }
            for (Vertex child : parent.children) {
                String key = Math.min(parent.index, child.index) + ":" + Math.max(parent.index, child.index);
                List<ElkEdge> pair = edgesByPair.get(key);
                if (pair == null || pair.isEmpty()) { continue; }
                // The first edge by identifier takes the bus; parallel duplicates keep their own lane.
                ElkEdge edge = pair.get(0);
                Double trunk = trunkOf.get(child);
                if (trunk != null) {
                    result.put(edge, hanging(parent, child, edge, trunk, bus, busMin, busMax, trunkOf, direction));
                    continue;
                }
                if (!rowBus) { continue; }
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

    /** Parent exit, bus, trunk, stub: the connector of a hanging leaf, with its junction points. */
    private static Connector hanging(final Vertex parent, final Vertex child, final ElkEdge edge, final double trunk,
            final double bus, final double busMin, final double busMax, final Map<Vertex, Double> trunkOf,
            final Direction direction) {
        double axis = across(parent, direction);
        double childAxis = across(child, direction);
        double row = center(child, direction);
        boolean onAxis = Math.abs(trunk - axis) < 1e-9;
        List<KVector> points = new ArrayList<>();
        points.add(point(axis, exit(parent, direction), direction));
        // A trunk on the parent's axis continues the stub straight down; no collinear bus point.
        if (!onAxis) {
            points.add(point(axis, bus, direction));
            points.add(point(trunk, bus, direction));
        }
        points.add(point(trunk, row, direction));
        points.add(point(sideBorder(child, direction, trunk < childAxis), row, direction));
        KVectorChain junctions = new KVectorChain();
        // Where the trunk leaves the bus: a junction unless the bus ends there, or there is no bus.
        boolean busEnd = Math.abs(trunk - busMin) < 1e-9 || Math.abs(trunk - busMax) < 1e-9;
        if (busMax - busMin > 1e-9 && (onAxis || !busEnd)) { junctions.add(point(trunk, bus, direction)); }
        // The stub leaves a junction when the trunk continues beyond this row or serves both sides here.
        boolean shared = false;
        for (Map.Entry<Vertex, Double> other : trunkOf.entrySet()) {
            if (other.getKey() == child || Math.abs(other.getValue() - trunk) > 1e-9) { continue; }
            double otherRow = center(other.getKey(), direction);
            if ((otherRow - row) * sign(direction) > 1e-9 || Math.abs(otherRow - row) < 1e-9) { shared = true; }
        }
        if (shared) { junctions.add(point(trunk, row, direction)); }
        boolean forward = edge.getSources().get(0) == parent.node;
        if (!forward) { Collections.reverse(points); }
        edge.setProperty(CoreOptions.JUNCTION_POINTS, junctions.isEmpty() ? null : junctions);
        return new Connector(points, forward ? points.size() - 1 : 1);
    }

    /** Depth coordinate of the node center. */
    private static double center(final Vertex vertex, final Direction direction) {
        return horizontal(direction) ? vertex.node.getX() + vertex.node.getWidth() / 2
                : vertex.node.getY() + vertex.node.getHeight() / 2;
    }

    /** Cross-axis coordinate of the node border facing the lower or the higher side. */
    private static double sideBorder(final Vertex vertex, final Direction direction, final boolean lower) {
        ElkNode node = vertex.node;
        if (horizontal(direction)) { return lower ? node.getY() : node.getY() + node.getHeight(); }
        return lower ? node.getX() : node.getX() + node.getWidth();
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
