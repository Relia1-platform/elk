/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;

/**
 * Room for edge labels is reserved while nodes are placed, so that the router can keep a labeled
 * edge on its direct segment. Every labeled edge is modelled as an axis-aligned label rectangle
 * beside the middle of the straight segment between the centers of its two endpoint vertices.
 * Nothing here moves a node; the placement kernels only receive larger footprints or a larger scale.
 */
public final class EdgeLabelReservation {
    /** Minimum gap between the labels of one edge, in case the label-label spacing is zero. */
    private static final double MINIMUM_GAP = 8;

    /** Spacing options that separate labels from their edge, from nodes, and from each other. */
    public static final class Margins {
        public final double edgeLabel;
        public final double labelNode;
        public final double labelLabel;
        public final double labelGap;

        private Margins(final double edgeLabel, final double labelNode, final double labelLabel) {
            this.edgeLabel = edgeLabel;
            this.labelNode = labelNode;
            this.labelLabel = labelLabel;
            this.labelGap = Math.max(MINIMUM_GAP, labelLabel);
        }

        public static Margins of(final ElkNode graph) {
            return new Margins(finite(graph.getProperty(CoreOptions.SPACING_EDGE_LABEL), 2),
                    finite(graph.getProperty(CoreOptions.SPACING_LABEL_NODE), 5),
                    finite(graph.getProperty(CoreOptions.SPACING_LABEL_LABEL), 0));
        }

        private static double finite(final Double value, final double fallback) {
            return value == null || !Double.isFinite(value) || value < 0 ? fallback : value;
        }
    }

    /**
     * The labels of one edge between a pair of vertices. Labels of one edge follow each other along
     * the segment; further labeled edges between the same pair take the next lane across it.
     */
    public static final class Reservation {
        public final Vertex a;
        public final Vertex b;
        public final List<ElkLabel> labels = new ArrayList<>();
        public final int lane;
        /** Distance from the segment to the near side of this lane, filled in by {@link #collect}. */
        double laneOffset;

        private Reservation(final Vertex a, final Vertex b, final int lane) {
            this.a = a;
            this.b = b;
            this.lane = lane;
        }

        /** Composite width and height when the labels follow a segment with direction (ux, uy). */
        public double[] size(final double ux, final double uy, final double gap) {
            return composite(labels, Math.abs(ux) >= Math.abs(uy), gap);
        }

        public double along(final double ux, final double uy, final double gap) {
            double[] size = size(ux, uy, gap);
            return Math.abs(ux) * size[0] + Math.abs(uy) * size[1];
        }

        public double across(final double ux, final double uy, final double gap) {
            double[] size = size(ux, uy, gap);
            return Math.abs(uy) * size[0] + Math.abs(ux) * size[1];
        }
    }

    private EdgeLabelReservation() { }

    /** Labels are laid side by side along horizontal segments and stacked along vertical ones. */
    public static double[] composite(final List<ElkLabel> labels, final boolean sideBySide, final double gap) {
        double width = 0;
        double height = 0;
        int count = 0;
        for (ElkLabel label : labels) {
            double w = Math.max(0, label.getWidth());
            double h = Math.max(0, label.getHeight());
            if (sideBySide) {
                width += w;
                height = Math.max(height, h);
            } else {
                width = Math.max(width, w);
                height += h;
            }
            count++;
        }
        if (count > 1) {
            if (sideBySide) { width += (count - 1) * gap; } else { height += (count - 1) * gap; }
        }
        return new double[] {width, height};
    }

    /** One reservation per labeled edge whose endpoints both map to vertices of the component. */
    public static List<Reservation> collect(final GeometryGraph graph, final List<Vertex> component) {
        Map<String, Integer> lanes = new HashMap<>();
        List<Reservation> result = new ArrayList<>();
        // Unlabeled graphs pay nothing beyond this scan; kernels call this once per region attempt.
        List<ElkEdge> edges = new ArrayList<>();
        for (ElkEdge edge : graph.graph.getContainedEdges()) {
            if (!edge.getLabels().isEmpty()) { edges.add(edge); }
        }
        if (edges.isEmpty()) { return result; }
        boolean[] member = new boolean[graph.vertices.size()];
        for (Vertex vertex : component) { member[vertex.index] = true; }
        // Lanes follow the router's edge order, which is by identifier regardless of input order.
        Collections.sort(edges, Comparator.comparing(e -> e.getIdentifier() == null ? "" : e.getIdentifier()));
        for (ElkEdge edge : edges) {
            if (edge.getLabels().isEmpty() || edge.getSources().size() != 1 || edge.getTargets().size() != 1) {
                continue;
            }
            Vertex source = graph.endpoint(edge.getSources().get(0));
            Vertex target = graph.endpoint(edge.getTargets().get(0));
            if (source == null || target == null || source == target
                    || !member[source.index] || !member[target.index]) {
                continue;
            }
            Vertex a = source.index < target.index ? source : target;
            Vertex b = source.index < target.index ? target : source;
            String key = a.index + ":" + b.index;
            int lane = lanes.getOrDefault(key, 0);
            lanes.put(key, lane + 1);
            Reservation reservation = new Reservation(a, b, lane);
            reservation.labels.addAll(edge.getLabels());
            result.add(reservation);
        }
        return result;
    }

    /** Distance from the segment to the near side of each lane, for the given direction per pair. */
    private static void laneOffsets(final List<Reservation> reservations, final double[] ux, final double[] uy,
            final Margins margins) {
        Map<String, Double> next = new HashMap<>();
        // Reservations are in edge order, so lanes of one pair appear in increasing order.
        for (int i = 0; i < reservations.size(); i++) {
            Reservation reservation = reservations.get(i);
            String key = reservation.a.index + ":" + reservation.b.index;
            double offset = next.getOrDefault(key, margins.edgeLabel);
            reservation.laneOffset = offset;
            next.put(key, offset + reservation.across(ux[i], uy[i], margins.labelGap) + margins.labelGap);
        }
    }

    /**
     * Smallest uniform scale of the current vertex coordinates at which every label rectangle,
     * placed beside the middle of its segment, keeps its distance from all node footprints and from
     * other label rectangles. Vertex coordinates are read at unit scale and are not modified.
     * Scaling moves the label center linearly, so each separation is a linear condition on the scale.
     */
    public static double expand(final List<Reservation> reservations, final List<Vertex> component,
            final double scale, final double nodeMargin, final Margins margins) {
        if (reservations.isEmpty()) { return scale; }
        int count = reservations.size();
        double[] ux = new double[count];
        double[] uy = new double[count];
        for (int i = 0; i < count; i++) {
            Reservation reservation = reservations.get(i);
            double dx = reservation.b.x - reservation.a.x;
            double dy = reservation.b.y - reservation.a.y;
            double length = Math.hypot(dx, dy);
            ux[i] = length < 1e-12 ? 1 : dx / length;
            uy[i] = length < 1e-12 ? 0 : dy / length;
        }
        laneOffsets(reservations, ux, uy, margins);
        double[] mx = new double[count];
        double[] my = new double[count];
        double[] ox = new double[count];
        double[] oy = new double[count];
        double[] halfWidth = new double[count];
        double[] halfHeight = new double[count];
        for (int i = 0; i < count; i++) {
            Reservation reservation = reservations.get(i);
            double[] size = reservation.size(ux[i], uy[i], margins.labelGap);
            double offset = reservation.laneOffset + reservation.across(ux[i], uy[i], margins.labelGap) / 2;
            mx[i] = (reservation.a.x + reservation.b.x) / 2;
            my[i] = (reservation.a.y + reservation.b.y) / 2;
            // The router tries this side first; the reserved rectangle lies on it.
            ox[i] = uy[i] * offset;
            oy[i] = -ux[i] * offset;
            halfWidth[i] = size[0] / 2;
            halfHeight[i] = size[1] / 2;
        }
        double result = Math.max(scale, 1e-9);
        for (int i = 0; i < count; i++) {
            // The segment between the two footprint borders must hold the label along its length.
            Reservation reservation = reservations.get(i);
            double length = Math.hypot(reservation.b.x - reservation.a.x, reservation.b.y - reservation.a.y);
            if (length < 1e-12) { continue; }
            double needed = border(reservation.a, ux[i], uy[i]) + border(reservation.b, -ux[i], -uy[i])
                    + reservation.along(ux[i], uy[i], margins.labelGap) + 2 * margins.edgeLabel;
            result = Math.max(result, needed / length);
        }
        for (int pass = 0; pass < 32; pass++) {
            double previous = result;
            for (int i = 0; i < count; i++) {
                for (Vertex vertex : component) {
                    result = separate(result, mx[i] - vertex.x, my[i] - vertex.y, ox[i], oy[i],
                            halfWidth[i] + vertex.right + nodeMargin, halfWidth[i] - vertex.left + nodeMargin,
                            halfHeight[i] + vertex.bottom + nodeMargin, halfHeight[i] - vertex.top + nodeMargin);
                }
                for (int j = i + 1; j < count; j++) {
                    double width = halfWidth[i] + halfWidth[j] + margins.labelLabel;
                    double height = halfHeight[i] + halfHeight[j] + margins.labelLabel;
                    result = separate(result, mx[i] - mx[j], my[i] - my[j], ox[i] - ox[j], oy[i] - oy[j],
                            width, width, height, height);
                }
            }
            if (result <= previous) { break; }
        }
        return result;
    }

    /** Distance from the vertex center to its footprint border along the direction (ux, uy). */
    private static double border(final Vertex vertex, final double ux, final double uy) {
        double distance = Double.POSITIVE_INFINITY;
        if (ux > 1e-12) { distance = Math.min(distance, vertex.right / ux); }
        if (ux < -1e-12) { distance = Math.min(distance, vertex.left / ux); }
        if (uy > 1e-12) { distance = Math.min(distance, vertex.bottom / uy); }
        if (uy < -1e-12) { distance = Math.min(distance, vertex.top / uy); }
        return distance == Double.POSITIVE_INFINITY ? 0 : Math.max(0, distance);
    }

    /**
     * Smallest s >= scale at which the center difference (s * a + b) separates two rectangles on at
     * least one axis: to the right, to the left, below or above by the given amounts.
     */
    static double separate(final double scale, final double ax, final double ay, final double bx, final double by,
            final double right, final double left, final double bottom, final double top) {
        double[][] conditions = {{ax, bx, right}, {-ax, -bx, left}, {ay, by, bottom}, {-ay, -by, top}};
        double best = Double.POSITIVE_INFINITY;
        for (double[] condition : conditions) {
            if (condition[0] * scale + condition[1] >= condition[2] - 1e-9) { return scale; }
            if (condition[0] > 1e-12) { best = Math.min(best, (condition[2] - condition[1]) / condition[0]); }
        }
        return best == Double.POSITIVE_INFINITY ? scale : Math.max(scale, best);
    }

    /**
     * Footprint extensions for contour placement: the child of every labeled tree edge receives
     * extra room in front (toward its parent) for the label along the edge, and on one side across
     * the edge. Both arrays are indexed by vertex index and only grow.
     */
    public static void treeExtents(final GeometryGraph graph, final List<Vertex> component,
            final Direction direction, final Margins margins, final double[] front, final double[] side) {
        List<Reservation> reservations = collect(graph, component);
        if (reservations.isEmpty()) { return; }
        boolean horizontal = direction == Direction.LEFT || direction == Direction.RIGHT;
        double[] ux = new double[reservations.size()];
        double[] uy = new double[reservations.size()];
        for (int i = 0; i < reservations.size(); i++) {
            ux[i] = horizontal ? 1 : 0;
            uy[i] = horizontal ? 0 : 1;
        }
        laneOffsets(reservations, ux, uy, margins);
        for (int i = 0; i < reservations.size(); i++) {
            Reservation reservation = reservations.get(i);
            Vertex child;
            if (reservation.a.parent == reservation.b) { child = reservation.a; }
            else if (reservation.b.parent == reservation.a) { child = reservation.b; }
            else { continue; }
            double along = reservation.along(ux[i], uy[i], margins.labelGap);
            double across = reservation.laneOffset + reservation.across(ux[i], uy[i], margins.labelGap);
            // An only child hangs straight below its parent. Siblings fan out, and a label beside a
            // slanted edge extends toward the levels by up to half the larger extent (at 45 degrees).
            double slant = child.parent.children.size() > 1 ? Math.max(along, across) / 2 : 0;
            front[child.index] = Math.max(front[child.index], along + 2 * margins.edgeLabel + slant);
            side[child.index] = Math.max(side[child.index], across + margins.labelNode);
        }
    }
}
