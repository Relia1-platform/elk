/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.eclipse.elk.alg.common.EdgeLabelReservation;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.properties.IProperty;
import org.eclipse.elk.graph.properties.Property;

/** A ring's only geometric degrees of freedom are its common radius and rigid rotation. */
public final class RingLayoutKernel {
    // Importers distinguish an explicitly positioned (0, 0) from an unpositioned new node.
    private static final IProperty<Boolean> POSITION_PROVIDED = new Property<>("org.eclipse.elk.json.positionProvided");
    private RingLayoutKernel() { }

    public static List<Vertex> order(final List<Vertex> component, final String anchor) {
        List<Vertex> order = new ArrayList<>(TopologyAnalysis.longestBasisCycle(component));
        if (order.isEmpty()) { order.addAll(component); }
        for (Vertex vertex : component) {
            if (order.contains(vertex)) { continue; }
            int best = 0;
            int bestScore = Integer.MIN_VALUE;
            for (int i = 0; i < order.size(); i++) {
                Vertex a = order.get(i);
                Vertex b = order.get((i + 1) % order.size());
                int score = (a.neighbors.contains(vertex) ? 2 : 0) + (b.neighbors.contains(vertex) ? 2 : 0)
                        - (a.neighbors.contains(b) ? 1 : 0);
                if (score > bestScore) { best = i + 1; bestScore = score; }
            }
            order.add(best, vertex);
        }
        if (!anchor.isEmpty()) {
            int index = -1;
            for (int i = 0; i < order.size(); i++) {
                if (GeometryGraph.identifier(order.get(i).node).equals(anchor)) { index = i; break; }
            }
            if (index < 0) { throw new IllegalArgumentException("Ring anchor does not exist: " + anchor); }
            Collections.rotate(order, -index);
        }
        return order;
    }

    public static double place(final List<Vertex> order, final double spacing, final double startAngle,
            final boolean clockwise, final boolean interactive, final double minimumRadius) {
        return place(order, spacing, startAngle, clockwise, interactive, minimumRadius, null);
    }

    /** With a graph, labels of the edges among the ring nodes reserve room by growing the radius. */
    public static double place(final List<Vertex> order, final double spacing, final double startAngle,
            final boolean clockwise, final boolean interactive, final double minimumRadius,
            final GeometryGraph graph) {
        int count = order.size();
        if (count == 0) { return 0; }
        if (count == 1) { order.get(0).x = 0; order.get(0).y = 0; return 0; }
        double step = (clockwise ? 1 : -1) * 2 * Math.PI / count;
        double radius = minimumRadius;
        // Circumscribed footprint disks make clearance independent of orientation and node size.
        for (int i = 0; i < count; i++) {
            for (int j = i + 1; j < count; j++) {
                double chord = 2 * Math.abs(Math.sin((j - i) * step / 2));
                radius = Math.max(radius,
                        (order.get(i).enclosingRadius() + order.get(j).enclosingRadius() + spacing) / chord);
            }
        }
        double angle = startAngle;
        if (interactive) {
            double cx = 0;
            double cy = 0;
            int positioned = 0;
            for (Vertex vertex : order) {
                if (hasPreviousPosition(vertex)) { cx += vertex.x; cy += vertex.y; positioned++; }
            }
            cx /= Math.max(1, positioned);
            cy /= Math.max(1, positioned);
            double real = 0;
            double imaginary = 0;
            for (int i = 0; i < count; i++) {
                if (!hasPreviousPosition(order.get(i))) { continue; }
                double oldX = order.get(i).x - cx;
                double oldY = order.get(i).y - cy;
                real += oldX * Math.cos(i * step) + oldY * Math.sin(i * step);
                imaginary += oldY * Math.cos(i * step) - oldX * Math.sin(i * step);
            }
            if (Math.hypot(real, imaginary) > 1e-8) { angle = Math.atan2(imaginary, real); }
        }
        for (int i = 0; i < count; i++) {
            order.get(i).x = Math.cos(angle + i * step);
            order.get(i).y = Math.sin(angle + i * step);
        }
        if (graph != null) {
            EdgeLabelReservation.Margins margins = EdgeLabelReservation.Margins.of(graph.graph);
            double clearance = Math.max(margins.labelNode,
                    Math.max(0, graph.graph.getProperty(CoreOptions.SPACING_EDGE_NODE)));
            radius = EdgeLabelReservation.expand(EdgeLabelReservation.collect(graph, order), order, radius,
                    clearance, margins);
        }
        for (int i = 0; i < count; i++) {
            order.get(i).x *= radius;
            order.get(i).y *= radius;
        }
        return radius;
    }

    private static boolean hasPreviousPosition(final Vertex vertex) {
        Boolean provided = vertex.node.getProperty(POSITION_PROVIDED);
        return provided != null ? provided : vertex.node.getX() != 0 || vertex.node.getY() != 0
                || vertex.node.hasProperty(CoreOptions.POSITION);
    }
}
