/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.radial;

import java.util.List;
import org.eclipse.elk.alg.common.EdgeLabelReservation;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.CoreOptions;

/** Equal sectors and a single radius step for the complete rooted component. */
public final class BalancedRadialLayout {
    private BalancedRadialLayout() { }

    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final double spacing, final double minimumRadius, final double startAngle, final boolean clockwise) {
        List<Vertex> traversal = data.spanningTree(component, root);
        double[] starts = new double[data.vertices.size()];
        double[] sectors = new double[data.vertices.size()];
        sectors[root.index] = 2 * Math.PI;
        starts[root.index] = -Math.PI / Math.max(1, root.children.size());
        double sign = clockwise ? 1 : -1;
        for (Vertex vertex : traversal) {
            double angle = startAngle + sign * (starts[vertex.index] + sectors[vertex.index] / 2);
            vertex.x = vertex.depth * Math.cos(angle);
            vertex.y = vertex.depth * Math.sin(angle);
            double sector = sectors[vertex.index] / Math.max(1, vertex.children.size());
            for (int i = 0; i < vertex.children.size(); i++) {
                Vertex child = vertex.children.get(i);
                starts[child.index] = starts[vertex.index] + i * sector;
                sectors[child.index] = sector;
            }
        }
        double step = Math.max(1, minimumRadius);
        // A uniform scale preserves all level radii and angles. Enclosing disks provide a conservative clearance.
        for (int i = 0; i < component.size(); i++) {
            Vertex a = component.get(i);
            for (int j = i + 1; j < component.size(); j++) {
                Vertex b = component.get(j);
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                double distance = Math.sqrt(dx * dx + dy * dy);
                if (distance < 1e-12) {
                    throw new IllegalArgumentException("Radial sectors cannot separate coincident nodes");
                }
                step = Math.max(step, (a.enclosingRadius() + b.enclosingRadius() + spacing) / distance);
            }
        }
        // Edge labels beside the straight radial segments keep their distance from all node disks.
        EdgeLabelReservation.Margins margins = EdgeLabelReservation.Margins.of(data.graph);
        double clearance = Math.max(margins.labelNode, Math.max(0, data.graph.getProperty(CoreOptions.SPACING_EDGE_NODE)));
        step = EdgeLabelReservation.expand(EdgeLabelReservation.collect(data, component), component, step,
                clearance, margins);
        for (Vertex vertex : component) {
            vertex.x *= step;
            vertex.y *= step;
        }
    }
}
