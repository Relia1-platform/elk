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
        place(data, component, root, spacing, minimumRadius, startAngle, clockwise, false);
    }

    /**
     * Interactive placement orders the children of every vertex by their previous angle around the
     * root, within the parent's sector, so that a relayout keeps the angular order of the drawing.
     */
    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final double spacing, final double minimumRadius, final double startAngle, final boolean clockwise,
            final boolean interactive) {
        List<Vertex> traversal = data.spanningTree(component, root);
        if (interactive && GeometryGraph.hasPreviousPosition(root)) {
            double rootX = root.x;
            double rootY = root.y;
            double orientation = clockwise ? 1 : -1;
            data.orderChildren(traversal, (parent, child) -> {
                double angle = Math.atan2(child.y - rootY, child.x - rootX);
                if (parent == root) {
                    // The first sector is centered on the start angle.
                    double turn = orientation * (angle - startAngle) + Math.PI / Math.max(1, parent.children.size());
                    return turn - 2 * Math.PI * Math.floor(turn / (2 * Math.PI));
                }
                double parentAngle = Math.atan2(parent.y - rootY, parent.x - rootX);
                double relative = angle - parentAngle;
                relative -= 2 * Math.PI * Math.floor((relative + Math.PI) / (2 * Math.PI));
                return orientation * relative;
            });
        }
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
