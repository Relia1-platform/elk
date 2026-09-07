/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.alg.layered.LayeredLayoutProvider;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.util.IElkProgressMonitor;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.util.ElkGraphUtil;

/** Layered placement of footprint proxies; original nodes, hierarchy and edges remain intact. */
final class LayeredRegionLayout {
    private LayeredRegionLayout() { }

    static void place(final List<Vertex> vertices, final double spacing, final Direction direction,
            final IElkProgressMonitor monitor) {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(CoreOptions.DIRECTION, direction);
        graph.setProperty(CoreOptions.SPACING_NODE_NODE, spacing);
        graph.setProperty(LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS, spacing);
        graph.setProperty(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED);
        Map<Vertex, ElkNode> proxies = new HashMap<>();
        for (Vertex vertex : vertices) {
            ElkNode proxy = ElkGraphUtil.createNode(graph);
            proxy.setIdentifier(vertex.node.getIdentifier());
            proxy.setDimensions(vertex.right - vertex.left, vertex.bottom - vertex.top);
            proxies.put(vertex, proxy);
        }
        for (Vertex vertex : vertices) {
            for (Vertex target : vertex.outgoing) {
                if (proxies.containsKey(target)) { ElkGraphUtil.createSimpleEdge(proxies.get(vertex), proxies.get(target)); }
            }
        }
        new LayeredLayoutProvider().layout(graph, monitor);
        for (Vertex vertex : vertices) {
            ElkNode proxy = proxies.get(vertex);
            vertex.x = proxy.getX() - vertex.left;
            vertex.y = proxy.getY() - vertex.top;
        }
        // Layered compaction can leave diagonal neighbours below the requested Euclidean clearance.
        // Expand centers uniformly; this preserves layer alignment and the ordering chosen by BK.
        double scale = 1;
        for (int i = 0; i < vertices.size(); i++) {
            for (int j = i + 1; j < vertices.size(); j++) {
                Vertex a = vertices.get(i);
                Vertex b = vertices.get(j);
                if (gap(a, b, scale) + 1e-8 >= spacing) { continue; }
                double low = scale;
                double high = scale * 2;
                while (gap(a, b, high) < spacing) {
                    high *= 2;
                    if (!Double.isFinite(high)) { throw new IllegalArgumentException("Coincident layered region nodes"); }
                }
                for (int step = 0; step < 48; step++) {
                    double middle = (low + high) / 2;
                    if (gap(a, b, middle) < spacing) { low = middle; } else { high = middle; }
                }
                scale = high;
            }
        }
        for (Vertex vertex : vertices) { vertex.x *= scale; vertex.y *= scale; }
    }

    private static double gap(final Vertex a, final Vertex b, final double scale) {
        double dx = Math.max(0, Math.max((a.x - b.x) * scale + a.left - b.right,
                (b.x - a.x) * scale + b.left - a.right));
        double dy = Math.max(0, Math.max((a.y - b.y) * scale + a.top - b.bottom,
                (b.y - a.y) * scale + b.top - a.bottom));
        return Math.hypot(dx, dy);
    }
}
