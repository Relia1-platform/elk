/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.elk.alg.common.GeometryBounds;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.alg.common.GeometryPacking;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.alg.mrtree.BalancedTreeLayout;
import org.eclipse.elk.alg.radial.BalancedRadialLayout;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.util.IElkProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.util.ElkGraphUtil;

/** Decomposes biconnected blocks and packs each region by translation only. */
final class MixedLayoutKernel {
    private static final class Region {
        final List<Vertex> core = new ArrayList<>();
        final List<Vertex> members = new ArrayList<>();
        final boolean ring;
        ElkNode proxy;
        GeometryBounds bounds;
        Region(final List<Vertex> core, final boolean ring) { this.core.addAll(core); this.ring = ring; }
    }

    private MixedLayoutKernel() { }

    static void place(final GeometryGraph graph, final List<Vertex> component,
            final IElkProgressMonitor monitor) {
        double spacing = graph.graph.getProperty(CoreOptions.SPACING_NODE_NODE);
        double angle = graph.graph.getProperty(GeometricOptions.START_ANGLE);
        boolean clockwise = graph.graph.getProperty(GeometricOptions.CLOCKWISE);
        Direction direction = graph.graph.getProperty(CoreOptions.DIRECTION);
        Set<Vertex> allowed = new HashSet<>(component);
        Map<Vertex, Region> owner = new HashMap<>();
        List<Region> regions = new ArrayList<>();
        TopologyAnalysis topology = new TopologyAnalysis(graph);
        for (List<Vertex> block : topology.blocks) {
            if (block.size() < 3 || !allowed.contains(block.get(0))) { continue; }
            List<Vertex> remainder = new ArrayList<>();
            for (Vertex vertex : block) { if (!owner.containsKey(vertex)) { remainder.add(vertex); } }
            if (remainder.isEmpty()) { continue; }
            List<Vertex> cycle = TopologyAnalysis.longestBasisCycle(remainder);
            boolean ring = TopologyAnalysis.qualifiesAsRing(remainder, cycle);
            Region region = new Region(ring ? cycle : remainder, ring);
            regions.add(region);
            for (Vertex vertex : remainder) { owner.put(vertex, region); region.members.add(vertex); }
        }
        if (regions.isEmpty()) {
            Vertex root = graph.chooseRoot(component);
            BalancedTreeLayout.place(graph, component, root, direction, spacing);
            return;
        }
        // Multi-source BFS assigns hanging branches exactly once, including articulation nodes.
        ArrayDeque<Vertex> queue = new ArrayDeque<>();
        for (Region region : regions) { queue.addAll(region.core); }
        while (!queue.isEmpty()) {
            Vertex vertex = queue.remove();
            for (Vertex neighbor : vertex.neighbors) {
                if (allowed.contains(neighbor) && !owner.containsKey(neighbor)) {
                    Region region = owner.get(vertex);
                    owner.put(neighbor, region);
                    region.members.add(neighbor);
                    queue.add(neighbor);
                }
            }
        }
        ElkNode macro = ElkGraphUtil.createGraph();
        // Label spacing applies to the composed regions exactly as to the scope itself.
        macro.setProperty(CoreOptions.SPACING_EDGE_NODE, graph.graph.getProperty(CoreOptions.SPACING_EDGE_NODE));
        macro.setProperty(CoreOptions.SPACING_EDGE_LABEL, graph.graph.getProperty(CoreOptions.SPACING_EDGE_LABEL));
        macro.setProperty(CoreOptions.SPACING_LABEL_NODE, graph.graph.getProperty(CoreOptions.SPACING_LABEL_NODE));
        macro.setProperty(CoreOptions.SPACING_LABEL_LABEL, graph.graph.getProperty(CoreOptions.SPACING_LABEL_LABEL));
        for (Region region : regions) {
            if (region.ring) {
                placeRingBranches(graph, region, spacing, angle, clockwise);
            } else {
                LayeredRegionLayout.place(graph, region.members, spacing, direction, monitor.subTask(1));
            }
            region.bounds = GeometryPacking.bounds(region.members);
            region.proxy = ElkGraphUtil.createNode(macro);
            region.proxy.setIdentifier("region-" + region.core.get(0).index);
            region.proxy.setDimensions(region.bounds.maxX - region.bounds.minX,
                    region.bounds.maxY - region.bounds.minY);
            for (Vertex vertex : region.members) {
                if (vertex.node.getProperty(GeometryGraph.ROOT_HINT)) {
                    region.proxy.setProperty(GeometryGraph.ROOT_HINT, true);
                }
            }
        }
        Map<String, ElkEdge> connections = new HashMap<>();
        for (Vertex vertex : component) {
            for (Vertex target : vertex.outgoing) {
                Region a = owner.get(vertex);
                Region b = owner.get(target);
                if (b == null || a == b) { continue; }
                String key = a.proxy.getIdentifier() + ":" + b.proxy.getIdentifier();
                if (!connections.containsKey(key)) {
                    connections.put(key, ElkGraphUtil.createSimpleEdge(a.proxy, b.proxy));
                }
            }
        }
        // Cross-link labels reserve room between regions through proxy labels of the same size.
        for (ElkEdge edge : graph.graph.getContainedEdges()) {
            if (edge.getLabels().isEmpty() || edge.getSources().size() != 1 || edge.getTargets().size() != 1) { continue; }
            Region a = owner.get(graph.endpoint(edge.getSources().get(0)));
            Region b = owner.get(graph.endpoint(edge.getTargets().get(0)));
            if (a == null || b == null || a == b) { continue; }
            ElkEdge proxy = connections.get(a.proxy.getIdentifier() + ":" + b.proxy.getIdentifier());
            if (proxy == null) { proxy = connections.get(b.proxy.getIdentifier() + ":" + a.proxy.getIdentifier()); }
            if (proxy == null) { continue; }
            for (ElkLabel label : edge.getLabels()) {
                ElkLabel copy = ElkGraphUtil.createLabel(proxy);
                copy.setDimensions(label.getWidth(), label.getHeight());
            }
        }
        GeometryGraph structure = new GeometryGraph(macro, false);
        List<Vertex> vertices = structure.vertices;
        Vertex root = structure.chooseRoot(vertices);
        double regionSpacing = Math.max(spacing, graph.graph.getProperty(CoreOptions.SPACING_COMPONENT_COMPONENT));
        if (root.node.getProperty(GeometryGraph.ROOT_HINT)) {
            BalancedRadialLayout.place(structure, vertices, root, regionSpacing, 0, angle, clockwise);
        } else if (TopologyAnalysis.edgeCount(vertices) == vertices.size() - 1) {
            BalancedTreeLayout.place(structure, vertices, root, direction, regionSpacing);
        } else {
            List<Vertex> cycle = TopologyAnalysis.longestBasisCycle(vertices);
            if (TopologyAnalysis.qualifiesAsRing(vertices, cycle)) {
                RingLayoutKernel.place(cycle, regionSpacing, angle, clockwise, false, 0, structure);
            } else {
                LayeredRegionLayout.place(structure, vertices, regionSpacing, direction, monitor.subTask(1));
            }
        }
        structure.applyPositions();
        for (Region region : regions) {
            GeometryPacking.translate(region.members, region.proxy.getX() - region.bounds.minX,
                    region.proxy.getY() - region.bounds.minY);
        }
        monitor.log("Geometric CLUSTER: " + regions.size() + " regions, " + topology.bridges.size() + " bridges");
    }

    private static void placeRingBranches(final GeometryGraph graph, final Region region, final double spacing,
            final double angle, final boolean clockwise) {
        String anchorId = graph.graph.getProperty(GeometricOptions.RING_ANCHOR_ID);
        for (int i = 0; i < region.core.size(); i++) {
            if (GeometryGraph.identifier(region.core.get(i).node).equals(anchorId)) {
                Collections.rotate(region.core, -i);
                break;
            }
        }
        Set<Vertex> core = new HashSet<>(region.core);
        Set<Vertex> allowed = new HashSet<>(region.members);
        Set<Vertex> visited = new HashSet<>(region.core);
        Map<Vertex, List<Vertex>> branches = new HashMap<>();
        for (Vertex anchor : region.core) {
            List<Vertex> branch = new ArrayList<>();
            branch.add(anchor);
            ArrayDeque<Vertex> queue = new ArrayDeque<>();
            queue.add(anchor);
            while (!queue.isEmpty()) {
                Vertex vertex = queue.remove();
                for (Vertex neighbor : vertex.neighbors) {
                    if (allowed.contains(neighbor) && !core.contains(neighbor) && visited.add(neighbor)) {
                        branch.add(neighbor);
                        queue.add(neighbor);
                    }
                }
            }
            branches.put(anchor, branch);
        }
        Map<Vertex, double[]> local = new HashMap<>();
        for (Vertex anchor : region.core) {
            List<Vertex> branch = branches.get(anchor);
            BalancedTreeLayout.place(graph, branch, anchor, Direction.DOWN, spacing);
            // Nodes remain axis-aligned when their centers rotate into an outward sector.
            // Scale each branch uniformly to protect its footprints for any sector angle.
            double scale = 1;
            for (int i = 0; i < branch.size(); i++) {
                for (int j = i + 1; j < branch.size(); j++) {
                    Vertex a = branch.get(i);
                    Vertex b = branch.get(j);
                    scale = Math.max(scale, (a.enclosingRadius() + b.enclosingRadius() + spacing)
                            / Math.max(1e-8, Math.hypot(a.x - b.x, a.y - b.y)));
                }
            }
            for (Vertex vertex : branch) { local.put(vertex, new double[] {vertex.x * scale, vertex.y * scale}); }
        }
        double radius = 0;
        for (int attempt = 0; attempt < 80; attempt++) {
            radius = RingLayoutKernel.place(region.core, spacing, angle, clockwise, false, radius, graph);
            for (Vertex anchor : region.core) {
                double theta = Math.atan2(anchor.y, anchor.x);
                for (Vertex vertex : branches.get(anchor)) {
                    if (vertex == anchor) { continue; }
                    double[] point = local.get(vertex);
                    vertex.x = anchor.x - point[0] * Math.sin(theta) + point[1] * Math.cos(theta);
                    vertex.y = anchor.y + point[0] * Math.cos(theta) + point[1] * Math.sin(theta);
                }
            }
            boolean overlap = false;
            for (int i = 0; i < region.members.size(); i++) {
                Vertex a = region.members.get(i);
                for (int j = i + 1; j < region.members.size(); j++) {
                    Vertex b = region.members.get(j);
                    double dx = Math.max(0, Math.max(a.x + a.left - b.x - b.right,
                            b.x + b.left - a.x - a.right));
                    double dy = Math.max(0, Math.max(a.y + a.top - b.y - b.bottom,
                            b.y + b.top - a.y - a.bottom));
                    if (Math.hypot(dx, dy) + 1e-7 < spacing) {
                        overlap = true;
                    }
                }
            }
            if (!overlap) { return; }
            radius = radius * 1.2 + spacing;
        }
        throw new IllegalArgumentException("Cannot separate outward ring branches while preserving their geometry");
    }
}
