/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.core.math.ElkMargin;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.ElkConnectableShape;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;
import org.eclipse.elk.graph.properties.IProperty;
import org.eclipse.elk.graph.properties.Property;

/** A structural view. Every original edge remains in the ELK graph, including chords and parallel edges. */
public final class GeometryGraph {
    public static final IProperty<Boolean> ROOT_HINT = new Property<>("org.eclipse.elk.geometric.root", false);

    /** Coordinates refer to the node center; footprint extents include labels and ports. */
    public static final class Vertex {
        public final ElkNode node;
        public final int index;
        public final List<Vertex> neighbors = new ArrayList<>();
        public final List<Vertex> outgoing = new ArrayList<>();
        public final List<Vertex> incoming = new ArrayList<>();
        public final List<Vertex> children = new ArrayList<>();
        public Vertex parent;
        public int depth;
        public double x;
        public double y;
        public double left;
        public double right;
        public double top;
        public double bottom;

        private Vertex(final ElkNode node, final int index) {
            this.node = node;
            this.index = index;
            x = node.getX() + node.getWidth() / 2;
            y = node.getY() + node.getHeight() / 2;
            ElkMargin margin = node.getProperty(CoreOptions.MARGINS);
            left = -node.getWidth() / 2 - margin.left;
            right = node.getWidth() / 2 + margin.right;
            top = -node.getHeight() / 2 - margin.top;
            bottom = node.getHeight() / 2 + margin.bottom;
            for (ElkLabel label : node.getLabels()) {
                include(label.getX(), label.getY(), label.getWidth(), label.getHeight());
            }
            for (ElkPort port : node.getPorts()) {
                include(port.getX(), port.getY(), port.getWidth(), port.getHeight());
                for (ElkLabel label : port.getLabels()) {
                    include(port.getX() + label.getX(), port.getY() + label.getY(),
                            label.getWidth(), label.getHeight());
                }
            }
        }

        private void include(final double px, final double py, final double width, final double height) {
            left = Math.min(left, px - node.getWidth() / 2);
            right = Math.max(right, px + width - node.getWidth() / 2);
            top = Math.min(top, py - node.getHeight() / 2);
            bottom = Math.max(bottom, py + height - node.getHeight() / 2);
        }

        public double enclosingRadius() {
            double halfWidth = Math.max(Math.abs(left), Math.abs(right));
            double halfHeight = Math.max(Math.abs(top), Math.abs(bottom));
            return Math.sqrt(halfWidth * halfWidth + halfHeight * halfHeight);
        }
    }

    public final ElkNode graph;
    public final List<Vertex> vertices = new ArrayList<>();
    private final Map<ElkNode, Vertex> byNode = new HashMap<>();

    public GeometryGraph(final ElkNode graph, final boolean stableIds) {
        this.graph = graph;
        List<ElkNode> nodes = new ArrayList<>(graph.getChildren());
        if (stableIds) {
            Collections.sort(nodes, (a, b) -> identifier(a).compareTo(identifier(b)));
        }
        for (ElkNode node : nodes) {
            if (!Double.isFinite(node.getWidth()) || !Double.isFinite(node.getHeight())
                    || node.getWidth() < 0 || node.getHeight() < 0) {
                throw new IllegalArgumentException("Invalid node dimensions: " + identifier(node));
            }
            Vertex vertex = new Vertex(node, vertices.size());
            vertices.add(vertex);
            byNode.put(node, vertex);
        }
        for (ElkEdge edge : graph.getContainedEdges()) {
            if (edge.getSources().size() != 1 || edge.getTargets().size() != 1) {
                continue;
            }
            Vertex source = endpoint(edge.getSources().get(0));
            Vertex target = endpoint(edge.getTargets().get(0));
            if (source != null && target != null && source != target && !source.outgoing.contains(target)) {
                source.outgoing.add(target);
                target.incoming.add(source);
                if (!source.neighbors.contains(target)) {
                    source.neighbors.add(target);
                    target.neighbors.add(source);
                }
            }
        }
        Comparator<Vertex> order = Comparator.comparingInt(v -> v.index);
        for (Vertex vertex : vertices) {
            Collections.sort(vertex.neighbors, order);
            Collections.sort(vertex.outgoing, order);
        }
    }

    public static String identifier(final ElkNode node) {
        return node.getIdentifier() == null ? "" : node.getIdentifier();
    }

    public static ElkNode endpointNode(final ElkConnectableShape shape) {
        return shape instanceof ElkPort ? ((ElkPort) shape).getParent() : (ElkNode) shape;
    }

    /** Maps nested endpoints to their containing node at this hierarchy level. */
    public Vertex endpoint(final ElkConnectableShape shape) {
        ElkNode node = endpointNode(shape);
        while (node != null && node.getParent() != graph) {
            node = node.getParent();
        }
        return byNode.get(node);
    }

    public List<List<Vertex>> components() {
        List<List<Vertex>> result = new ArrayList<>();
        boolean[] visited = new boolean[vertices.size()];
        for (Vertex first : vertices) {
            if (visited[first.index]) {
                continue;
            }
            List<Vertex> component = new ArrayList<>();
            ArrayDeque<Vertex> queue = new ArrayDeque<>();
            queue.add(first);
            visited[first.index] = true;
            while (!queue.isEmpty()) {
                Vertex vertex = queue.remove();
                component.add(vertex);
                for (Vertex neighbor : vertex.neighbors) {
                    if (!visited[neighbor.index]) {
                        visited[neighbor.index] = true;
                        queue.add(neighbor);
                    }
                }
            }
            Collections.sort(component, Comparator.comparingInt(v -> v.index));
            result.add(component);
        }
        return result;
    }

    public Vertex chooseRoot(final List<Vertex> component) {
        Vertex hint = null;
        Vertex source = null;
        int sourceCount = 0;
        for (Vertex vertex : component) {
            if (vertex.node.getProperty(ROOT_HINT)) {
                if (hint != null) {
                    throw new IllegalArgumentException("Multiple geometric roots in one component");
                }
                hint = vertex;
            }
            if (vertex.incoming.isEmpty()) {
                source = vertex;
                sourceCount++;
            }
        }
        if (hint != null || sourceCount == 1) {
            return hint != null ? hint : source;
        }
        Vertex best = component.get(0);
        for (Vertex vertex : component) {
            if (vertex.outgoing.size() > best.outgoing.size()
                    || vertex.outgoing.size() == best.outgoing.size()
                    && (vertex.neighbors.size() > best.neighbors.size()
                    || vertex.neighbors.size() == best.neighbors.size()
                    && identifier(vertex.node).compareTo(identifier(best.node)) < 0)) {
                best = vertex;
            }
        }
        return best;
    }

    /** BFS projection, preferring outgoing edges. Original direction and multiplicity are never modified. */
    public List<Vertex> spanningTree(final List<Vertex> component, final Vertex root) {
        boolean[] allowed = new boolean[vertices.size()];
        boolean[] visited = new boolean[vertices.size()];
        for (Vertex vertex : component) {
            allowed[vertex.index] = true;
            vertex.children.clear();
            vertex.parent = null;
            vertex.depth = 0;
        }
        List<Vertex> traversal = new ArrayList<>();
        ArrayDeque<Vertex> queue = new ArrayDeque<>();
        queue.add(root);
        visited[root.index] = true;
        while (!queue.isEmpty()) {
            Vertex parent = queue.remove();
            traversal.add(parent);
            List<Vertex> candidates = new ArrayList<>(parent.outgoing);
            for (Vertex neighbor : parent.neighbors) {
                if (!candidates.contains(neighbor)) {
                    candidates.add(neighbor);
                }
            }
            for (Vertex child : candidates) {
                if (allowed[child.index] && !visited[child.index]) {
                    visited[child.index] = true;
                    child.parent = parent;
                    child.depth = parent.depth + 1;
                    parent.children.add(child);
                    queue.add(child);
                }
            }
        }
        return traversal;
    }

    public void applyPositions() {
        for (Vertex vertex : vertices) {
            vertex.node.setLocation(vertex.x - vertex.node.getWidth() / 2,
                    vertex.y - vertex.node.getHeight() / 2);
        }
    }
}
