/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.test;

import static org.junit.Assert.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.eclipse.elk.alg.common.FixedNodeRouter;
import org.eclipse.elk.alg.geometric.GeometricLayoutProvider;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkBendPoint;
import org.eclipse.elk.graph.ElkConnectableShape;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Edge labels must stay readable without pathological routes. The shared label fixtures are checked
 * with invariants that do not depend on exact coordinates, and the simple tree, star and ring
 * fixtures additionally compare against their unlabeled twins.
 */
public class GeometricLabelTest {
    private static final double LABEL_NODE_SPACING = 5;
    private static final double EDGE_LABEL_SPACING = 2;

    @BeforeClass
    public static void initialize() { GeometricLayoutTest.initialize(); }

    /** An absolute rectangle with the element it belongs to. */
    private static final class Box {
        final String id;
        final double x;
        final double y;
        final double width;
        final double height;
        Box(final String id, final double x, final double y, final double width, final double height) {
            this.id = id; this.x = x; this.y = y; this.width = width; this.height = height;
        }
        double gap(final Box other) {
            double dx = Math.max(0, Math.max(x - other.x - other.width, other.x - x - width));
            double dy = Math.max(0, Math.max(y - other.y - other.height, other.y - y - height));
            boolean overlap = x < other.x + other.width && x + width > other.x
                    && y < other.y + other.height && y + height > other.y;
            return overlap ? -1 : Math.hypot(dx, dy);
        }
    }

    /** One edge with its points and labels in absolute coordinates. */
    private static final class RoutedEdge {
        final ElkEdge edge;
        final List<KVector> points = new ArrayList<>();
        final List<Box> labels = new ArrayList<>();
        RoutedEdge(final ElkEdge edge) { this.edge = edge; }
        double length() {
            double length = 0;
            for (int i = 1; i < points.size(); i++) { length += points.get(i - 1).distance(points.get(i)); }
            return length;
        }
        double direct() { return points.get(0).distance(points.get(points.size() - 1)); }
        int bends() {
            int bends = 0;
            for (int i = 1; i < points.size() - 1; i++) {
                KVector a = points.get(i - 1);
                KVector b = points.get(i);
                KVector c = points.get(i + 1);
                if (Math.abs(a.distance(b) + b.distance(c) - a.distance(c)) > 1e-6) { bends++; }
            }
            return bends;
        }
    }

    private static List<Path> labelFixtures() throws Exception {
        Path root = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry");
        try (java.util.stream.Stream<Path> paths = Files.list(root)) {
            return paths.filter(p -> p.getFileName().toString().endsWith("labels.json")).sorted()
                    .collect(Collectors.toList());
        }
    }

    private static ElkNode load(final Path fixture) throws Exception {
        return ElkGraphJson.forGraph(Files.readString(fixture, StandardCharsets.UTF_8)).toElk();
    }

    private static ElkNode layout(final ElkNode graph) {
        new RecursiveGraphLayoutEngine().layout(graph, new BasicProgressMonitor());
        return graph;
    }

    @Test
    public void sharedLabelFixturesKeepLabelsReadable() throws Exception {
        List<Path> fixtures = labelFixtures();
        assertTrue("label fixtures present", fixtures.size() >= 8);
        for (Path fixture : fixtures) {
            ElkNode graph = load(fixture);
            String name = graph.getIdentifier();
            List<String> before = signature(graph);
            layout(graph);
            assertEquals(name + ": structure", before, signature(graph));
            verifyLabels(name, graph);
            // Deterministic: a second layout of the same input reproduces every coordinate.
            ElkNode again = layout(load(fixture));
            assertEquals(name + ": determinism", geometry(graph), geometry(again));
        }
    }

    @Test
    public void labelsAddAtMostOneBendPerLabeledEdge() throws Exception {
        for (String name : new String[] {"symmetric-tree-labels", "star-labels", "ring-labels"}) {
            Path fixture = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry", name + ".json");
            ElkNode labeled = layout(load(fixture));
            ElkNode plain = load(fixture);
            int labeledEdges = 0;
            for (ElkEdge edge : plain.getContainedEdges()) {
                if (!edge.getLabels().isEmpty()) { labeledEdges++; }
                edge.getLabels().clear();
            }
            layout(plain);
            Map<String, RoutedEdge> withLabels = routes(labeled);
            Map<String, RoutedEdge> withoutLabels = routes(plain);
            int addedBends = 0;
            for (Map.Entry<String, RoutedEdge> entry : withLabels.entrySet()) {
                RoutedEdge a = entry.getValue();
                RoutedEdge b = withoutLabels.get(entry.getKey());
                addedBends += a.bends() - b.bends();
                // Reserving room can scale the drawing; the detour factor is what labels must not inflate.
                double detour = (a.length() / Math.max(1e-9, a.direct())) / (b.length() / Math.max(1e-9, b.direct()));
                assertTrue(name + ": " + entry.getKey() + " detour factor ratio " + detour, detour <= 1.5 + 1e-9);
            }
            assertTrue(name + ": added bends " + addedBends + " for " + labeledEdges + " labeled edges",
                    addedBends <= labeledEdges);
            if (name.equals("star-labels")) {
                int total = 0;
                for (RoutedEdge route : withLabels.values()) { total += route.bends(); }
                assertTrue("six-leaf star bends " + total, total <= 6);
            }
        }
    }

    @Test
    public void stableIdOrderIsInvariantToInputPermutation() throws Exception {
        Path fixture = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry", "star-labels.json");
        ElkNode reference = layout(load(fixture));
        ElkNode permuted = load(fixture);
        assertEquals("STABLE_ID", String.valueOf(permuted.getProperty(
                org.eclipse.elk.alg.geometric.options.GeometricOptions.ORDER)));
        List<ElkNode> nodes = new ArrayList<>(permuted.getChildren());
        Collections.reverse(nodes);
        permuted.getChildren().clear();
        permuted.getChildren().addAll(nodes);
        List<ElkEdge> edges = new ArrayList<>(permuted.getContainedEdges());
        Collections.reverse(edges);
        permuted.getContainedEdges().clear();
        permuted.getContainedEdges().addAll(edges);
        layout(permuted);
        assertEquals(geometry(reference), geometry(permuted));
    }

    @Test
    public void labelBesideASufficientSegmentKeepsTheDirectRoute() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode a = node(graph, 0, 0);
        ElkNode b = node(graph, 200, 0);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(a, b);
        edge.setIdentifier("ab");
        ElkLabel label = ElkGraphUtil.createLabel(edge);
        label.setDimensions(60, 16);
        new FixedNodeRouter(graph).route();
        ElkEdgeSection section = edge.getSections().get(0);
        assertEquals("own label is not an obstacle", 0, section.getBendPoints().size());
        assertEquals(section.getStartY(), section.getEndY(), 1e-9);
        verifyLabels("direct", graph);
    }

    @Test
    public void blockedSegmentUsesASmallLocalDetour() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode a = node(graph, 0, 0);
        ElkNode b = node(graph, 200, 0);
        // Two blockers leave a corridor for the edge but not for a label beside it.
        ElkNode above = ElkGraphUtil.createNode(graph);
        above.setIdentifier("above");
        above.setDimensions(60, 22);
        above.setLocation(80, -30);
        ElkNode below = ElkGraphUtil.createNode(graph);
        below.setIdentifier("below");
        below.setDimensions(60, 22);
        below.setLocation(80, 28);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(a, b);
        edge.setIdentifier("ab");
        ElkLabel label = ElkGraphUtil.createLabel(edge);
        label.setDimensions(60, 16);
        new FixedNodeRouter(graph).route();
        RoutedEdge route = routes(graph).get("ab");
        assertTrue("bends " + route.bends(), route.bends() <= 2);
        for (KVector point : route.points) {
            assertTrue("stays near the direct line: " + point, Math.abs(point.y - 10) <= 80);
        }
        assertTrue("detour factor " + route.length() / route.direct(), route.length() / route.direct() <= 1.5);
        verifyLabels("detour", graph);
    }

    @Test
    public void inlineLabelsAreCenteredOnTheirSegment() {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(CoreOptions.EDGE_LABELS_INLINE, true);
        ElkNode a = node(graph, 0, 0);
        ElkNode b = node(graph, 200, 0);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(a, b);
        ElkLabel label = ElkGraphUtil.createLabel(edge);
        label.setDimensions(60, 16);
        new FixedNodeRouter(graph).route();
        assertEquals(0, edge.getSections().get(0).getBendPoints().size());
        assertEquals(10, label.getY() + label.getHeight() / 2, 1e-9);
        assertEquals(110, label.getX() + label.getWidth() / 2, 1e-9);
    }

    @Test
    public void selfLoopWidensForItsLabel() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode a = node(graph, 0, 0);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(a, a);
        edge.setIdentifier("loop");
        ElkLabel label = ElkGraphUtil.createLabel(edge);
        label.setDimensions(120, 16);
        new FixedNodeRouter(graph).route();
        verifyLabels("loop", graph);
        RoutedEdge route = routes(graph).get("loop");
        double top = Double.POSITIVE_INFINITY;
        for (KVector point : route.points) { top = Math.min(top, point.y); }
        assertTrue("label sits above the loop", label.getY() + label.getHeight() <= top + 1e-9);
    }

    @Test
    public void laterEdgesRouteAroundPlacedLabels() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode west = node(graph, -130, 0);
        ElkNode east = node(graph, 130, 0);
        ElkNode north = node(graph, 0, -100);
        ElkNode south = node(graph, 0, 100);
        ElkEdge first = ElkGraphUtil.createSimpleEdge(north, south);
        first.setIdentifier("a");
        ElkLabel label = ElkGraphUtil.createLabel(first);
        label.setDimensions(80, 30);
        ElkEdge second = ElkGraphUtil.createSimpleEdge(west, east);
        second.setIdentifier("b");
        new GeometricLayoutProvider().layout(graph, new BasicProgressMonitor());
        verifyLabels("order", graph);
    }

    // ---------------------------------------------------------------------------------------------

    /** Every label is adjacent to its own edge, clear of nodes and other labels, and uncrossed. */
    static void verifyLabels(final String name, final ElkNode graph) {
        List<Box> nodes = new ArrayList<>();
        collectNodes(graph, 0, 0, nodes);
        Map<String, RoutedEdge> routes = routes(graph);
        List<Box> allLabels = new ArrayList<>();
        for (RoutedEdge route : routes.values()) { allLabels.addAll(route.labels); }
        for (RoutedEdge route : routes.values()) {
            for (Box label : route.labels) {
                assertTrue(name + ": " + label.id + " finite", Double.isFinite(label.x) && Double.isFinite(label.y));
                for (Box node : nodes) {
                    if (isAncestor(node.id, route.edge)) { continue; }
                    assertTrue(name + ": " + label.id + " overlaps node " + node.id,
                            label.gap(node) + 1e-6 >= LABEL_NODE_SPACING);
                }
                for (Box other : allLabels) {
                    if (other == label) { continue; }
                    assertTrue(name + ": " + label.id + " overlaps label " + other.id, label.gap(other) >= -1e-6);
                }
                assertTrue(name + ": " + label.id + " is not adjacent to its edge", adjacent(label, route));
                for (RoutedEdge other : routes.values()) {
                    if (other == route) { continue; }
                    for (int i = 1; i < other.points.size(); i++) {
                        assertFalse(name + ": " + other.edge.getIdentifier() + " crosses label " + label.id,
                                crosses(other.points.get(i - 1), other.points.get(i), label));
                    }
                }
            }
        }
    }

    private static boolean adjacent(final Box label, final RoutedEdge route) {
        double cx = label.x + label.width / 2;
        double cy = label.y + label.height / 2;
        for (int i = 1; i < route.points.size(); i++) {
            KVector a = route.points.get(i - 1);
            KVector b = route.points.get(i);
            double length = a.distance(b);
            if (length < 1e-9) { continue; }
            double ux = (b.x - a.x) / length;
            double uy = (b.y - a.y) / length;
            double projection = (cx - a.x) * ux + (cy - a.y) * uy;
            double half = (Math.abs(ux) * label.width + Math.abs(uy) * label.height) / 2;
            double normalHalf = (Math.abs(uy) * label.width + Math.abs(ux) * label.height) / 2;
            double distance = Math.abs((cx - a.x) * uy - (cy - a.y) * ux);
            if (projection >= half - 1e-6 && projection <= length - half + 1e-6
                    && distance <= normalHalf + EDGE_LABEL_SPACING + 8 + 1e-6) { return true; }
        }
        return false;
    }

    private static boolean isAncestor(final String nodeId, final ElkEdge edge) {
        for (ElkConnectableShape shape : edge.getSources()) { if (hasAncestor(shape, nodeId)) { return true; } }
        for (ElkConnectableShape shape : edge.getTargets()) { if (hasAncestor(shape, nodeId)) { return true; } }
        return false;
    }

    private static boolean hasAncestor(final ElkConnectableShape shape, final String nodeId) {
        ElkNode node = shape instanceof ElkPort ? ((ElkPort) shape).getParent() : (ElkNode) shape;
        for (ElkNode parent = node.getParent(); parent != null; parent = parent.getParent()) {
            if (nodeId != null && nodeId.equals(parent.getIdentifier())) { return true; }
        }
        return false;
    }

    private static void collectNodes(final ElkNode graph, final double ox, final double oy, final List<Box> boxes) {
        for (ElkNode child : graph.getChildren()) {
            double x = ox + child.getX();
            double y = oy + child.getY();
            boxes.add(new Box(child.getIdentifier(), x, y, child.getWidth(), child.getHeight()));
            collectNodes(child, x, y, boxes);
        }
    }

    static Map<String, RoutedEdge> routes(final ElkNode graph) {
        Map<String, RoutedEdge> result = new HashMap<>();
        collectRoutes(graph, 0, 0, result);
        return result;
    }

    private static void collectRoutes(final ElkNode graph, final double ox, final double oy,
            final Map<String, RoutedEdge> result) {
        for (ElkEdge edge : graph.getContainedEdges()) {
            RoutedEdge route = new RoutedEdge(edge);
            assertFalse("route for " + edge.getIdentifier(), edge.getSections().isEmpty());
            for (ElkEdgeSection section : edge.getSections()) {
                route.points.add(new KVector(ox + section.getStartX(), oy + section.getStartY()));
                for (ElkBendPoint bend : section.getBendPoints()) {
                    route.points.add(new KVector(ox + bend.getX(), oy + bend.getY()));
                }
                route.points.add(new KVector(ox + section.getEndX(), oy + section.getEndY()));
            }
            for (ElkLabel label : edge.getLabels()) {
                route.labels.add(new Box(label.getIdentifier(), ox + label.getX(), oy + label.getY(),
                        label.getWidth(), label.getHeight()));
            }
            result.put(edge.getIdentifier(), route);
        }
        for (ElkNode child : graph.getChildren()) {
            collectRoutes(child, ox + child.getX(), oy + child.getY(), result);
        }
    }

    private static boolean crosses(final KVector a, final KVector b, final Box box) {
        double enter = 0;
        double leave = 1;
        double[] origin = {a.x, a.y};
        double[] delta = {b.x - a.x, b.y - a.y};
        double[] low = {box.x + 1e-6, box.y + 1e-6};
        double[] high = {box.x + box.width - 1e-6, box.y + box.height - 1e-6};
        for (int axis = 0; axis < 2; axis++) {
            if (Math.abs(delta[axis]) < 1e-12) {
                if (origin[axis] <= low[axis] || origin[axis] >= high[axis]) { return false; }
            } else {
                double p = (low[axis] - origin[axis]) / delta[axis];
                double q = (high[axis] - origin[axis]) / delta[axis];
                enter = Math.max(enter, Math.min(p, q));
                leave = Math.min(leave, Math.max(p, q));
            }
        }
        return enter < leave;
    }

    /** Identifiers, endpoints and hierarchy, independent of geometry. */
    private static List<String> signature(final ElkNode graph) {
        List<String> result = new ArrayList<>();
        for (ElkNode child : graph.getChildren()) {
            result.add("node " + child.getIdentifier() + " in " + graph.getIdentifier());
            for (ElkPort port : child.getPorts()) { result.add("port " + port.getIdentifier() + " of " + child.getIdentifier()); }
            result.addAll(signature(child));
        }
        for (ElkEdge edge : graph.getContainedEdges()) {
            StringBuilder builder = new StringBuilder("edge " + edge.getIdentifier() + " in " + graph.getIdentifier());
            for (ElkConnectableShape shape : edge.getSources()) { builder.append(" from ").append(shape.getIdentifier()); }
            for (ElkConnectableShape shape : edge.getTargets()) { builder.append(" to ").append(shape.getIdentifier()); }
            for (ElkLabel label : edge.getLabels()) {
                builder.append(" label ").append(label.getIdentifier()).append(' ')
                        .append(label.getWidth()).append('x').append(label.getHeight());
            }
            result.add(builder.toString());
        }
        return result;
    }

    static List<String> geometry(final ElkNode graph) {
        List<String> result = new ArrayList<>();
        result.add("size " + graph.getWidth() + " " + graph.getHeight());
        List<ElkNode> nodes = new ArrayList<>(graph.getChildren());
        nodes.sort((a, b) -> a.getIdentifier().compareTo(b.getIdentifier()));
        for (ElkNode node : nodes) {
            result.add("node " + node.getIdentifier() + " " + node.getX() + " " + node.getY());
            result.addAll(geometry(node));
        }
        List<ElkEdge> edges = new ArrayList<>(graph.getContainedEdges());
        edges.sort((a, b) -> a.getIdentifier().compareTo(b.getIdentifier()));
        for (ElkEdge edge : edges) {
            StringBuilder builder = new StringBuilder("edge " + edge.getIdentifier());
            for (ElkEdgeSection section : edge.getSections()) {
                builder.append(' ').append(section.getStartX()).append(',').append(section.getStartY());
                for (ElkBendPoint bend : section.getBendPoints()) {
                    builder.append(' ').append(bend.getX()).append(',').append(bend.getY());
                }
                builder.append(' ').append(section.getEndX()).append(',').append(section.getEndY());
            }
            for (ElkLabel label : edge.getLabels()) {
                builder.append(" label ").append(label.getIdentifier()).append(' ')
                        .append(label.getX()).append(',').append(label.getY());
            }
            result.add(builder.toString());
        }
        return result;
    }

    private static ElkNode node(final ElkNode graph, final double x, final double y) {
        ElkNode node = ElkGraphUtil.createNode(graph);
        node.setIdentifier("n" + graph.getChildren().size());
        node.setDimensions(20, 20);
        node.setLocation(x, y);
        return node;
    }
}
