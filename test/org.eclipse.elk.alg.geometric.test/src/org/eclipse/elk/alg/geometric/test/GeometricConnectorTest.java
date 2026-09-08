/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.test;

import static org.junit.Assert.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.geometric.GeometricLayoutProvider;
import org.eclipse.elk.alg.geometric.options.GeometricMode;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.alg.geometric.options.TreeRouting;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.math.KVectorChain;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkBendPoint;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/** Org-chart bus connectors for trees and the route-only mode that keeps node positions. */
public class GeometricConnectorTest {
    @BeforeClass
    public static void initialize() { GeometricLayoutTest.initialize(); }

    private static ElkNode fixture(final String name) throws Exception {
        Path path = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry", name + ".json");
        ElkNode graph = ElkGraphJson.forGraph(Files.readString(path, StandardCharsets.UTF_8)).toElk();
        new RecursiveGraphLayoutEngine().layout(graph, new BasicProgressMonitor());
        return graph;
    }

    private static List<KVector> points(final ElkEdge edge) {
        List<KVector> result = new ArrayList<>();
        for (ElkEdgeSection section : edge.getSections()) {
            result.add(new KVector(section.getStartX(), section.getStartY()));
            for (ElkBendPoint bend : section.getBendPoints()) { result.add(new KVector(bend.getX(), bend.getY())); }
            result.add(new KVector(section.getEndX(), section.getEndY()));
        }
        return result;
    }

    private static void assertOrthogonal(final ElkEdge edge) {
        List<KVector> points = points(edge);
        for (int i = 1; i < points.size(); i++) {
            KVector a = points.get(i - 1);
            KVector b = points.get(i);
            assertTrue(edge.getIdentifier() + ": segment " + i + " is not axis-aligned",
                    Math.abs(a.x - b.x) < 1e-9 || Math.abs(a.y - b.y) < 1e-9);
        }
    }

    private static int bends(final ElkEdge edge) {
        List<KVector> points = points(edge);
        int bends = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            KVector a = points.get(i - 1);
            KVector b = points.get(i);
            KVector c = points.get(i + 1);
            if (Math.abs(a.distance(b) + b.distance(c) - a.distance(c)) > 1e-6) { bends++; }
        }
        return bends;
    }

    @Test
    public void busRoutingSharesOneBusPerParentAndKeepsLabelsOnTheDrops() throws Exception {
        ElkNode graph = fixture("symmetric-tree-bus-labels");
        Map<String, ElkNode> nodes = new HashMap<>();
        for (ElkNode node : graph.getChildren()) { nodes.put(node.getIdentifier(), node); }
        Map<String, Double> busOfParent = new HashMap<>();
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertOrthogonal(edge);
            int bends = bends(edge);
            assertTrue(edge.getIdentifier() + ": " + bends + " bends", bends == 0 || bends == 2);
            List<KVector> points = points(edge);
            ElkNode parent = (ElkNode) edge.getSources().get(0);
            ElkNode child = (ElkNode) edge.getTargets().get(0);
            assertEquals("leaves the parent's bottom center", parent.getX() + parent.getWidth() / 2, points.get(0).x, 1e-9);
            assertEquals(parent.getY() + parent.getHeight(), points.get(0).y, 1e-9);
            KVector last = points.get(points.size() - 1);
            assertTrue("enters the child's top within its width",
                    last.x >= child.getX() - 1e-9 && last.x <= child.getX() + child.getWidth() + 1e-9);
            assertEquals(child.getY(), last.y, 1e-9);
            if (bends == 2) {
                double bus = points.get(1).y;
                assertEquals(bus, points.get(2).y, 1e-9);
                Double known = busOfParent.get(parent.getIdentifier());
                if (known != null) { assertEquals("siblings share the bus", known, bus, 1e-9); }
                busOfParent.put(parent.getIdentifier(), bus);
                KVectorChain junctions = edge.getProperty(CoreOptions.JUNCTION_POINTS);
                assertNotNull(edge.getIdentifier() + ": junction", junctions);
                assertEquals(points.get(2).x, junctions.getFirst().x, 1e-9);
                assertEquals(bus, junctions.getFirst().y, 1e-9);
            }
        }
        assertEquals("every parent has a bus", 3, busOfParent.size());
        GeometricLabelTest.verifyLabels("symmetric-tree-bus-labels", graph);
        // Sibling drops carry the labels beside them: the label overlaps the drop's vertical extent.
        for (ElkEdge edge : graph.getContainedEdges()) {
            List<KVector> points = points(edge);
            KVector dropTop = points.get(points.size() - 2);
            KVector dropEnd = points.get(points.size() - 1);
            ElkLabel label = edge.getLabels().get(0);
            assertTrue(edge.getIdentifier() + ": label is not beside its drop",
                    label.getY() >= dropTop.y - 1e-6 && label.getY() + label.getHeight() <= dropEnd.y + 1e-6);
        }
        assertEquals("no crossings among bus connectors", 0, crossings(graph));
    }

    @Test
    public void busRoutingFollowsTheHorizontalDirection() throws Exception {
        ElkNode graph = fixture("unequal-tree-bus");
        int buses = 0;
        for (ElkEdge edge : graph.getContainedEdges()) {
            if (edge.getIdentifier().startsWith("x")) { continue; }
            assertOrthogonal(edge);
            List<KVector> points = points(edge);
            ElkNode parent = (ElkNode) edge.getSources().get(0);
            assertEquals("leaves the parent's east side", parent.getX() + parent.getWidth(), points.get(0).x, 1e-9);
            if (bends(edge) == 2) {
                assertEquals("the bus is vertical", points.get(1).x, points.get(2).x, 1e-9);
                buses++;
            }
        }
        assertTrue(buses >= 3);
        GeometricLabelTest.verifyLabels("unequal-tree-bus", graph);
    }

    @Test
    public void routeOnlyModeKeepsRelativePositionsAndAvoidsObstacles() throws Exception {
        Path path = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry", "fixed-routing-labels.json");
        ElkNode input = ElkGraphJson.forGraph(Files.readString(path, StandardCharsets.UTF_8)).toElk();
        Map<String, KVector> before = new HashMap<>();
        for (ElkNode node : input.getChildren()) { before.put(node.getIdentifier(), new KVector(node.getX(), node.getY())); }
        ElkNode graph = fixture("fixed-routing-labels");
        ElkNode reference = graph.getChildren().get(0);
        KVector shift = new KVector(reference.getX() - before.get(reference.getIdentifier()).x,
                reference.getY() - before.get(reference.getIdentifier()).y);
        for (ElkNode node : graph.getChildren()) {
            KVector original = before.get(node.getIdentifier());
            assertEquals(node.getIdentifier() + " moved", original.x + shift.x, node.getX(), 1e-9);
            assertEquals(node.getIdentifier() + " moved", original.y + shift.y, node.getY(), 1e-9);
        }
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertFalse(edge.getIdentifier() + ": route", edge.getSections().isEmpty());
            List<KVector> points = points(edge);
            for (ElkNode node : graph.getChildren()) {
                if (edge.getSources().contains(node) || edge.getTargets().contains(node)) { continue; }
                for (int i = 1; i < points.size(); i++) {
                    assertFalse(edge.getIdentifier() + " crosses " + node.getIdentifier(),
                            GeometricRoutingTest.crossesNode(points.get(i - 1), points.get(i), node));
                }
            }
        }
        assertTrue("the blocked edge detours around the obstacle", bends(edgeOf(graph, "ab")) >= 2);
        GeometricLabelTest.verifyLabels("fixed-routing-labels", graph);
    }

    @Test
    public void routeOnlyModeIsLenientWhenAnEndpointIsEnclosed() {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(GeometricOptions.MODE, GeometricMode.FIXED);
        ElkNode big = ElkGraphUtil.createNode(graph);
        big.setIdentifier("big");
        big.setDimensions(200, 200);
        big.setLocation(0, 0);
        ElkNode inner = ElkGraphUtil.createNode(graph);
        inner.setIdentifier("inner");
        inner.setDimensions(20, 20);
        inner.setLocation(90, 90);
        ElkNode outside = ElkGraphUtil.createNode(graph);
        outside.setIdentifier("outside");
        outside.setDimensions(40, 30);
        outside.setLocation(300, 85);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(inner, outside);
        edge.setIdentifier("escape");
        ElkLabel label = ElkGraphUtil.createLabel(edge);
        label.setDimensions(50, 14);
        new GeometricLayoutProvider().layout(graph, new BasicProgressMonitor());
        assertEquals("a direct connector is used instead of failing", 1, edge.getSections().size());
        assertTrue(edge.getSections().get(0).getBendPoints().isEmpty());
        assertTrue(Double.isFinite(label.getX()) && Double.isFinite(label.getY()));
        assertEquals("positions are kept", 90, inner.getX() - big.getX(), 1e-9);
    }

    @Test
    public void directRoutingIsTheDefaultAndUnchanged() throws Exception {
        ElkNode graph = fixture("symmetric-tree-labels");
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertEquals(edge.getIdentifier() + ": direct tree edges are straight", 0, bends(edge));
            KVectorChain junctions = edge.getProperty(CoreOptions.JUNCTION_POINTS);
            assertTrue(junctions == null || junctions.isEmpty());
        }
        assertEquals(TreeRouting.DIRECT, graph.getProperty(GeometricOptions.TREE_ROUTING));
    }

    private static ElkEdge edgeOf(final ElkNode graph, final String id) {
        for (ElkEdge edge : graph.getContainedEdges()) { if (id.equals(edge.getIdentifier())) { return edge; } }
        throw new AssertionError("missing edge " + id);
    }

    private static int crossings(final ElkNode graph) {
        List<KVector[]> segments = new ArrayList<>();
        List<ElkEdge> owners = new ArrayList<>();
        for (ElkEdge edge : graph.getContainedEdges()) {
            List<KVector> points = points(edge);
            for (int i = 1; i < points.size(); i++) {
                segments.add(new KVector[] {points.get(i - 1), points.get(i)});
                owners.add(edge);
            }
        }
        int count = 0;
        for (int i = 0; i < segments.size(); i++) {
            for (int j = i + 1; j < segments.size(); j++) {
                if (owners.get(i) == owners.get(j)) { continue; }
                KVector a = segments.get(i)[0], b = segments.get(i)[1], c = segments.get(j)[0], d = segments.get(j)[1];
                if (side(a, b, c) * side(a, b, d) < -1e-9 && side(c, d, a) * side(c, d, b) < -1e-9) { count++; }
            }
        }
        return count;
    }

    private static double side(final KVector a, final KVector b, final KVector p) {
        return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    }
}
