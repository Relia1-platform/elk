/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.test;

import static org.junit.Assert.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;

import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.math.KVectorChain;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkBendPoint;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.junit.BeforeClass;
import org.junit.Test;

/** Hanging leaves: org-chart columns beside trunks for parents with many leaf children. */
public class GeometricHangingTest {
    private static final double EPSILON = 1e-9;

    @BeforeClass
    public static void initialize() { GeometricLayoutTest.initialize(); }

    private static ElkNode load(final String name) throws Exception {
        Path path = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry", name + ".json");
        return ElkGraphJson.forGraph(Files.readString(path, StandardCharsets.UTF_8)).toElk();
    }

    private static ElkNode layout(final ElkNode graph) {
        new RecursiveGraphLayoutEngine().layout(graph, new BasicProgressMonitor());
        return graph;
    }

    private static ElkNode node(final ElkNode graph, final String id) {
        for (ElkNode child : graph.getChildren()) { if (id.equals(child.getIdentifier())) { return child; } }
        throw new AssertionError("no node " + id);
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
                    Math.abs(a.x - b.x) < EPSILON || Math.abs(a.y - b.y) < EPSILON);
        }
    }

    private static TreeSet<Long> distinct(final List<ElkNode> nodes, final boolean xAxis) {
        TreeSet<Long> values = new TreeSet<>();
        for (ElkNode node : nodes) { values.add(Math.round((xAxis ? node.getX() : node.getY()) * 1000)); }
        return values;
    }

    private static List<ElkNode> leaves(final ElkNode graph, final String prefix) {
        List<ElkNode> result = new ArrayList<>();
        for (ElkNode child : graph.getChildren()) { if (child.getIdentifier().startsWith(prefix)) { result.add(child); } }
        return result;
    }

    private static void assertNoOverlap(final ElkNode graph, final double spacing) {
        List<ElkNode> nodes = graph.getChildren();
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = i + 1; j < nodes.size(); j++) {
                ElkNode a = nodes.get(i);
                ElkNode b = nodes.get(j);
                double dx = Math.max(b.getX() - (a.getX() + a.getWidth()), a.getX() - (b.getX() + b.getWidth()));
                double dy = Math.max(b.getY() - (a.getY() + a.getHeight()), a.getY() - (b.getY() + b.getHeight()));
                assertTrue(a.getIdentifier() + " and " + b.getIdentifier() + " keep the spacing",
                        Math.max(dx, dy) >= spacing - EPSILON);
            }
        }
    }

    /** The label of a hanging edge lies on the stub: between the trunk and the leaf, in the leaf's row. */
    private static void assertLabelOnStub(final ElkEdge edge, final ElkNode leaf, final boolean horizontalStub) {
        List<KVector> points = points(edge);
        KVector stubStart = points.get(points.size() - 2);
        KVector stubEnd = points.get(points.size() - 1);
        for (ElkLabel label : edge.getLabels()) {
            double cx = label.getX() + label.getWidth() / 2;
            double cy = label.getY() + label.getHeight() / 2;
            if (horizontalStub) {
                assertTrue(edge.getIdentifier() + " label between trunk and leaf",
                        cx > Math.min(stubStart.x, stubEnd.x) - EPSILON && cx < Math.max(stubStart.x, stubEnd.x) + EPSILON);
                assertTrue(edge.getIdentifier() + " label beside the stub",
                        Math.abs(cy - stubStart.y) - label.getHeight() / 2 <= leaf.getHeight());
            } else {
                assertTrue(edge.getIdentifier() + " label between trunk and leaf",
                        cy > Math.min(stubStart.y, stubEnd.y) - EPSILON && cy < Math.max(stubStart.y, stubEnd.y) + EPSILON);
                assertTrue(edge.getIdentifier() + " label beside the stub",
                        Math.abs(cx - stubStart.x) - label.getWidth() / 2 <= leaf.getWidth());
            }
        }
    }

    @Test
    public void starLeavesHangInColumnsWithLabelsOnStubs() throws Exception {
        ElkNode graph = layout(load("hub-hanging-labels"));
        ElkNode hub = node(graph, "hub");
        List<ElkNode> leaves = leaves(graph, "d");
        assertEquals(24, leaves.size());
        // Two trunks with six rows: four leaf columns, six rows, all below the hub.
        assertEquals("leaf columns", 4, distinct(leaves, true).size());
        assertEquals("leaf rows", 6, distinct(leaves, false).size());
        for (ElkNode leaf : leaves) { assertTrue(leaf.getY() >= hub.getY() + hub.getHeight()); }
        assertNoOverlap(graph, 24);
        double busY = Double.NaN;
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertOrthogonal(edge);
            List<KVector> points = points(edge);
            assertTrue(edge.getIdentifier() + " has 4 or 5 points", points.size() == 4 || points.size() == 5);
            assertEquals("leaves the hub's bottom center", hub.getX() + hub.getWidth() / 2, points.get(0).x, EPSILON);
            if (Double.isNaN(busY)) { busY = points.get(1).y; }
            assertEquals(edge.getIdentifier() + " shares the bus", busY, points.get(1).y, EPSILON);
            ElkNode leaf = (ElkNode) edge.getTargets().get(0);
            KVector end = points.get(points.size() - 1);
            assertEquals(edge.getIdentifier() + " enters at the row center", leaf.getY() + leaf.getHeight() / 2, end.y, EPSILON);
            assertTrue(edge.getIdentifier() + " enters a side", Math.abs(end.x - leaf.getX()) < EPSILON
                    || Math.abs(end.x - leaf.getX() - leaf.getWidth()) < EPSILON);
            assertLabelOnStub(edge, leaf, true);
            KVectorChain junctions = edge.getProperty(CoreOptions.JUNCTION_POINTS);
            assertTrue(edge.getIdentifier() + " has junction points", junctions != null && !junctions.isEmpty());
        }
        // Far more compact than the radial star the same graph produces without the option.
        ElkNode radial = load("hub-hanging-labels");
        radial.getProperties().removeKey(GeometricOptions.TREE_HANGING);
        layout(radial);
        assertTrue("radial " + radial.getWidth() + "x" + radial.getHeight() + " vs hanging " + graph.getWidth() + "x"
                + graph.getHeight(), radial.getWidth() * radial.getHeight() > 4 * graph.getWidth() * graph.getHeight());
    }

    @Test
    public void hangingBlockSitsBetweenTheOtherSubtreesOnTheSharedBus() throws Exception {
        ElkNode graph = layout(load("tree-hanging-bus"));
        ElkNode r = node(graph, "r");
        ElkNode a = node(graph, "a");
        ElkNode b = node(graph, "b");
        List<ElkNode> hanging = leaves(graph, "l");
        assertEquals(10, hanging.size());
        double blockLeft = Double.POSITIVE_INFINITY;
        double blockRight = Double.NEGATIVE_INFINITY;
        for (ElkNode leaf : hanging) {
            blockLeft = Math.min(blockLeft, leaf.getX());
            blockRight = Math.max(blockRight, leaf.getX() + leaf.getWidth());
        }
        assertTrue("a is left of the block", a.getX() + a.getWidth() <= blockLeft + EPSILON);
        assertTrue("b is right of the block", b.getX() >= blockRight - EPSILON);
        assertEquals("first row shares the level of the row children", a.getY(), hanging.get(0).getY(), EPSILON);
        // Ten leaves make the squarest block with one trunk: two columns, five rows.
        assertEquals("leaf columns", 2, distinct(hanging, true).size());
        assertEquals("leaf rows", 5, distinct(hanging, false).size());
        assertNoOverlap(graph, 24);
        double busY = Double.NaN;
        for (ElkEdge edge : graph.getContainedEdges()) {
            if (edge.getSources().get(0) != r) { continue; }
            assertOrthogonal(edge);
            List<KVector> points = points(edge);
            ElkNode child = (ElkNode) edge.getTargets().get(0);
            // A hanging connector on the root's axis runs straight through the bus level without a point there.
            if (points.size() > 3) {
                if (Double.isNaN(busY)) { busY = points.get(1).y; }
                assertEquals(edge.getIdentifier() + " shares the bus", busY, points.get(1).y, EPSILON);
            }
            if (child == a || child == b) {
                assertEquals(edge.getIdentifier() + " is a bus drop", 4, points.size());
            } else {
                assertTrue(edge.getIdentifier() + " is a hanging connector", points.size() >= 3);
                assertLabelOnStub(edge, child, true);
            }
        }
        // The block's single trunk continues the root's axis: no jog at the bus.
        for (ElkEdge edge : graph.getContainedEdges()) {
            if (edge.getSources().get(0) != r || !hanging.contains(edge.getTargets().get(0))) { continue; }
            List<KVector> points = points(edge);
            assertEquals(edge.getIdentifier() + " trunk under the root", r.getX() + r.getWidth() / 2,
                    points.get(points.size() - 2).x, EPSILON);
            assertEquals(edge.getIdentifier() + " runs straight down from the root", 3, points.size());
        }
    }

    @Test
    public void horizontalTreesHangLeavesInRowsWithVerticalStubs() throws Exception {
        ElkNode graph = load("hub-hanging-labels");
        graph.setProperty(CoreOptions.DIRECTION, Direction.RIGHT);
        layout(graph);
        ElkNode hub = node(graph, "hub");
        List<ElkNode> leaves = leaves(graph, "d");
        // Trunks run horizontally: two leaf rows per trunk, and the rows of the block become columns.
        int rows = distinct(leaves, false).size();
        int columns = distinct(leaves, true).size();
        assertEquals("two rows per trunk", 0, rows % 2);
        assertTrue("rows " + rows + " x columns " + columns + " hold 24 leaves", rows * columns >= 24 && rows <= 8);
        for (ElkNode leaf : leaves) { assertTrue(leaf.getX() >= hub.getX() + hub.getWidth()); }
        assertNoOverlap(graph, 24);
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertOrthogonal(edge);
            ElkNode leaf = (ElkNode) edge.getTargets().get(0);
            KVector end = points(edge).get(points(edge).size() - 1);
            assertEquals(edge.getIdentifier() + " enters at the column center", leaf.getX() + leaf.getWidth() / 2, end.x, EPSILON);
            assertLabelOnStub(edge, leaf, false);
        }
    }

    @Test
    public void thresholdOffKeepsTheRadialStarAndStableIdsMakeItOrderInvariant() throws Exception {
        ElkNode high = load("hub-hanging-labels");
        high.setProperty(GeometricOptions.TREE_HANGING, 30);
        layout(high);
        ElkNode without = load("hub-hanging-labels");
        without.getProperties().removeKey(GeometricOptions.TREE_HANGING);
        layout(without);
        assertEquals(GeometricLabelTest.geometry(without), GeometricLabelTest.geometry(high));
        ElkNode reference = layout(load("hub-hanging-labels"));
        ElkNode permuted = load("hub-hanging-labels");
        List<ElkNode> children = new ArrayList<>(permuted.getChildren());
        Collections.reverse(children);
        permuted.getChildren().clear();
        permuted.getChildren().addAll(children);
        List<ElkEdge> edges = new ArrayList<>(permuted.getContainedEdges());
        Collections.reverse(edges);
        permuted.getContainedEdges().clear();
        permuted.getContainedEdges().addAll(edges);
        assertEquals(GeometricLabelTest.geometry(reference), GeometricLabelTest.geometry(layout(permuted)));
    }
}
