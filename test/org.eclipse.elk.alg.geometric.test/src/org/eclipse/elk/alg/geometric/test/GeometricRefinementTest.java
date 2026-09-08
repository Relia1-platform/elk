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

import org.eclipse.elk.alg.geometric.options.GeometricMode;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.alg.geometric.options.GeometricOrder;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/** The refinement stage: straightening, alignment, equal gaps and grid snapping under hard constraints. */
public class GeometricRefinementTest {
    private static final double EPSILON = 1e-9;
    private static final double SPACING = 24;
    /** Tolerance of the refinement for the spacing above: max(4, spacing / 4). */
    private static final double TOLERANCE = 6;

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

    private static ElkNode graph(final boolean refine) {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setIdentifier("refine");
        graph.setProperty(CoreOptions.ALGORITHM, "org.eclipse.elk.geometric");
        graph.setProperty(GeometricOptions.MODE, GeometricMode.FIXED);
        graph.setProperty(GeometricOptions.REFINE, refine);
        graph.setProperty(CoreOptions.SPACING_NODE_NODE, SPACING);
        return graph;
    }

    private static ElkNode box(final ElkNode graph, final String id, final double x, final double y,
            final double width, final double height) {
        ElkNode node = ElkGraphUtil.createNode(graph);
        node.setIdentifier(id);
        node.setLocation(x, y);
        node.setDimensions(width, height);
        return node;
    }

    private static ElkEdge link(final ElkNode source, final ElkNode target) {
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(source, target);
        edge.setIdentifier(source.getIdentifier() + target.getIdentifier());
        return edge;
    }

    private static ElkNode node(final ElkNode graph, final String id) {
        for (ElkNode child : graph.getChildren()) { if (id.equals(child.getIdentifier())) { return child; } }
        throw new AssertionError("no node " + id);
    }

    private static double centerX(final ElkNode node) { return node.getX() + node.getWidth() / 2; }
    private static double centerY(final ElkNode node) { return node.getY() + node.getHeight() / 2; }

    private static boolean straight(final ElkEdge edge) {
        ElkNode source = (ElkNode) edge.getSources().get(0);
        ElkNode target = (ElkNode) edge.getTargets().get(0);
        return Math.abs(centerX(source) - centerX(target)) < EPSILON || Math.abs(centerY(source) - centerY(target)) < EPSILON;
    }

    private static boolean nearlyStraight(final ElkEdge edge) {
        ElkNode source = (ElkNode) edge.getSources().get(0);
        ElkNode target = (ElkNode) edge.getTargets().get(0);
        double dx = Math.abs(centerX(source) - centerX(target));
        double dy = Math.abs(centerY(source) - centerY(target));
        return !straight(edge) && (dx <= TOLERANCE || dy <= TOLERANCE);
    }

    /** Smallest footprint separation between any two children, as the larger of the axis gaps. */
    private static double minimumSeparation(final ElkNode graph) {
        double minimum = Double.POSITIVE_INFINITY;
        List<ElkNode> nodes = graph.getChildren();
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = i + 1; j < nodes.size(); j++) {
                ElkNode a = nodes.get(i);
                ElkNode b = nodes.get(j);
                double dx = Math.max(b.getX() - (a.getX() + a.getWidth()), a.getX() - (b.getX() + b.getWidth()));
                double dy = Math.max(b.getY() - (a.getY() + a.getHeight()), a.getY() - (b.getY() + b.getHeight()));
                minimum = Math.min(minimum, Math.max(dx, dy));
            }
        }
        return minimum;
    }

    @Test
    public void nearlyStraightConnectorsBecomeStraightAndSpacingIsKept() throws Exception {
        ElkNode unrefined = load("fixed-refine-labels");
        unrefined.setProperty(GeometricOptions.REFINE, false);
        layout(unrefined);
        ElkNode refined = layout(load("fixed-refine-labels"));
        int nearlyBefore = 0;
        int straightBefore = 0;
        for (ElkEdge edge : unrefined.getContainedEdges()) {
            if (nearlyStraight(edge)) { nearlyBefore++; }
            if (straight(edge)) { straightBefore++; }
        }
        int nearlyAfter = 0;
        int straightAfter = 0;
        for (ElkEdge edge : refined.getContainedEdges()) {
            if (nearlyStraight(edge)) { nearlyAfter++; }
            if (straight(edge)) { straightAfter++; }
        }
        String counts = "straight " + straightBefore + " -> " + straightAfter + ", nearly " + nearlyBefore + " -> " + nearlyAfter;
        assertTrue("fixture has nearly straight connectors to refine: " + counts, nearlyBefore >= 2);
        assertEquals("every nearly straight connector is straightened: " + counts, 0, nearlyAfter);
        assertTrue("straight connectors gained: " + counts, straightAfter >= straightBefore + nearlyBefore);
        assertTrue("spacing kept: " + minimumSeparation(refined), minimumSeparation(refined) >= SPACING - EPSILON);
        // Every node stays close to where the caller put it, relative to the others.
        String reference = refined.getChildren().get(0).getIdentifier();
        double shiftX = centerX(node(refined, reference)) - centerX(node(unrefined, reference));
        double shiftY = centerY(node(refined, reference)) - centerY(node(unrefined, reference));
        for (ElkNode node : refined.getChildren()) {
            ElkNode original = node(unrefined, node.getIdentifier());
            double dx = centerX(node) - centerX(original) - shiftX;
            double dy = centerY(node) - centerY(original) - shiftY;
            assertTrue(node.getIdentifier() + " moved " + dx + "," + dy, Math.hypot(dx, dy) <= 3 * TOLERANCE + EPSILON);
            assertEquals(node.getIdentifier() + " snapped", node.getX(), Math.round(node.getX()), 1e-6);
            assertEquals(node.getIdentifier() + " snapped", node.getY(), Math.round(node.getY()), 1e-6);
        }
    }

    @Test
    public void straighteningMovesTheLessAnchoredEndAndRespectsOrder() {
        ElkNode graph = graph(true);
        ElkNode a = box(graph, "a", 100, 0, 40, 30);
        ElkNode b = box(graph, "b", 104, 100, 40, 30);
        ElkNode c = box(graph, "c", 100, 200, 40, 30);
        ElkNode d = box(graph, "d", 300, 100, 40, 30);
        link(a, b);
        link(b, c);
        link(b, d);
        layout(graph);
        // b is the only node off the shared axis of a and c; it joins them, d keeps its row with b.
        assertEquals(centerX(a), centerX(b), EPSILON);
        assertEquals(centerX(c), centerX(b), EPSILON);
        assertEquals(centerY(d), centerY(b), EPSILON);
        assertEquals(centerX(a) + 200, centerX(d), EPSILON);
    }

    @Test
    public void nearlyEqualGapsBecomeEqualWithoutBreakingStraightConnectors() {
        ElkNode graph = graph(true);
        ElkNode a = box(graph, "a", 0, 0, 40, 30);
        ElkNode b = box(graph, "b", 64, 0, 40, 30);
        ElkNode c = box(graph, "c", 134, 0, 40, 30);
        ElkNode d = box(graph, "d", 204, 0, 40, 30);
        layout(graph);
        double gap1 = b.getX() - (a.getX() + a.getWidth());
        double gap2 = c.getX() - (b.getX() + b.getWidth());
        double gap3 = d.getX() - (c.getX() + c.getWidth());
        assertEquals("gaps equalized", gap1, gap2, EPSILON);
        assertEquals("gaps equalized", gap2, gap3, EPSILON);
        assertEquals("row extent kept", 204 - 0, d.getX() - a.getX(), EPSILON);
        assertTrue("spacing kept", gap1 >= SPACING - EPSILON);

        // A straight connector below the middle node anchors it, so the gaps keep their difference.
        ElkNode anchored = graph(true);
        ElkNode e = box(anchored, "e", 0, 0, 40, 30);
        ElkNode f = box(anchored, "f", 64, 0, 40, 30);
        ElkNode g = box(anchored, "g", 134, 0, 40, 30);
        ElkNode h = box(anchored, "h", 204, 0, 40, 30);
        ElkNode under = box(anchored, "u", 64, 100, 40, 30);
        link(f, under);
        layout(anchored);
        assertEquals(centerX(f), centerX(under), EPSILON);
        assertEquals("first gap unchanged", 24, f.getX() - (e.getX() + e.getWidth()), EPSILON);
        assertEquals("second gap unchanged", 30, g.getX() - (f.getX() + f.getWidth()), EPSILON);
        assertEquals("third gap unchanged", 30, h.getX() - (g.getX() + g.getWidth()), EPSILON);
    }

    @Test
    public void snappingKeepsAlignedNodesTogetherAndNeverViolatesSpacing() {
        ElkNode graph = graph(true);
        ElkNode a = box(graph, "a", 10.4, 0.3, 40, 30);
        ElkNode b = box(graph, "b", 10.4, 100.3, 40, 30);
        ElkNode c = box(graph, "c", 74.4, 0.3, 40, 30);
        layout(graph);
        for (ElkNode node : graph.getChildren()) {
            assertEquals(node.getIdentifier() + " x on grid", node.getX(), Math.round(node.getX()), 1e-6);
            assertEquals(node.getIdentifier() + " y on grid", node.getY(), Math.round(node.getY()), 1e-6);
        }
        assertEquals("column stays aligned", a.getX(), b.getX(), EPSILON);
        assertEquals("row stays aligned", a.getY(), c.getY(), EPSILON);
        assertTrue(minimumSeparation(graph) >= SPACING - EPSILON);

        // Coarser grid: the snap that would bring two nodes below the spacing is refused.
        ElkNode tight = graph(true);
        tight.setProperty(GeometricOptions.REFINE_GRID, 8.0);
        ElkNode p = box(tight, "p", 0, 0, 40, 30);
        ElkNode q = box(tight, "q", 65, 0, 40, 30);
        layout(tight);
        assertTrue("spacing kept", q.getX() - (p.getX() + p.getWidth()) >= SPACING - EPSILON);
        assertEquals("q snapped relative to p", 64, q.getX() - p.getX(), EPSILON);
        assertEquals("p on the grid in the final frame", p.getX(), Math.round(p.getX() / 8) * 8, EPSILON);
        assertEquals("q on the grid in the final frame", q.getX(), Math.round(q.getX() / 8) * 8, EPSILON);
    }

    @Test
    public void refinementIsOffByDefaultAndDeterministicUnderStableIds() throws Exception {
        ElkNode plain = layout(load("mixed-refine-labels"));
        ElkNode again = layout(load("mixed-refine-labels"));
        assertEquals(GeometricLabelTest.geometry(plain), GeometricLabelTest.geometry(again));
        ElkNode permuted = load("mixed-refine-labels");
        List<ElkNode> children = new ArrayList<>(permuted.getChildren());
        Collections.reverse(children);
        permuted.getChildren().clear();
        permuted.getChildren().addAll(children);
        List<ElkEdge> edges = new ArrayList<>(permuted.getContainedEdges());
        Collections.reverse(edges);
        permuted.getContainedEdges().clear();
        permuted.getContainedEdges().addAll(edges);
        assertEquals(GeometricOrder.STABLE_ID, permuted.getProperty(GeometricOptions.ORDER));
        assertEquals(GeometricLabelTest.geometry(plain), GeometricLabelTest.geometry(layout(permuted)));
        for (ElkNode node : plain.getChildren()) {
            assertEquals(node.getIdentifier() + " snapped", node.getX(), Math.round(node.getX()), 1e-6);
        }
        assertTrue(minimumSeparation(plain) >= SPACING - EPSILON);
        ElkNode unrefined = load("mixed-refine-labels");
        unrefined.setProperty(GeometricOptions.REFINE, false);
        layout(unrefined);
        ElkNode defaults = load("mixed-refine-labels");
        defaults.getProperties().removeKey(GeometricOptions.REFINE);
        assertFalse(defaults.getProperty(GeometricOptions.REFINE));
        assertEquals(GeometricLabelTest.geometry(unrefined), GeometricLabelTest.geometry(layout(defaults)));
    }
}
