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

import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.geometric.options.GeometricMode;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/** Stable relayout: previous positions decide sibling order, angular order, root, and ring orientation. */
public class GeometricRelayoutTest {
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

    /** The fixture again, carrying the positions of the laid-out graph, with an interactive relayout requested. */
    private static ElkNode relayoutInput(final String name, final ElkNode laidOut, final boolean interactive)
            throws Exception {
        ElkNode input = load(name);
        Map<String, double[]> positions = new HashMap<>();
        for (ElkNode node : laidOut.getChildren()) { positions.put(node.getIdentifier(), new double[] {node.getX(), node.getY()}); }
        for (ElkNode node : input.getChildren()) {
            double[] position = positions.get(node.getIdentifier());
            node.setLocation(position[0], position[1]);
            node.setProperty(GeometryGraph.POSITION_PROVIDED, true);
        }
        input.setProperty(CoreOptions.INTERACTIVE, interactive);
        return input;
    }

    private static Map<String, double[]> centers(final ElkNode graph) {
        Map<String, double[]> result = new HashMap<>();
        for (ElkNode node : graph.getChildren()) {
            result.put(node.getIdentifier(), new double[] {node.getX() + node.getWidth() / 2, node.getY() + node.getHeight() / 2});
        }
        return result;
    }

    /** Mean displacement of the nodes both graphs share, after removing the common translation. */
    private static double meanDisplacement(final ElkNode before, final ElkNode after) {
        Map<String, double[]> old = centers(before);
        Map<String, double[]> now = centers(after);
        double dx = 0;
        double dy = 0;
        int shared = 0;
        for (String id : old.keySet()) {
            if (!now.containsKey(id)) { continue; }
            dx += now.get(id)[0] - old.get(id)[0];
            dy += now.get(id)[1] - old.get(id)[1];
            shared++;
        }
        dx /= shared;
        dy /= shared;
        double total = 0;
        for (String id : old.keySet()) {
            if (!now.containsKey(id)) { continue; }
            total += Math.hypot(now.get(id)[0] - old.get(id)[0] - dx, now.get(id)[1] - old.get(id)[1] - dy);
        }
        return total / shared;
    }

    private static void reverseChildrenAndEdges(final ElkNode graph) {
        List<ElkNode> children = new ArrayList<>(graph.getChildren());
        Collections.reverse(children);
        graph.getChildren().clear();
        graph.getChildren().addAll(children);
        List<ElkEdge> edges = new ArrayList<>(graph.getContainedEdges());
        Collections.reverse(edges);
        graph.getContainedEdges().clear();
        graph.getContainedEdges().addAll(edges);
    }

    private static ElkNode addLeaf(final ElkNode graph, final String parentId, final String id) {
        ElkNode parent = null;
        for (ElkNode node : graph.getChildren()) { if (parentId.equals(node.getIdentifier())) { parent = node; } }
        ElkNode leaf = ElkGraphUtil.createNode(graph);
        leaf.setIdentifier(id);
        leaf.setDimensions(40, 30);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(parent, leaf);
        edge.setIdentifier(parentId + "-" + id);
        return leaf;
    }

    @Test
    public void interactiveRelayoutOfAnUnchangedGraphIsIdentical() throws Exception {
        for (String name : new String[] {"symmetric-tree", "unequal-tree", "star", "radial-asymmetric", "ring", "two-rings"}) {
            ElkNode first = layout(load(name));
            ElkNode again = layout(relayoutInput(name, first, true));
            assertEquals(name, GeometricLabelTest.geometry(first), GeometricLabelTest.geometry(again));
        }
    }

    @Test
    public void treeSiblingsKeepTheirOrderWhenTheModelOrderChanges() throws Exception {
        String name = "unequal-tree";
        ElkNode first = layout(load(name));
        ElkNode shuffled = relayoutInput(name, first, true);
        reverseChildrenAndEdges(shuffled);
        layout(shuffled);
        assertTrue("interactive relayout keeps positions: " + meanDisplacement(first, shuffled),
                meanDisplacement(first, shuffled) < EPSILON);
        ElkNode plain = relayoutInput(name, first, false);
        reverseChildrenAndEdges(plain);
        layout(plain);
        assertTrue("model order changes the drawing without interactive mode", meanDisplacement(first, plain) > 10);
    }

    @Test
    public void newLeavesAppendAndTheRestStaysPut() throws Exception {
        String name = "symmetric-tree";
        ElkNode first = layout(load(name));
        ElkNode grown = relayoutInput(name, first, true);
        addLeaf(grown, "a", "a0");
        layout(grown);
        Map<String, double[]> old = centers(first);
        Map<String, double[]> now = centers(grown);
        // The new leaf comes after a's previous children; the other subtree keeps its internal geometry.
        assertTrue("a0 after a2", now.get("a0")[0] > now.get("a2")[0]);
        assertEquals("b's subtree keeps its shape", old.get("b2")[0] - old.get("b1")[0], now.get("b2")[0] - now.get("b1")[0], EPSILON);
        assertTrue("mean displacement under one node width: " + meanDisplacement(first, grown),
                meanDisplacement(first, grown) < 40);
    }

    @Test
    public void radialChildrenKeepTheirAngularOrderAndTheRoot() throws Exception {
        String name = "radial-asymmetric";
        ElkNode first = layout(load(name));
        ElkNode shuffled = relayoutInput(name, first, true);
        reverseChildrenAndEdges(shuffled);
        layout(shuffled);
        assertTrue("interactive relayout keeps the radial drawing: " + meanDisplacement(first, shuffled),
                meanDisplacement(first, shuffled) < EPSILON);
        ElkNode plain = relayoutInput(name, first, false);
        reverseChildrenAndEdges(plain);
        layout(plain);
        assertTrue("model order changes the radial drawing without interactive mode", meanDisplacement(first, plain) > 10);
    }

    @Test
    public void ringOrientationFollowsThePreviousDrawing() throws Exception {
        String name = "ring";
        ElkNode first = layout(load(name));
        ElkNode mirrored = relayoutInput(name, first, true);
        // Mirror the previous positions: the cycle has to be traversed the other way round to fit.
        for (ElkNode node : mirrored.getChildren()) { node.setX(first.getWidth() - node.getX() - node.getWidth()); }
        ElkNode expected = relayoutInput(name, mirrored, true);
        layout(mirrored);
        Map<String, double[]> was = centers(expected);
        Map<String, double[]> now = centers(mirrored);
        double dx = 0;
        double dy = 0;
        for (String id : was.keySet()) { dx += now.get(id)[0] - was.get(id)[0]; dy += now.get(id)[1] - was.get(id)[1]; }
        dx /= was.size();
        dy /= was.size();
        for (String id : was.keySet()) {
            assertEquals(id + " x", was.get(id)[0] + dx, now.get(id)[0], 1e-6);
            assertEquals(id + " y", was.get(id)[1] + dy, now.get(id)[1], 1e-6);
        }
    }

    @Test
    public void previousCentralityBreaksRootTies() {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(CoreOptions.ALGORITHM, "org.eclipse.elk.geometric");
        graph.setProperty(GeometricOptions.MODE, GeometricMode.RADIAL);
        graph.setProperty(CoreOptions.INTERACTIVE, true);
        // Two equally connected hubs; the previous drawing was centered on the second one.
        ElkNode a = ElkGraphUtil.createNode(graph);
        a.setIdentifier("a");
        a.setDimensions(40, 30);
        a.setLocation(300, 100);
        ElkNode b = ElkGraphUtil.createNode(graph);
        b.setIdentifier("b");
        b.setDimensions(40, 30);
        b.setLocation(180, 100);
        for (String id : new String[] {"a1", "a2", "b1", "b2"}) {
            ElkNode leaf = ElkGraphUtil.createNode(graph);
            leaf.setIdentifier(id);
            leaf.setDimensions(40, 30);
            leaf.setLocation(id.startsWith("a") ? 400 : 50, id.endsWith("1") ? 0 : 200);
            ElkGraphUtil.createSimpleEdge(id.startsWith("a") ? a : b, leaf).setIdentifier(id);
        }
        ElkGraphUtil.createSimpleEdge(b, a).setIdentifier("ba");
        ElkGraphUtil.createSimpleEdge(a, b).setIdentifier("ab");
        layout(graph);
        assertEquals("b stays the center", graph.getWidth() / 2, b.getX() + b.getWidth() / 2, 1e-6);
        assertEquals("b stays the center", graph.getHeight() / 2, b.getY() + b.getHeight() / 2, 1e-6);
    }
}
