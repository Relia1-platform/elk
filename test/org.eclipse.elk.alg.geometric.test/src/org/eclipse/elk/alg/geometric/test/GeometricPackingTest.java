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
import org.eclipse.elk.alg.geometric.options.GeometricPacking;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/** Compact component packing: decreasing height, first fit, aligned tops, order independence. */
public class GeometricPackingTest {
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

    /** Isolated boxes of decreasing height in a scrambled model order, each its own component. */
    private static ElkNode boxes(final boolean compact) {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(CoreOptions.ALGORITHM, "org.eclipse.elk.geometric");
        graph.setProperty(CoreOptions.SPACING_COMPONENT_COMPONENT, 20.0);
        graph.setProperty(GeometricOptions.PACKING, compact ? GeometricPacking.COMPACT : GeometricPacking.SHELF);
        double[][] sizes = {{40, 20}, {60, 100}, {50, 60}, {40, 80}, {30, 40}, {70, 60}, {40, 20}};
        for (int i = 0; i < sizes.length; i++) {
            ElkNode node = ElkGraphUtil.createNode(graph);
            node.setIdentifier("n" + i);
            node.setDimensions(sizes[i][0], sizes[i][1]);
        }
        return graph;
    }

    private static void assertSeparated(final ElkNode graph, final double spacing) {
        List<ElkNode> nodes = graph.getChildren();
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = i + 1; j < nodes.size(); j++) {
                ElkNode a = nodes.get(i);
                ElkNode b = nodes.get(j);
                double dx = Math.max(b.getX() - (a.getX() + a.getWidth()), a.getX() - (b.getX() + b.getWidth()));
                double dy = Math.max(b.getY() - (a.getY() + a.getHeight()), a.getY() - (b.getY() + b.getHeight()));
                assertTrue(a.getIdentifier() + " and " + b.getIdentifier(), Math.max(dx, dy) >= spacing - EPSILON);
            }
        }
    }

    @Test
    public void compactPackingSortsByHeightAlignsTopsAndFillsShelves() {
        ElkNode graph = layout(boxes(true));
        assertSeparated(graph, 20);
        ElkNode tallest = null;
        for (ElkNode node : graph.getChildren()) { if (tallest == null || node.getHeight() > tallest.getHeight()) { tallest = node; } }
        for (ElkNode node : graph.getChildren()) {
            assertTrue("tallest first: " + node.getIdentifier(), node.getX() >= tallest.getX() - EPSILON && node.getY() >= tallest.getY() - EPSILON);
        }
        // Every shelf's boxes share the shelf top, and shelf heights decrease downward.
        TreeSet<Long> tops = new TreeSet<>();
        for (ElkNode node : graph.getChildren()) { tops.add(Math.round(node.getY() * 1000)); }
        assertTrue("several shelves", tops.size() >= 2 && tops.size() < graph.getChildren().size());
        double previous = Double.POSITIVE_INFINITY;
        for (long top : tops) {
            double tallestOnShelf = 0;
            for (ElkNode node : graph.getChildren()) {
                if (Math.round(node.getY() * 1000) == top) { tallestOnShelf = Math.max(tallestOnShelf, node.getHeight()); }
            }
            assertTrue("shelves get shorter downward", tallestOnShelf <= previous + EPSILON);
            previous = tallestOnShelf;
        }
        ElkNode shelf = layout(boxes(false));
        assertTrue("compact is not larger: " + graph.getWidth() + "x" + graph.getHeight() + " vs " + shelf.getWidth() + "x" + shelf.getHeight(),
                graph.getWidth() * graph.getHeight() <= shelf.getWidth() * shelf.getHeight() + EPSILON);
    }

    @Test
    public void compactPackingDoesNotDependOnTheInputOrder() {
        ElkNode reference = layout(boxes(true));
        ElkNode reversed = boxes(true);
        List<ElkNode> children = new ArrayList<>(reversed.getChildren());
        Collections.reverse(children);
        reversed.getChildren().clear();
        reversed.getChildren().addAll(children);
        layout(reversed);
        for (ElkNode node : reference.getChildren()) {
            for (ElkNode other : reversed.getChildren()) {
                if (!other.getIdentifier().equals(node.getIdentifier())) { continue; }
                assertEquals(node.getIdentifier() + " x", node.getX(), other.getX(), EPSILON);
                assertEquals(node.getIdentifier() + " y", node.getY(), other.getY(), EPSILON);
            }
        }
    }

    @Test
    public void forestFixturePacksDenserThanShelvesAndKeepsComponentSpacing() throws Exception {
        ElkNode compact = layout(load("forest-compact"));
        assertSeparated(compact, 24);
        ElkNode shelf = load("forest-compact");
        shelf.setProperty(GeometricOptions.PACKING, GeometricPacking.SHELF);
        layout(shelf);
        assertTrue("compact " + compact.getWidth() + "x" + compact.getHeight() + " vs shelf " + shelf.getWidth() + "x" + shelf.getHeight(),
                compact.getWidth() * compact.getHeight() < shelf.getWidth() * shelf.getHeight());
        ElkNode plain = load("forest-compact");
        plain.getProperties().removeKey(GeometricOptions.PACKING);
        layout(plain);
        assertEquals("SHELF is the default", GeometricLabelTest.geometry(shelf), GeometricLabelTest.geometry(plain));
        ElkNode permuted = load("forest-compact");
        List<ElkNode> children = new ArrayList<>(permuted.getChildren());
        Collections.reverse(children);
        permuted.getChildren().clear();
        permuted.getChildren().addAll(children);
        List<ElkEdge> edges = new ArrayList<>(permuted.getContainedEdges());
        Collections.reverse(edges);
        permuted.getContainedEdges().clear();
        permuted.getContainedEdges().addAll(edges);
        assertEquals(GeometricLabelTest.geometry(compact), GeometricLabelTest.geometry(layout(permuted)));
    }
}
