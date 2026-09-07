/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.test;

import static org.junit.Assert.*;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.geometric.GeometricLayoutProvider;
import org.eclipse.elk.alg.geometric.TopologyAnalysis;
import org.eclipse.elk.alg.geometric.options.GeometricMetaDataProvider;
import org.eclipse.elk.alg.geometric.options.GeometricMode;
import org.eclipse.elk.alg.geometric.options.GeometricOptions;
import org.eclipse.elk.alg.layered.options.LayeredMetaDataProvider;
import org.eclipse.elk.alg.mrtree.options.MrTreeMetaDataProvider;
import org.eclipse.elk.alg.radial.options.RadialMetaDataProvider;
import org.eclipse.elk.core.RecursiveGraphLayoutEngine;
import org.eclipse.elk.core.data.LayoutMetaDataService;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkBendPoint;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.json.ElkGraphJson;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

public class GeometricLayoutTest {
    @BeforeClass
    public static void initialize() {
        LayoutMetaDataService.getInstance().registerLayoutMetaDataProviders(new CoreOptions(),
                new LayeredMetaDataProvider(), new MrTreeMetaDataProvider(), new RadialMetaDataProvider(),
                new GeometricMetaDataProvider());
    }

    @Test
    public void sharedFixturesPreserveGeometryAndData() throws Exception {
        Path fixtureRoot = Paths.get(System.getProperty("ELK_REPO"), "test", "geometry");
        List<Path> fixtures;
        try (java.util.stream.Stream<Path> paths = Files.list(fixtureRoot)) {
            fixtures = paths.filter(p -> p.toString().endsWith(".json")).sorted().collect(Collectors.toList());
        }
        assertTrue(fixtures.size() >= 9);
        Path output = Paths.get("target", "geometric-results");
        Files.createDirectories(output);
        for (Path fixture : fixtures) {
            ElkNode graph = ElkGraphJson.forGraph(Files.readString(fixture, StandardCharsets.UTF_8)).toElk();
            List<ElkNode> nodes = new ArrayList<>(graph.getChildren());
            List<ElkEdge> edges = new ArrayList<>(graph.getContainedEdges());
            new RecursiveGraphLayoutEngine().layout(graph, new BasicProgressMonitor());
            assertEquals(nodes, graph.getChildren());
            assertEquals(edges, graph.getContainedEdges());
            contained(graph);
            clear(graph, 24);
            String name = graph.getIdentifier();
            if (name.startsWith("ring")) {
                List<ElkNode> ring = graph.getChildren().stream()
                        .filter(n -> n.getIdentifier().startsWith("n")).collect(Collectors.toList());
                circular(ring, graph);
            }
            if (name.equals("two-rings")) {
                for (String prefix : new String[] {"a", "b"}) {
                    circular(graph.getChildren().stream().filter(n -> n.getIdentifier().startsWith(prefix))
                            .collect(Collectors.toList()), graph);
                }
            }
            if (name.startsWith("star") || name.equals("radial-asymmetric")) {
                ElkNode root = graph.getChildren().get(0);
                assertEquals(graph.getWidth() / 2, cx(root), epsilon(graph));
                assertEquals(graph.getHeight() / 2, cy(root), epsilon(graph));
            }
            Files.writeString(output.resolve(fixture.getFileName()), ElkGraphJson.forGraph(graph).toJson());
        }
    }

    @Test
    public void fixedSizeFailureIsTransactional() {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setDimensions(10, 10);
        graph.setProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE, true);
        graph.setProperty(GeometricOptions.MODE, GeometricMode.TREE);
        ElkNode a = ElkGraphUtil.createNode(graph);
        ElkNode b = ElkGraphUtil.createNode(graph);
        a.setDimensions(50, 50); b.setDimensions(50, 50);
        a.setLocation(7, 9); b.setLocation(11, 13);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(a, b);
        try {
            new GeometricLayoutProvider().layout(graph, new BasicProgressMonitor());
            fail("infeasible bounds must throw");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("fixed graph size"));
        }
        assertEquals(7, a.getX(), 0); assertEquals(13, b.getY(), 0);
        assertTrue(edge.getSections().isEmpty());
        assertEquals(10, graph.getWidth(), 0);
    }

    @Test
    public void tarjanDoesNotDuplicateArticulationNodes() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode[] nodes = new ElkNode[6];
        for (int i = 0; i < nodes.length; i++) { nodes[i] = ElkGraphUtil.createNode(graph); }
        for (int[] edge : new int[][] {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {3, 4}, {4, 2}, {4, 5}}) {
            ElkGraphUtil.createSimpleEdge(nodes[edge[0]], nodes[edge[1]]);
        }
        TopologyAnalysis analysis = new TopologyAnalysis(new GeometryGraph(graph, false));
        assertEquals(3, analysis.blocks.size());
        assertEquals(2, analysis.articulations.size());
        assertEquals(1, analysis.bridges.size());
        assertEquals(6, graph.getChildren().size());
    }

    private static double epsilon(final ElkNode graph) { return 1e-6 * Math.max(1, Math.max(graph.getWidth(), graph.getHeight())); }
    private static double cx(final ElkNode node) { return node.getX() + node.getWidth() / 2; }
    private static double cy(final ElkNode node) { return node.getY() + node.getHeight() / 2; }

    private static void point(final ElkNode graph, final double x, final double y) {
        double e = epsilon(graph);
        assertTrue(Double.isFinite(x) && Double.isFinite(y));
        assertTrue(graph.getIdentifier() + ": containment", x >= -e && y >= -e
                && x <= graph.getWidth() + e && y <= graph.getHeight() + e);
    }

    private static void contained(final ElkNode graph) {
        for (ElkNode node : graph.getChildren()) {
            point(graph, node.getX(), node.getY());
            point(graph, node.getX() + node.getWidth(), node.getY() + node.getHeight());
        }
        for (ElkEdge edge : graph.getContainedEdges()) {
            assertFalse(edge.getIdentifier(), edge.getSections().isEmpty());
            for (ElkEdgeSection section : edge.getSections()) {
                point(graph, section.getStartX(), section.getStartY());
                point(graph, section.getEndX(), section.getEndY());
                for (ElkBendPoint bend : section.getBendPoints()) { point(graph, bend.getX(), bend.getY()); }
            }
        }
    }

    private static void clear(final ElkNode graph, final double spacing) {
        List<ElkNode> nodes = graph.getChildren();
        for (int i = 0; i < nodes.size(); i++) {
            ElkNode a = nodes.get(i);
            for (int j = i + 1; j < nodes.size(); j++) {
                ElkNode b = nodes.get(j);
                double dx = Math.max(0, Math.max(a.getX() - b.getX() - b.getWidth(), b.getX() - a.getX() - a.getWidth()));
                double dy = Math.max(0, Math.max(a.getY() - b.getY() - b.getHeight(), b.getY() - a.getY() - a.getHeight()));
                assertTrue(a.getIdentifier() + "/" + b.getIdentifier(), Math.hypot(dx, dy) + epsilon(graph) >= spacing);
            }
        }
    }

    private static void circular(final List<ElkNode> nodes, final ElkNode graph) {
        double x = nodes.stream().mapToDouble(GeometricLayoutTest::cx).average().getAsDouble();
        double y = nodes.stream().mapToDouble(GeometricLayoutTest::cy).average().getAsDouble();
        double radius = Math.hypot(cx(nodes.get(0)) - x, cy(nodes.get(0)) - y);
        for (int i = 0; i < nodes.size(); i++) {
            ElkNode a = nodes.get(i);
            ElkNode b = nodes.get((i + 1) % nodes.size());
            assertEquals(radius, Math.hypot(cx(a) - x, cy(a) - y), epsilon(graph));
            double gap = (Math.atan2(cy(b) - y, cx(b) - x) - Math.atan2(cy(a) - y, cx(a) - x) + 4 * Math.PI) % (2 * Math.PI);
            assertEquals(2 * Math.PI / nodes.size(), gap, 1e-6);
        }
    }
}
