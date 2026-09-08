/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import org.eclipse.elk.alg.common.FixedNodeRouter;
import org.eclipse.elk.alg.geometric.GeometricLayoutProvider;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.math.KVectorChain;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortConstraints;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

/** Router tests use fixed, hand-positioned obstacles to isolate routing from the placement kernels. */
public class GeometricRoutingTest {
    @BeforeClass
    public static void initialize() { GeometricLayoutTest.initialize(); }

    @Test
    public void aLaterLabelDoesNotCoverAnEarlierRoute() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode west = node(graph, -130, 0);
        ElkNode east = node(graph, 130, 0);
        ElkNode north = node(graph, 0, -100);
        ElkNode south = node(graph, 0, 100);
        ElkEdge first = ElkGraphUtil.createSimpleEdge(west, east);
        first.setIdentifier("a");
        ElkEdge second = ElkGraphUtil.createSimpleEdge(north, south);
        second.setIdentifier("b");
        ElkLabel label = ElkGraphUtil.createLabel(second);
        label.setDimensions(80, 30);
        new FixedNodeRouter(graph).route();
        List<KVector> points = points(first.getSections().get(0));
        for (int i = 1; i < points.size(); i++) {
            assertFalse("new label covers an earlier edge", crosses(points.get(i - 1), points.get(i),
                    label.getX(), label.getY(), label.getWidth(), label.getHeight()));
        }
    }

    @Test
    public void reroutingDiscardsOldJunctionGeometry() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(node(graph, 0, 0), node(graph, 100, 0));
        KVectorChain old = new KVectorChain();
        old.add(new KVector(1000, 1000));
        edge.setProperty(CoreOptions.JUNCTION_POINTS, old);
        new GeometricLayoutProvider().layout(graph, new BasicProgressMonitor());
        KVectorChain result = edge.getProperty(CoreOptions.JUNCTION_POINTS);
        assertTrue(result == null || result.isEmpty());
        assertEquals("old property values are not mutated", 1000, old.getFirst().x, 0);
    }

    @Test
    public void westPortSelfLoopNeverCrossesTheNodeBody() {
        ElkNode graph = ElkGraphUtil.createGraph();
        ElkNode node = node(graph, 0, 0);
        node.setProperty(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_POS);
        ElkPort port = ElkGraphUtil.createPort(node);
        port.setLocation(0, 10);
        port.setProperty(CoreOptions.PORT_SIDE, PortSide.WEST);
        ElkEdge edge = ElkGraphUtil.createSimpleEdge(port, port);
        new GeometricLayoutProvider().layout(graph, new BasicProgressMonitor());
        List<KVector> points = points(edge.getSections().get(0));
        assertEquals(node.getX(), points.get(0).x, 1e-6);
        assertTrue(points.get(1).x < node.getX());
        for (int i = 1; i < points.size(); i++) {
            assertFalse(crosses(points.get(i - 1), points.get(i),
                    node.getX(), node.getY(), node.getWidth(), node.getHeight()));
        }
    }

    private static ElkNode node(final ElkNode graph, final double x, final double y) {
        ElkNode node = ElkGraphUtil.createNode(graph);
        node.setDimensions(20, 20);
        node.setLocation(x, y);
        return node;
    }

    private static List<KVector> points(final ElkEdgeSection section) {
        List<KVector> result = new ArrayList<>();
        result.add(new KVector(section.getStartX(), section.getStartY()));
        section.getBendPoints().forEach(p -> result.add(new KVector(p.getX(), p.getY())));
        result.add(new KVector(section.getEndX(), section.getEndY()));
        return result;
    }

    /** Open-interior clipping of a segment against a node's rectangle. */
    static boolean crossesNode(final KVector a, final KVector b, final ElkNode node) {
        return crosses(a, b, node.getX(), node.getY(), node.getWidth(), node.getHeight());
    }

    private static boolean crosses(final KVector a, final KVector b, final double x, final double y,
            final double width, final double height) {
        double enter = 0;
        double leave = 1;
        double[] origin = {a.x, a.y};
        double[] delta = {b.x - a.x, b.y - a.y};
        double[] low = {x + 1e-6, y + 1e-6};
        double[] high = {x + width - 1e-6, y + height - 1e-6};
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
}
