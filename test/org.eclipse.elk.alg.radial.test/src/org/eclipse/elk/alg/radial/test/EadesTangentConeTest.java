/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.radial.test;

import static org.junit.Assert.assertTrue;

import org.eclipse.elk.alg.radial.RadialLayoutProvider;
import org.eclipse.elk.alg.radial.options.CompactionStrategy;
import org.eclipse.elk.alg.radial.options.RadialOptions;
import org.eclipse.elk.alg.test.PlainJavaInitialization;
import org.eclipse.elk.core.util.BasicProgressMonitor;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.util.ElkGraphUtil;
import org.junit.BeforeClass;
import org.junit.Test;

public class EadesTangentConeTest {
    @BeforeClass
    public static void initialize() { PlainJavaInitialization.initializePlainJavaLayout(); }

    @Test
    public void supportEdgesStayOutsideTheParentTangent() {
        ElkNode graph = ElkGraphUtil.createGraph();
        graph.setProperty(RadialOptions.RADIUS, 100.0);
        graph.setProperty(RadialOptions.COMPACTOR, CompactionStrategy.NONE);
        graph.setProperty(RadialOptions.ROTATE, false);
        ElkNode root = node(graph);
        ElkNode heavy = node(graph);
        ElkGraphUtil.createSimpleEdge(root, heavy);
        ElkGraphUtil.createSimpleEdge(root, node(graph));
        for (int i = 0; i < 12; i++) { ElkGraphUtil.createSimpleEdge(heavy, node(graph)); }
        new RadialLayoutProvider().layout(graph, new BasicProgressMonitor());
        double radialX = heavy.getX() - root.getX();
        double radialY = heavy.getY() - root.getY();
        for (ElkNode leaf : graph.getChildren().subList(3, graph.getChildren().size())) {
            double dot = (leaf.getX() - heavy.getX()) * radialX + (leaf.getY() - heavy.getY()) * radialY;
            assertTrue("finite coordinates", Double.isFinite(leaf.getX()) && Double.isFinite(leaf.getY()));
            assertTrue("support edge points into the inner circle", dot >= -1e-6);
        }
    }

    private static ElkNode node(final ElkNode graph) {
        ElkNode node = ElkGraphUtil.createNode(graph);
        node.setDimensions(40, 30);
        return node;
    }
}
