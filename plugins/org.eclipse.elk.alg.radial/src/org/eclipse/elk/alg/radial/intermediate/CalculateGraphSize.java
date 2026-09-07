/*******************************************************************************
 * Copyright (c) 2017 Kiel University and others.
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.

 * This Source Code may also be made available under the following Secondary
 * Licenses when the conditions for such availability set forth in the Eclipse
 * Public License v. 2.0 are satisfied: GPL-3.0 which is available at
 * https://www.gnu.org/licenses/gpl-3.0-standalone.html.
 *
 * SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later
 *******************************************************************************/
package org.eclipse.elk.alg.radial.intermediate;

import org.eclipse.elk.alg.radial.InternalProperties;
import org.eclipse.elk.alg.radial.options.RadialOptions;
import org.eclipse.elk.core.alg.ILayoutProcessor;
import org.eclipse.elk.core.math.ElkMargin;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.IElkProgressMonitor;
import org.eclipse.elk.core.util.ElkUtil;
import org.eclipse.elk.graph.ElkNode;

/**
 * Calculate the size of the graph and shift nodes into positive coordinates if necessary.
 *
 */
public class CalculateGraphSize implements ILayoutProcessor<ElkNode> {

    /** Shift the nodes such that each nodes has x and y coordinates bigger 0. */
    public void process(final ElkNode graph, final IElkProgressMonitor progressMonitor) {
        progressMonitor.begin("Calculate Graph Size", 1);
        progressMonitor.logGraph(graph, "Before");
        // calculate the offset from border spacing and node distribution
        double minXPos = Double.MAX_VALUE;
        double minYPos = Double.MAX_VALUE;
        double maxXPos = -Double.MAX_VALUE;
        double maxYPos = -Double.MAX_VALUE;

        for (ElkNode node : graph.getChildren()) {
            double posX = node.getX();
            double posY = node.getY();
            double width = node.getWidth();
            double height = node.getHeight();
            ElkMargin margins = node.getProperty(CoreOptions.MARGINS);

            minXPos = Math.min(minXPos, posX - margins.left);
            minYPos = Math.min(minYPos, posY - margins.top);
            maxXPos = Math.max(maxXPos, posX + width + margins.right);
            maxYPos = Math.max(maxYPos, posY + height + margins.bottom);
        }

        ElkPadding padding = graph.getProperty(CoreOptions.PADDING);
        if (graph.getChildren().isEmpty()) {
            minXPos = minYPos = maxXPos = maxYPos = 0;
        }
        KVector offset = new KVector(minXPos - padding.getLeft(), minYPos - padding.getTop());
        
        
        double width = maxXPos - minXPos + padding.getHorizontal();
        double height = maxYPos - minYPos + padding.getVertical();
        
        if (graph.getProperty(RadialOptions.CENTER_ON_ROOT) && !graph.getChildren().isEmpty()) {
            ElkNode root = graph.getProperty(InternalProperties.ROOT_NODE);
            double rootX = root.getX() + root.getWidth() / 2;
            double rootY = root.getY() + root.getHeight() / 2;
            // Expand towards the farther extent. Margins affect bounds, not the node's center.
            double halfWidth = Math.max(rootX - minXPos + padding.left, maxXPos - rootX + padding.right);
            double halfHeight = Math.max(rootY - minYPos + padding.top, maxYPos - rootY + padding.bottom);
            width = 2 * halfWidth;
            height = 2 * halfHeight;
            offset.x = rootX - halfWidth;
            offset.y = rootY - halfHeight;
        }

        ElkUtil.translate(graph, -offset.x, -offset.y);

        // set up the graph
        if (!graph.getProperty(CoreOptions.NODE_SIZE_FIXED_GRAPH_SIZE)) {
            graph.setWidth(width);
            graph.setHeight(height);
        }

        // store child area info
        graph.setProperty(CoreOptions.CHILD_AREA_WIDTH, width - padding.getHorizontal());
        graph.setProperty(CoreOptions.CHILD_AREA_HEIGHT, height - padding.getVertical());
        progressMonitor.logGraph(graph, "After");
    }
}
