/*******************************************************************************
 * Copyright (c) 2013 - 2022 Kiel University and others.
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
package org.eclipse.elk.alg.mrtree;

import java.util.List;

import org.eclipse.elk.alg.common.NodeMicroLayout;
import org.eclipse.elk.alg.common.FixedNodeRouter;
import org.eclipse.elk.alg.common.GeometryBounds;
import org.eclipse.elk.alg.common.GeometryClearance;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.alg.common.GeometryPacking;
import org.eclipse.elk.alg.mrtree.options.TreeNodePlacement;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.alg.mrtree.graph.TGraph;
import org.eclipse.elk.alg.mrtree.options.MrTreeOptions;
import org.eclipse.elk.core.AbstractLayoutProvider;
import org.eclipse.elk.core.util.IElkProgressMonitor;
import org.eclipse.elk.graph.ElkNode;

/**
 * Layout provider to connect the tree layouter to the Eclipse based layout services and orchestrate
 * the pre layout processing.
 * 
 * @author sor
 * @author sgu
 * @author sdo
 */
public class TreeLayoutProvider extends AbstractLayoutProvider {

    // /////////////////////////////////////////////////////////////////////////////
    // Variables

    /** the layout algorithm used for this layout. */
    private MrTree klayTree = new MrTree();
    /** connected components processor. */
    private ComponentsProcessor componentsProcessor = new ComponentsProcessor();
    
    private final float defaultWork = 0.1f;

    // /////////////////////////////////////////////////////////////////////////////
    // Regular Layout

    /**
     * {@inheritDoc}
     */
    @Override
    public void layout(final ElkNode layoutGraph, final IElkProgressMonitor progressMonitor) {
        
        // If requested, compute nodes's dimensions, place node labels, ports, port labels, etc.
        if (!layoutGraph.getProperty(MrTreeOptions.OMIT_NODE_MICRO_LAYOUT)) {
            NodeMicroLayout.forGraph(layoutGraph)
                           .execute();
        }
        if (layoutGraph.getProperty(MrTreeOptions.NODE_PLACEMENT) == TreeNodePlacement.BALANCED) {
            progressMonitor.begin("Balanced tree layout", 1);
            GeometryGraph geometry = new GeometryGraph(layoutGraph, false);
            List<List<Vertex>> components = geometry.components();
            for (List<Vertex> component : components) {
                BalancedTreeLayout.place(geometry, component, geometry.chooseRoot(component),
                        layoutGraph.getProperty(CoreOptions.DIRECTION),
                        GeometryClearance.nodeSpacing(layoutGraph, layoutGraph.getProperty(CoreOptions.SPACING_NODE_NODE)));
            }
            GeometryPacking.pack(components, layoutGraph.getProperty(CoreOptions.SPACING_COMPONENT_COMPONENT),
                    layoutGraph.getProperty(CoreOptions.ASPECT_RATIO));
            geometry.applyPositions();
            new FixedNodeRouter(layoutGraph).route();
            GeometryBounds.normalize(layoutGraph, null, true);
            progressMonitor.done();
            return;
        }
        // build tGraph
        IElkProgressMonitor pm = progressMonitor.subTask(defaultWork);
        pm.begin("build tGraph", 1);
        IGraphImporter<ElkNode> graphImporter = new ElkGraphImporter();
        TGraph tGraph = graphImporter.importGraph(layoutGraph);
        pm.done();

        // split the input graph into components
        pm = progressMonitor.subTask(defaultWork);
        pm.begin("Split graph", 1);
        List<TGraph> components = componentsProcessor.split(tGraph);
        pm.done();

        // perform the actual layout on the components
        for (TGraph comp : components) {
            klayTree.doLayout(comp, progressMonitor.subTask((1.0f - defaultWork * 4) / components.size()));
        }

        // pack the components back into one graph
        pm = progressMonitor.subTask(defaultWork);
        pm.begin("Pack components", 1);
        tGraph = componentsProcessor.pack(components);
        pm.done();

        // apply the layout results to the original graph
        pm = progressMonitor.subTask(defaultWork);
        pm.begin("Apply layout results", 1);
        graphImporter.applyLayout(tGraph);
        pm.done();
    }

}
