/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.HashMap;
import java.util.Map;

import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.ElkConnectableShape;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkGraphElement;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkShape;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;

/** The public graph is changed only after all placement, routing and bounds checks succeed. */
final class GeometryTransaction {
    final ElkNode working;
    private final EcoreUtil.Copier copier = new EcoreUtil.Copier();
    private final Map<EObject, EObject> original = new HashMap<>();

    GeometryTransaction(final ElkNode input) {
        working = (ElkNode) copier.copy(input);
        copier.copyReferences();
        for (Map.Entry<EObject, EObject> entry : copier.entrySet()) {
            original.put(entry.getValue(), entry.getKey());
            if (entry.getKey() instanceof ElkGraphElement) {
                ElkGraphElement source = (ElkGraphElement) entry.getKey();
                ElkGraphElement copy = (ElkGraphElement) entry.getValue();
                copy.copyProperties(source);
                if (source instanceof ElkNode) {
                    if (source.hasProperty(CoreOptions.PADDING)) {
                        copy.setProperty(CoreOptions.PADDING, source.getProperty(CoreOptions.PADDING).clone());
                    }
                    if (source.hasProperty(CoreOptions.MARGINS)) {
                        copy.setProperty(CoreOptions.MARGINS, source.getProperty(CoreOptions.MARGINS).clone());
                    }
                }
            }
        }
    }

    void commit() {
        for (Map.Entry<EObject, EObject> entry : copier.entrySet()) {
            if (entry.getKey() instanceof ElkShape) {
                ElkShape target = (ElkShape) entry.getKey();
                ElkShape result = (ElkShape) entry.getValue();
                target.setLocation(result.getX(), result.getY());
                target.setDimensions(result.getWidth(), result.getHeight());
                if (target instanceof ElkNode) {
                    target.setProperty(CoreOptions.CHILD_AREA_WIDTH, result.getProperty(CoreOptions.CHILD_AREA_WIDTH));
                    target.setProperty(CoreOptions.CHILD_AREA_HEIGHT, result.getProperty(CoreOptions.CHILD_AREA_HEIGHT));
                }
            } else if (entry.getKey() instanceof ElkEdge) {
                ElkEdge target = (ElkEdge) entry.getKey();
                ElkEdge result = (ElkEdge) entry.getValue();
                EcoreUtil.Copier sections = new EcoreUtil.Copier();
                sections.copyAll(result.getSections());
                sections.copyReferences();
                target.getSections().clear();
                for (ElkEdgeSection section : result.getSections()) {
                    ElkEdgeSection copy = (ElkEdgeSection) sections.get(section);
                    copy.setIncomingShape((ElkConnectableShape) original.get(section.getIncomingShape()));
                    copy.setOutgoingShape((ElkConnectableShape) original.get(section.getOutgoingShape()));
                    target.getSections().add(copy);
                }
                target.setProperty(CoreOptions.JUNCTION_POINTS, result.getProperty(CoreOptions.JUNCTION_POINTS));
            }
        }
    }
}
