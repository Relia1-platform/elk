/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.ElkNode;

/** Reserves room for the rectangular clearance envelope used by fixed-position routing. */
public final class GeometryClearance {
    private GeometryClearance() { }

    public static double nodeSpacing(final ElkNode graph, final double requested) {
        double edgeClearance = graph.getProperty(CoreOptions.SPACING_EDGE_NODE);
        if (!Double.isFinite(requested) || requested < 0 || !Double.isFinite(edgeClearance) || edgeClearance < 0) {
            throw new IllegalArgumentException("Geometric spacing must be finite and nonnegative");
        }
        // Inflating a rectangle by d can enlarge its enclosing disk by sqrt(2)*d.
        return Math.max(requested, 2 * Math.sqrt(2) * edgeClearance + 1e-6);
    }
}
