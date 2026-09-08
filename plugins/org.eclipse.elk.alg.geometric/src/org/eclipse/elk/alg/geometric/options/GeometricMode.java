/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.options;

/** Geometric layout families. FIXED keeps the given node positions and only routes edges and labels. */
public enum GeometricMode {
    AUTO, TREE, RADIAL, RING, CLUSTER, FIXED
}
