/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.options;

/** Connector style of the geometric router. */
public enum GeometricRouting {
    /** Straight connectors, detouring around obstacles with the fewest bends. */
    POLYLINE,
    /** Axis-aligned connectors that leave and enter nodes through the facing sides. */
    ORTHOGONAL
}
