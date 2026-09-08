/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.options;

/** How connected components of one scope are arranged. */
public enum GeometricPacking {
    /** Shelves in model order: components fill a row and wrap, as before. */
    SHELF,
    /** Shelves by decreasing height, first fit: denser, tops aligned, independent of input order. */
    COMPACT
}
