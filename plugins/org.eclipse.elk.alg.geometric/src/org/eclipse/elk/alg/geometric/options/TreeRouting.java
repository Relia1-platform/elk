/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric.options;

/** Connector style for parent-child edges of a placed tree. */
public enum TreeRouting {
    /** Straight connectors from parent to child, routed around obstacles when necessary. */
    DIRECT,
    /** One orthogonal bus per parent with perpendicular drops into the children, as in an org chart. */
    BUS
}
