/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.mrtree;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.eclipse.elk.alg.common.EdgeLabelReservation;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.Direction;

/**
 * Iterative contour placement. The two directional passes remove a left-to-right packing bias.
 * Labeled tree edges reserve room in front of and beside their child for packing only; the visible
 * envelope that centers a parent over its descendants excludes those reservations.
 */
public final class BalancedTreeLayout {
    private BalancedTreeLayout() { }

    private static final class Band {
        private double left;
        private double right;
        private Band(final double left, final double right) {
            this.left = left;
            this.right = right;
        }
    }

    private static final class Profile {
        private final LinkedList<Band> bands = new LinkedList<>();
        private double offset;
        private double min;
        private double max;
    }

    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final Direction direction, final double spacing) {
        place(data, component, root, direction, spacing, false);
    }

    /** With bus routing, labeled children reserve a full drop for their label instead of a slanted edge. */
    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final Direction direction, final double spacing, final boolean busRouting) {
        List<Vertex> traversal = data.spanningTree(component, root);
        Profile[] profiles = new Profile[data.vertices.size()];
        double[] offsets = new double[data.vertices.size()];
        double[] labelFront = new double[data.vertices.size()];
        double[] labelSide = new double[data.vertices.size()];
        EdgeLabelReservation.treeExtents(data, component, direction, EdgeLabelReservation.Margins.of(data.graph),
                labelFront, labelSide, busRouting);
        double before = 0;
        double after = 0;
        for (Vertex vertex : component) {
            before = Math.max(before, -front(vertex, direction) + labelFront[vertex.index]);
            after = Math.max(after, back(vertex, direction));
        }
        double step = before + after + spacing;
        for (int index = traversal.size() - 1; index >= 0; index--) {
            Vertex vertex = traversal.get(index);
            double left = left(vertex, direction);
            double right = right(vertex, direction);
            double packRight = Math.max(right, labelSide[vertex.index]);
            Profile profile;
            if (vertex.children.isEmpty()) {
                profile = new Profile();
                profile.bands.add(new Band(left, packRight));
                profile.min = left;
                profile.max = right;
            } else if (vertex.children.size() == 1) {
                // Reuse the child's contour: a long chain needs linear memory and no recursion.
                Vertex child = vertex.children.get(0);
                profile = profiles[child.index];
                double shift = -(profile.min + profile.max) / 2;
                offsets[child.index] = shift;
                profile.offset += shift;
                profile.min = Math.min(left, profile.min + shift);
                profile.max = Math.max(right, profile.max + shift);
                profile.bands.addFirst(new Band(left - profile.offset, packRight - profile.offset));
                profiles[child.index] = null;
            } else {
                double[] forward = pack(vertex.children, profiles, spacing, false);
                double[] backward = pack(vertex.children, profiles, spacing, true);
                double min = Double.POSITIVE_INFINITY;
                double max = Double.NEGATIVE_INFINITY;
                for (int i = 0; i < vertex.children.size(); i++) {
                    Vertex child = vertex.children.get(i);
                    offsets[child.index] = (forward[i] + backward[i]) / 2;
                    min = Math.min(min, profiles[child.index].min + offsets[child.index]);
                    max = Math.max(max, profiles[child.index].max + offsets[child.index]);
                }
                double midpoint = (min + max) / 2;
                List<Band> union = new ArrayList<>();
                for (Vertex child : vertex.children) {
                    offsets[child.index] -= midpoint;
                    merge(union, profiles[child.index], offsets[child.index], false);
                    profiles[child.index] = null;
                }
                profile = new Profile();
                profile.bands.add(new Band(left, packRight));
                profile.bands.addAll(union);
                profile.min = Math.min(left, min - midpoint);
                profile.max = Math.max(right, max - midpoint);
            }
            profiles[vertex.index] = profile;
        }
        double[] axis = new double[data.vertices.size()];
        for (Vertex vertex : traversal) {
            if (vertex.parent != null) {
                axis[vertex.index] = axis[vertex.parent.index] + offsets[vertex.index];
            }
            double x = axis[vertex.index];
            double y = vertex.depth * step;
            switch (direction) {
            case LEFT: vertex.x = -y; vertex.y = x; break;
            case RIGHT: vertex.x = y; vertex.y = x; break;
            case UP: vertex.x = x; vertex.y = -y; break;
            default: vertex.x = x; vertex.y = y; break;
            }
        }
    }

    private static double[] pack(final List<Vertex> children, final Profile[] profiles,
            final double spacing, final boolean reverse) {
        List<Band> occupied = new ArrayList<>();
        double[] positions = new double[children.size()];
        for (int k = 0; k < children.size(); k++) {
            int i = reverse ? children.size() - 1 - k : k;
            Profile profile = profiles[children.get(i).index];
            double shift = 0;
            int depth = 0;
            for (Band band : profile.bands) {
                double left = reverse ? -band.right - profile.offset : band.left + profile.offset;
                if (depth < occupied.size()) {
                    shift = Math.max(shift, occupied.get(depth).right + spacing - left);
                }
                depth++;
            }
            positions[i] = reverse ? -shift : shift;
            merge(occupied, profile, shift, reverse);
        }
        return positions;
    }

    private static void merge(final List<Band> union, final Profile profile,
            final double shift, final boolean reverse) {
        int depth = 0;
        for (Band band : profile.bands) {
            double left = (reverse ? -band.right - profile.offset : band.left + profile.offset) + shift;
            double right = (reverse ? -band.left - profile.offset : band.right + profile.offset) + shift;
            if (depth == union.size()) {
                union.add(new Band(left, right));
            } else {
                Band old = union.get(depth);
                old.left = Math.min(old.left, left);
                old.right = Math.max(old.right, right);
            }
            depth++;
        }
    }

    private static boolean horizontal(final Direction direction) {
        return direction == Direction.LEFT || direction == Direction.RIGHT;
    }
    private static double left(final Vertex vertex, final Direction direction) {
        return horizontal(direction) ? vertex.top : vertex.left;
    }
    private static double right(final Vertex vertex, final Direction direction) {
        return horizontal(direction) ? vertex.bottom : vertex.right;
    }
    private static double front(final Vertex vertex, final Direction direction) {
        switch (direction) {
        case LEFT: return -vertex.right;
        case RIGHT: return vertex.left;
        case UP: return -vertex.bottom;
        default: return vertex.top;
        }
    }
    private static double back(final Vertex vertex, final Direction direction) {
        switch (direction) {
        case LEFT: return -vertex.left;
        case RIGHT: return vertex.right;
        case UP: return -vertex.top;
        default: return vertex.bottom;
        }
    }
}
