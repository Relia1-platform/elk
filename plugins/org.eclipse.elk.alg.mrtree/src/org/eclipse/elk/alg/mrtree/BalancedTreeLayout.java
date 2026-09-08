/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.mrtree;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.eclipse.elk.alg.common.EdgeLabelReservation;
import org.eclipse.elk.alg.common.EdgeLabelReservation.Reservation;
import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.Direction;

/**
 * Iterative contour placement. The two directional passes remove a left-to-right packing bias.
 * Labeled tree edges reserve room in front of and beside their child for packing only; the visible
 * envelope that centers a parent over its descendants excludes those reservations.
 *
 * A parent with more leaf children than the hanging threshold gets them as an org-chart block:
 * the leaves hang in columns beside vertical trunks, in rows of one leaf per side, connected by a
 * stub across the tree axis that is long enough for their labels. The block sits on the parent's
 * axis, so a middle trunk continues the parent's spine, the other children pack outward on both
 * sides of it, and its first row keeps the level rhythm.
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

    /** A hanging block: leaf positions relative to the block axis and the parent's level. */
    private static final class Block {
        private final List<Vertex> leaves;
        private final double[] axis;
        private final double[] level;
        private final double[] trunk;
        private final Profile profile = new Profile();
        private double offset;
        private Block(final List<Vertex> leaves) {
            this.leaves = leaves;
            axis = new double[leaves.size()];
            level = new double[leaves.size()];
            trunk = new double[leaves.size()];
        }
    }

    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final Direction direction, final double spacing) {
        place(data, component, root, direction, spacing, false);
    }

    /** With bus routing, labeled children reserve a full drop for their label instead of a slanted edge. */
    public static void place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final Direction direction, final double spacing, final boolean busRouting) {
        place(data, component, root, direction, spacing, busRouting, 0);
    }

    /**
     * Places the component and returns, for every hanging leaf, the offset of its trunk from the
     * parent's cross-axis coordinate, so that connectors can be drawn after packing and translation.
     *
     * @param hangingThreshold a parent with more leaf children than this gets a hanging block; zero
     *            keeps every child in its row
     */
    public static Map<Vertex, Double> place(final GeometryGraph data, final List<Vertex> component, final Vertex root,
            final Direction direction, final double spacing, final boolean busRouting, final int hangingThreshold) {
        List<Vertex> traversal = data.spanningTree(component, root);
        Profile[] profiles = new Profile[data.vertices.size()];
        double[] offsets = new double[data.vertices.size()];
        double[] labelFront = new double[data.vertices.size()];
        double[] labelSide = new double[data.vertices.size()];
        EdgeLabelReservation.Margins margins = EdgeLabelReservation.Margins.of(data.graph);
        EdgeLabelReservation.treeExtents(data, component, direction, margins, labelFront, labelSide, busRouting);
        Map<Vertex, Block> blocks = new HashMap<>();
        Block[] hanging = new Block[data.vertices.size()];
        if (hangingThreshold > 0) {
            for (Vertex vertex : component) {
                List<Vertex> leaves = new ArrayList<>();
                for (Vertex child : vertex.children) {
                    if (child.children.isEmpty()) { leaves.add(child); }
                }
                if (leaves.size() > hangingThreshold) {
                    Block block = new Block(leaves);
                    blocks.put(vertex, block);
                    for (Vertex leaf : leaves) {
                        hanging[leaf.index] = block;
                        // Hanging labels sit on the stubs, not in front of the level.
                        labelFront[leaf.index] = 0;
                        labelSide[leaf.index] = 0;
                    }
                }
            }
        }
        double before = 0;
        double after = 0;
        for (Vertex vertex : component) {
            before = Math.max(before, -front(vertex, direction) + labelFront[vertex.index]);
            after = Math.max(after, back(vertex, direction));
        }
        double step = before + after + spacing;
        if (!blocks.isEmpty()) {
            Map<Vertex, double[]> stubLabels = stubLabels(data, component, direction, margins);
            for (Block block : blocks.values()) { build(block, direction, spacing, step, after, margins, stubLabels); }
        }
        for (int index = traversal.size() - 1; index >= 0; index--) {
            Vertex vertex = traversal.get(index);
            double left = left(vertex, direction);
            double right = right(vertex, direction);
            double packRight = Math.max(right, labelSide[vertex.index]);
            Block block = blocks.get(vertex);
            Profile profile;
            if (vertex.children.isEmpty()) {
                profile = new Profile();
                profile.bands.add(new Band(left, packRight));
                profile.min = left;
                profile.max = right;
            } else if (vertex.children.size() == 1 && block == null) {
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
                // Row children pack as before. A hanging block is pinned to the parent's axis and
                // the row children pack outward from it on both sides, half of them each.
                List<Vertex> row = new ArrayList<>();
                for (Vertex child : vertex.children) {
                    if (hanging[child.index] != block || block == null) { row.add(child); }
                }
                List<Profile> items = new ArrayList<>();
                for (Vertex child : row) { items.add(profiles[child.index]); }
                int blockIndex = -1;
                double[] itemOffsets;
                if (block != null) {
                    blockIndex = row.size() / 2;
                    items.add(blockIndex, block.profile);
                    itemOffsets = new double[items.size()];
                    double[] rightward = pack(items.subList(blockIndex, items.size()), spacing, false);
                    double[] leftward = pack(items.subList(0, blockIndex + 1), spacing, true);
                    for (int i = 0; i < items.size(); i++) {
                        itemOffsets[i] = i < blockIndex ? leftward[i] : rightward[i - blockIndex];
                    }
                } else {
                    double[] forward = pack(items, spacing, false);
                    double[] backward = pack(items, spacing, true);
                    itemOffsets = new double[items.size()];
                    for (int i = 0; i < items.size(); i++) { itemOffsets[i] = (forward[i] + backward[i]) / 2; }
                }
                double min = Double.POSITIVE_INFINITY;
                double max = Double.NEGATIVE_INFINITY;
                for (int i = 0; i < items.size(); i++) {
                    min = Math.min(min, items.get(i).min + itemOffsets[i]);
                    max = Math.max(max, items.get(i).max + itemOffsets[i]);
                }
                double midpoint = block != null ? 0 : (min + max) / 2;
                List<Band> union = new ArrayList<>();
                for (int i = 0; i < items.size(); i++) {
                    itemOffsets[i] -= midpoint;
                    merge(union, items.get(i), itemOffsets[i], false);
                }
                int r = 0;
                for (int i = 0; i < items.size(); i++) {
                    if (i == blockIndex) {
                        block.offset = itemOffsets[i];
                        for (int j = 0; j < block.leaves.size(); j++) {
                            offsets[block.leaves.get(j).index] = block.offset + block.axis[j];
                            profiles[block.leaves.get(j).index] = null;
                        }
                    } else {
                        Vertex child = row.get(r++);
                        offsets[child.index] = itemOffsets[i];
                        profiles[child.index] = null;
                    }
                }
                profile = new Profile();
                profile.bands.add(new Band(left, packRight));
                profile.bands.addAll(union);
                profile.min = Math.min(left, min - midpoint);
                profile.max = Math.max(right, max - midpoint);
            }
            profiles[vertex.index] = profile;
        }
        Map<Vertex, Double> trunks = new HashMap<>();
        double[] axis = new double[data.vertices.size()];
        double[] level = new double[data.vertices.size()];
        for (Vertex vertex : traversal) {
            if (vertex.parent != null) {
                axis[vertex.index] = axis[vertex.parent.index] + offsets[vertex.index];
                Block block = hanging[vertex.index];
                if (block != null) {
                    int j = block.leaves.indexOf(vertex);
                    level[vertex.index] = level[vertex.parent.index] + block.level[j];
                    trunks.put(vertex, block.offset + block.trunk[j]);
                } else {
                    level[vertex.index] = level[vertex.parent.index] + step;
                }
            }
            double x = axis[vertex.index];
            double y = level[vertex.index];
            switch (direction) {
            case LEFT: vertex.x = -y; vertex.y = x; break;
            case RIGHT: vertex.x = y; vertex.y = x; break;
            case UP: vertex.x = x; vertex.y = -y; break;
            default: vertex.x = x; vertex.y = y; break;
            }
        }
        return trunks;
    }

    /** Label extents of tree edges measured along and across a stub, keyed by the child. */
    private static Map<Vertex, double[]> stubLabels(final GeometryGraph data, final List<Vertex> component,
            final Direction direction, final EdgeLabelReservation.Margins margins) {
        Map<Vertex, double[]> result = new HashMap<>();
        boolean horizontal = horizontal(direction);
        // A stub runs across the tree axis: horizontally for DOWN and UP, vertically for LEFT and RIGHT.
        double ux = horizontal ? 0 : 1;
        double uy = horizontal ? 1 : 0;
        for (Reservation reservation : EdgeLabelReservation.collect(data, component)) {
            Vertex child;
            if (reservation.a.parent == reservation.b) { child = reservation.a; }
            else if (reservation.b.parent == reservation.a) { child = reservation.b; }
            else { continue; }
            double along = reservation.along(ux, uy, margins.labelGap);
            double across = reservation.across(ux, uy, margins.labelGap);
            double[] known = result.get(child);
            if (known == null) {
                result.put(child, new double[] {along, across});
            } else {
                known[0] = Math.max(known[0], along);
                known[1] = Math.max(known[1], across);
            }
        }
        return result;
    }

    /**
     * Lays the block out: rows of one leaf per trunk side, the number of rows chosen so that the
     * block is as square as possible, stubs long enough for the widest label, and one contour
     * band per level the block covers.
     */
    private static void build(final Block block, final Direction direction, final double spacing, final double step,
            final double after, final EdgeLabelReservation.Margins margins, final Map<Vertex, double[]> stubLabels) {
        int count = block.leaves.size();
        double column = 0;
        double minFront = 0;
        double maxBack = 0;
        double stub = spacing;
        double rowExtent = 0;
        for (Vertex leaf : block.leaves) {
            column = Math.max(column, right(leaf, direction) - left(leaf, direction));
            minFront = Math.min(minFront, front(leaf, direction));
            maxBack = Math.max(maxBack, back(leaf, direction));
            double[] label = stubLabels.get(leaf);
            if (label != null) {
                stub = Math.max(stub, label[0] + 2 * margins.labelNode);
                // Beside the stub, at the row center, a label reaches across into the next row's room.
                rowExtent = Math.max(rowExtent, (back(leaf, direction) - front(leaf, direction)) / 2
                        + label[1] + margins.edgeLabel + margins.labelNode);
            }
        }
        double rowPitch = Math.max(maxBack - minFront, rowExtent) + spacing;
        double trunkPitch = 2 * (column + stub) + spacing;
        // The squarest block: rows per trunk against trunks, judged by the larger side.
        int rows = 1;
        double best = Double.POSITIVE_INFINITY;
        for (int candidate = 1; candidate <= (count + 1) / 2; candidate++) {
            int trunks = (count + 2 * candidate - 1) / (2 * candidate);
            double width = trunks * trunkPitch - spacing;
            double height = (candidate - 1) * rowPitch + maxBack - minFront;
            double side = Math.max(width, height);
            if (side < best - 1e-9) {
                best = side;
                rows = candidate;
            }
        }
        int trunks = (count + 2 * rows - 1) / (2 * rows);
        double width = trunks * trunkPitch - spacing;
        for (int j = 0; j < count; j++) {
            int trunk = j / (2 * rows);
            int slot = j % (2 * rows);
            int row = slot / 2;
            boolean lower = slot % 2 == 0;
            Vertex leaf = block.leaves.get(j);
            double trunkAxis = -width / 2 + column + stub + trunk * trunkPitch;
            block.trunk[j] = trunkAxis;
            block.axis[j] = lower ? trunkAxis - stub - right(leaf, direction) : trunkAxis + stub - left(leaf, direction);
            block.level[j] = step + row * rowPitch;
        }
        // Contour bands: the block spans the levels from its first row to its last, rounded up.
        double bottom = step + (rows - 1) * rowPitch + maxBack;
        int levels = Math.max(1, (int) Math.ceil((bottom - after) / step - 1e-9));
        for (int k = 0; k < levels; k++) { block.profile.bands.add(new Band(-width / 2, width / 2)); }
        block.profile.min = -width / 2;
        block.profile.max = width / 2;
    }

    private static double[] pack(final List<Profile> items, final double spacing, final boolean reverse) {
        List<Band> occupied = new ArrayList<>();
        double[] positions = new double[items.size()];
        for (int k = 0; k < items.size(); k++) {
            int i = reverse ? items.size() - 1 - k : k;
            Profile profile = items.get(i);
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
