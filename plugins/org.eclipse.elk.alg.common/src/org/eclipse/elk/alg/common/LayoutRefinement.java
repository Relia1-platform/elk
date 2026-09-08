/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.eclipse.elk.alg.common.GeometryGraph.Vertex;

/**
 * Polishes placed node positions with small moves that keep every hard constraint: connectors whose
 * ends almost share an axis are straightened, nodes that almost share a row or column are aligned,
 * nearly equal gaps in a row or column are made equal, and node borders snap to a grid. A move
 * never brings two nodes closer than the required spacing, never shrinks a gap that is already
 * below it, never grows the drawing when the graph size is fixed, and is accepted only when the
 * local score improves: a straight connector weighs most, then alignment, then grid crispness,
 * minus a small penalty per pixel moved, so positions stay close to the placement and nothing
 * churns. Nodes that already share a coordinate move together when they snap, so alignment and
 * symmetry survive. Moves are tried in a fixed order, so the result is deterministic.
 */
public final class LayoutRefinement {
    private static final double EPSILON = 1e-9;
    /** A straight connector counts at both of its ends, alignment once per axis and node. */
    private static final double STRAIGHT_WEIGHT = 3;
    private static final double ALIGN_WEIGHT = 1;
    private static final double GRID_WEIGHT = 0.5;
    private static final double GAP_WEIGHT = 3;
    /** Penalty per tolerance of displacement, so that a move has to gain something. */
    private static final double MOVE_WEIGHT = 0.25;
    private static final int SWEEPS = 8;
    /** Gaps of a row are equalized only when the largest is at most this multiple of the smallest. */
    private static final double EQUALIZE_RATIO = 1.5;
    /** Tolerance as a fraction of the node spacing, with a floor in pixels. */
    private static final double TOLERANCE_FRACTION = 0.25;
    private static final double TOLERANCE_MINIMUM = 4;

    private final List<Vertex> vertices;
    private final int[] component;
    private final double spacing;
    private final double componentSpacing;
    private final double grid;
    private final double tolerance;
    private final boolean keepBounds;
    private double boundLeft = Double.POSITIVE_INFINITY;
    private double boundTop = Double.POSITIVE_INFINITY;
    private double boundRight = Double.NEGATIVE_INFINITY;
    private double boundBottom = Double.NEGATIVE_INFINITY;
    private int moves;

    private LayoutRefinement(final GeometryGraph graph, final List<List<Vertex>> components, final double spacing,
            final double componentSpacing, final double grid, final boolean keepBounds) {
        // Index order is identifier order under stable identifiers, so sweeps are permutation invariant.
        vertices = graph.vertices;
        component = new int[vertices.size()];
        for (int c = 0; c < components.size(); c++) {
            for (Vertex vertex : components.get(c)) { component[vertex.index] = c; }
        }
        this.spacing = spacing;
        this.componentSpacing = componentSpacing;
        this.grid = grid;
        this.keepBounds = keepBounds;
        tolerance = Math.max(TOLERANCE_MINIMUM, TOLERANCE_FRACTION * spacing);
        for (Vertex vertex : vertices) {
            boundLeft = Math.min(boundLeft, vertex.x + vertex.left);
            boundTop = Math.min(boundTop, vertex.y + vertex.top);
            boundRight = Math.max(boundRight, vertex.x + vertex.right);
            boundBottom = Math.max(boundBottom, vertex.y + vertex.bottom);
        }
    }

    /**
     * Refines the vertex positions in place and returns the number of accepted moves.
     *
     * @param graph the placed graph; vertex positions are read and written
     * @param components connected components, whose nodes keep the component spacing apart
     * @param spacing required separation between node footprints of one component
     * @param componentSpacing required separation between nodes of different components
     * @param grid grid the node borders snap to; zero or less disables snapping
     * @param keepBounds whether the footprint bounds must not grow, for a fixed graph size
     */
    public static int refine(final GeometryGraph graph, final List<List<Vertex>> components, final double spacing,
            final double componentSpacing, final double grid, final boolean keepBounds) {
        if (graph.vertices.isEmpty() || !(spacing >= 0) || !(componentSpacing >= 0) || !Double.isFinite(grid)) {
            return 0;
        }
        return new LayoutRefinement(graph, components, spacing, componentSpacing, grid, keepBounds).run();
    }

    private int run() {
        for (int sweep = 0; sweep < SWEEPS; sweep++) {
            int before = moves;
            straighten();
            align(true);
            align(false);
            equalize(true);
            equalize(false);
            snap(true);
            snap(false);
            if (moves == before) { break; }
        }
        return moves;
    }

    // ---------------------------------------------------------------------------------------------
    // Moves

    /** Connectors whose ends are within the tolerance of a shared axis get that axis. */
    private void straighten() {
        for (Vertex vertex : vertices) {
            for (Vertex neighbor : vertex.neighbors) {
                if (neighbor.index < vertex.index) { continue; }
                double dx = Math.abs(neighbor.x - vertex.x);
                double dy = Math.abs(neighbor.y - vertex.y);
                if (dx < EPSILON || dy < EPSILON) { continue; }
                boolean alongX = dx <= dy;
                if ((alongX ? dx : dy) > tolerance) { continue; }
                join(vertex, neighbor, alongX);
            }
        }
    }

    /** Nodes whose centers are within the tolerance on one axis share that coordinate. */
    private void align(final boolean xAxis) {
        List<Vertex> sorted = sorted(xAxis);
        for (int i = 1; i < sorted.size(); i++) {
            Vertex a = sorted.get(i - 1);
            Vertex b = sorted.get(i);
            double gap = Math.abs(coordinate(b, xAxis) - coordinate(a, xAxis));
            if (gap < EPSILON || gap > tolerance) { continue; }
            join(a, b, xAxis);
        }
    }

    /**
     * Moves one of two vertices onto the other's coordinate on one axis, whichever gains more. The
     * mover carries its block: neighbors whose connector on that axis is straight or nearly straight
     * and would get worse, transitively, so that a staircase of small offsets, as layered placement
     * leaves it, is straightened step by step instead of being pushed beyond the tolerance.
     */
    private boolean join(final Vertex a, final Vertex b, final boolean xAxis) {
        List<Vertex> blockA = block(a, b, xAxis);
        List<Vertex> blockB = block(b, a, xAxis);
        double gainA = blockGain(blockA, coordinate(b, xAxis) - coordinate(a, xAxis), xAxis);
        double gainB = blockGain(blockB, coordinate(a, xAxis) - coordinate(b, xAxis), xAxis);
        if (gainA <= EPSILON && gainB <= EPSILON) { return false; }
        boolean moveA = gainA > gainB + EPSILON || Math.abs(gainA - gainB) <= EPSILON && preferred(a, b) == a;
        List<Vertex> block = moveA ? blockA : blockB;
        double shift = moveA ? coordinate(b, xAxis) - coordinate(a, xAxis) : coordinate(a, xAxis) - coordinate(b, xAxis);
        for (Vertex vertex : block) {
            if (xAxis) { vertex.x += shift; } else { vertex.y += shift; }
        }
        moves++;
        return true;
    }

    /** The mover and, transitively, the neighbors it must carry along when it shifts on the axis. */
    private List<Vertex> block(final Vertex mover, final Vertex anchor, final boolean xAxis) {
        double shift = coordinate(anchor, xAxis) - coordinate(mover, xAxis);
        List<Vertex> block = new ArrayList<>();
        block.add(mover);
        for (int i = 0; i < block.size(); i++) {
            Vertex current = block.get(i);
            for (Vertex neighbor : current.neighbors) {
                if (neighbor == anchor || block.contains(neighbor)) { continue; }
                double offset = coordinate(neighbor, xAxis) - coordinate(current, xAxis);
                // Carried: already straight on this axis, or nearly straight on the far side of the move.
                boolean straight = Math.abs(offset) < EPSILON;
                boolean worsens = Math.abs(offset) <= tolerance && offset * shift < 0;
                if (straight || worsens) { block.add(neighbor); }
            }
        }
        return block;
    }

    /** Score gain of shifting the block on the axis, measured by applying and reverting it. */
    private double blockGain(final List<Vertex> block, final double shift, final boolean xAxis) {
        if (Math.abs(shift) < EPSILON) { return Double.NEGATIVE_INFINITY; }
        double delta = -MOVE_WEIGHT * Math.abs(shift) * block.size() / tolerance;
        int applied = 0;
        boolean feasible = true;
        for (Vertex vertex : block) {
            double x = xAxis ? vertex.x + shift : vertex.x;
            double y = xAxis ? vertex.y : vertex.y + shift;
            feasible = feasible(vertex, x, y);
            if (!feasible) { break; }
            delta += scoreDelta(vertex, x, y);
            vertex.x = x;
            vertex.y = y;
            applied++;
        }
        for (int i = 0; i < applied; i++) {
            Vertex vertex = block.get(i);
            if (xAxis) { vertex.x -= shift; } else { vertex.y -= shift; }
        }
        return feasible ? delta : Double.NEGATIVE_INFINITY;
    }

    /** With equal gains the node with fewer connectors moves, then the later one. */
    private static Vertex preferred(final Vertex a, final Vertex b) {
        if (a.neighbors.size() != b.neighbors.size()) { return a.neighbors.size() < b.neighbors.size() ? a : b; }
        return a.index > b.index ? a : b;
    }

    /** Nearly equal gaps between consecutive nodes of a row (or column) become exactly equal. */
    private void equalize(final boolean rows) {
        List<Vertex> sorted = new ArrayList<>(vertices);
        Collections.sort(sorted, (p, q) -> {
            int byLine = Double.compare(coordinate(p, !rows), coordinate(q, !rows));
            if (byLine != 0) { return byLine; }
            int byPosition = Double.compare(coordinate(p, rows), coordinate(q, rows));
            return byPosition != 0 ? byPosition : Integer.compare(p.index, q.index);
        });
        int i = 0;
        while (i < sorted.size()) {
            int j = i + 1;
            while (j < sorted.size()
                    && Math.abs(coordinate(sorted.get(j), !rows) - coordinate(sorted.get(i), !rows)) < EPSILON) { j++; }
            if (j - i >= 3) { equalizeRun(sorted.subList(i, j), rows); }
            i = j;
        }
    }

    private void equalizeRun(final List<Vertex> run, final boolean rows) {
        int count = run.size();
        double minimum = Double.POSITIVE_INFINITY;
        double maximum = Double.NEGATIVE_INFINITY;
        double total = 0;
        for (int i = 1; i < count; i++) {
            double gap = coordinate(run.get(i), rows) + lead(run.get(i), rows)
                    - coordinate(run.get(i - 1), rows) - trail(run.get(i - 1), rows);
            minimum = Math.min(minimum, gap);
            maximum = Math.max(maximum, gap);
            total += gap;
        }
        if (minimum <= EPSILON || maximum - minimum < EPSILON || maximum > EQUALIZE_RATIO * minimum) { return; }
        double equal = total / (count - 1);
        double[] target = new double[count];
        target[0] = coordinate(run.get(0), rows);
        for (int i = 1; i < count; i++) {
            target[i] = target[i - 1] + trail(run.get(i - 1), rows) + equal - lead(run.get(i), rows);
            if (Math.abs(target[i] - coordinate(run.get(i), rows)) > 2 * tolerance) { return; }
        }
        double[] originalX = new double[count];
        double[] originalY = new double[count];
        for (int i = 0; i < count; i++) {
            originalX[i] = run.get(i).x;
            originalY[i] = run.get(i).y;
        }
        double delta = GAP_WEIGHT * Math.min(1, (maximum - minimum) / equal);
        double displacement = 0;
        boolean feasible = true;
        for (int i = 1; i < count - 1 && feasible; i++) {
            Vertex vertex = run.get(i);
            double x = rows ? target[i] : vertex.x;
            double y = rows ? vertex.y : target[i];
            if (Math.abs(x - vertex.x) + Math.abs(y - vertex.y) < EPSILON) { continue; }
            feasible = feasible(vertex, x, y);
            if (feasible) {
                delta += scoreDelta(vertex, x, y);
                displacement += Math.hypot(x - vertex.x, y - vertex.y);
                vertex.x = x;
                vertex.y = y;
            }
        }
        delta -= MOVE_WEIGHT * displacement / tolerance;
        if (!feasible || delta <= EPSILON) {
            for (int i = 0; i < count; i++) {
                run.get(i).x = originalX[i];
                run.get(i).y = originalY[i];
            }
            return;
        }
        moves++;
    }

    /** Nodes sharing a coordinate snap together, so that their alignment survives. */
    private void snap(final boolean xAxis) {
        if (grid <= 0) { return; }
        List<Vertex> sorted = sorted(xAxis);
        int i = 0;
        while (i < sorted.size()) {
            int j = i + 1;
            while (j < sorted.size()
                    && Math.abs(coordinate(sorted.get(j), xAxis) - coordinate(sorted.get(i), xAxis)) < EPSILON) { j++; }
            List<Vertex> run = sorted.subList(i, j);
            Vertex first = run.get(0);
            double border = coordinate(first, xAxis) - half(first, xAxis);
            double shift = Math.round(border / grid) * grid - border;
            if (Math.abs(shift) >= EPSILON && !snapRun(run, xAxis, shift)) {
                snapRun(run, xAxis, shift + (shift > 0 ? -grid : grid));
            }
            i = j;
        }
    }

    private boolean snapRun(final List<Vertex> run, final boolean xAxis, final double shift) {
        double delta = -MOVE_WEIGHT * Math.abs(shift) * run.size() / tolerance;
        int applied = 0;
        boolean feasible = true;
        for (Vertex vertex : run) {
            double x = xAxis ? vertex.x + shift : vertex.x;
            double y = xAxis ? vertex.y : vertex.y + shift;
            feasible = feasible(vertex, x, y);
            if (!feasible) { break; }
            delta += scoreDelta(vertex, x, y);
            vertex.x = x;
            vertex.y = y;
            applied++;
        }
        if (!feasible || delta <= EPSILON) {
            for (int i = 0; i < applied; i++) {
                Vertex vertex = run.get(i);
                if (xAxis) { vertex.x -= shift; } else { vertex.y -= shift; }
            }
            return false;
        }
        moves++;
        return true;
    }

    // ---------------------------------------------------------------------------------------------
    // Acceptance

    /** Whether the vertex may move to the position: spacing to every other node and the bounds. */
    private boolean feasible(final Vertex vertex, final double x, final double y) {
        if (keepBounds && (x + vertex.left < boundLeft - EPSILON || x + vertex.right > boundRight + EPSILON
                || y + vertex.top < boundTop - EPSILON || y + vertex.bottom > boundBottom + EPSILON)) {
            return false;
        }
        for (Vertex other : vertices) {
            if (other == vertex) { continue; }
            double required = component[vertex.index] == component[other.index] ? spacing
                    : Math.max(spacing, componentSpacing);
            double before = separation(vertex, vertex.x, vertex.y, other);
            double after = separation(vertex, x, y, other);
            if (after + EPSILON < Math.min(required, before)) { return false; }
        }
        return true;
    }

    /** Separation of two footprints: the larger of the gaps along the two axes. */
    private static double separation(final Vertex vertex, final double x, final double y, final Vertex other) {
        double dx = Math.max(other.x + other.left - (x + vertex.right), x + vertex.left - (other.x + other.right));
        double dy = Math.max(other.y + other.top - (y + vertex.bottom), y + vertex.top - (other.y + other.bottom));
        return Math.max(dx, dy);
    }

    /** Change of the total score when the vertex moves to the position, others staying put. */
    private double scoreDelta(final Vertex vertex, final double x, final double y) {
        double delta = 0;
        for (Vertex neighbor : vertex.neighbors) {
            delta += 2 * STRAIGHT_WEIGHT * (straight(x, y, neighbor) - straight(vertex.x, vertex.y, neighbor));
        }
        delta += ALIGN_WEIGHT * (aligned(x, true, vertex) - aligned(vertex.x, true, vertex));
        delta += ALIGN_WEIGHT * (aligned(y, false, vertex) - aligned(vertex.y, false, vertex));
        delta += partnerDelta(vertex.x, x, true, vertex) + partnerDelta(vertex.y, y, false, vertex);
        delta += GRID_WEIGHT * (onGrid(x - half(vertex, true)) - onGrid(vertex.x - half(vertex, true)));
        delta += GRID_WEIGHT * (onGrid(y - half(vertex, false)) - onGrid(vertex.y - half(vertex, false)));
        return delta;
    }

    /** Alignment gained or lost by the nodes the vertex leaves or joins on one axis. */
    private double partnerDelta(final double before, final double after, final boolean xAxis, final Vertex vertex) {
        if (Math.abs(before - after) < EPSILON) { return 0; }
        double delta = 0;
        if (partners(before, xAxis, vertex) == 1) { delta -= ALIGN_WEIGHT; }
        if (partners(after, xAxis, vertex) == 1) { delta += ALIGN_WEIGHT; }
        return delta;
    }

    // ---------------------------------------------------------------------------------------------
    // Measures

    private static int straight(final double x, final double y, final Vertex other) {
        return Math.abs(x - other.x) < EPSILON || Math.abs(y - other.y) < EPSILON ? 1 : 0;
    }

    private int aligned(final double value, final boolean xAxis, final Vertex except) {
        return partners(value, xAxis, except) > 0 ? 1 : 0;
    }

    private int partners(final double value, final boolean xAxis, final Vertex except) {
        int count = 0;
        for (Vertex other : vertices) {
            if (other != except && Math.abs(coordinate(other, xAxis) - value) < EPSILON) { count++; }
        }
        return count;
    }

    private int onGrid(final double border) {
        return grid > 0 && Math.abs(border - Math.round(border / grid) * grid) < 1e-6 ? 1 : 0;
    }

    private List<Vertex> sorted(final boolean xAxis) {
        List<Vertex> sorted = new ArrayList<>(vertices);
        Collections.sort(sorted, (p, q) -> {
            int byCoordinate = Double.compare(coordinate(p, xAxis), coordinate(q, xAxis));
            return byCoordinate != 0 ? byCoordinate : Integer.compare(p.index, q.index);
        });
        return sorted;
    }

    private static double coordinate(final Vertex vertex, final boolean xAxis) { return xAxis ? vertex.x : vertex.y; }
    private static double lead(final Vertex vertex, final boolean xAxis) { return xAxis ? vertex.left : vertex.top; }
    private static double trail(final Vertex vertex, final boolean xAxis) { return xAxis ? vertex.right : vertex.bottom; }
    private static double half(final Vertex vertex, final boolean xAxis) {
        return xAxis ? vertex.node.getWidth() / 2 : vertex.node.getHeight() / 2;
    }
}
