/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.eclipse.elk.alg.common.EdgeLabelReservation.Margins;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.EdgeLabelPlacement;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.ElkConnectableShape;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;
import org.eclipse.elk.graph.properties.IProperty;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.elk.graph.util.ElkGraphUtil;

/**
 * Deterministic polyline routing around fixed rectangular footprints. No node is moved.
 *
 * Edge labels are placed after the edge has its shortest route: first beside (or, with
 * {@code edgeLabels.inline}, on) a sufficient segment of that route, sliding along the segment if
 * its middle is blocked; then on the smallest local detour that exposes a labeled segment; and only
 * as a last resort on an exterior corridor beyond the nearest side of the drawing. Placed labels
 * become obstacles for the edges routed afterwards, never for their own edge.
 */
public final class FixedNodeRouter {
    private static final double EPSILON = 1e-8;
    /** The geometric order option, read without a dependency on the geometric plugin. */
    private static final IProperty<Object> ORDER = new Property<>("org.eclipse.elk.geometric.order");
    /** Offset steps and tangential shifts tried for a local label detour, in both directions. */
    private static final int DETOUR_OFFSETS = 6;
    private static final int DETOUR_SHIFTS = 2;
    /** Detour candidates whose legs are obstructed and are completed with the visibility search. */
    private static final int DETOUR_SEARCHES = 6;
    /** Positions tried on each side of the preferred position along a segment. */
    private static final int SLIDE_STEPS = 6;

    private final GeometryGraph graph;
    private final List<Rect> obstacles = new ArrayList<>();
    private final Map<ElkNode, Rect> rectangles = new HashMap<>();
    private final Map<String, List<Rect>> cells = new HashMap<>();
    private final Map<String, Rect> visibilityCache = new HashMap<>();
    private Set<Rect> searchBlockers;
    private final List<Segment> routedSegments = new ArrayList<>();
    private final List<Segment> directLines = new ArrayList<>();
    private int routing;
    private final double clearance;
    private final double cellSize;
    private final Margins margins;
    private final boolean inline;
    private final EdgeLabelPlacement placement;

    public FixedNodeRouter(final ElkNode scope) {
        graph = new GeometryGraph(scope, false);
        clearance = Math.max(0, scope.getProperty(CoreOptions.SPACING_EDGE_NODE));
        margins = Margins.of(scope);
        inline = scope.getProperty(CoreOptions.EDGE_LABELS_INLINE);
        EdgeLabelPlacement hint = scope.getProperty(CoreOptions.EDGE_LABELS_PLACEMENT);
        placement = hint == null ? EdgeLabelPlacement.CENTER : hint;
        double total = 0;
        List<Scope> scopes = new ArrayList<>();
        scopes.add(new Scope(graph, 0, 0));
        for (int i = 0; i < scopes.size(); i++) {
            Scope frame = scopes.get(i);
            for (Vertex vertex : frame.graph.vertices) {
                Rect rect = new Rect(vertex.x + vertex.left + frame.x - clearance,
                        vertex.y + vertex.top + frame.y - clearance,
                        vertex.x + vertex.right + frame.x + clearance,
                        vertex.y + vertex.bottom + frame.y + clearance, vertex.node, clearance);
                obstacles.add(rect);
                rectangles.put(vertex.node, rect);
                total += Math.max(rect.right - rect.left, rect.bottom - rect.top);
                if (!vertex.node.getChildren().isEmpty()) {
                    scopes.add(new Scope(new GeometryGraph(vertex.node, false),
                            frame.x + vertex.node.getX(), frame.y + vertex.node.getY()));
                }
            }
        }
        Object order = scope.getProperty(ORDER);
        if (order != null && "STABLE_ID".equals(order.toString())) {
            // Visibility search tie-breaks follow obstacle order; stable identifiers make routes
            // independent of the input order of nodes, like the placement itself.
            Collections.sort(obstacles, (p, q) -> {
                int byId = GeometryGraph.identifier(p.node).compareTo(GeometryGraph.identifier(q.node));
                if (byId != 0) { return byId; }
                int byX = Double.compare(p.left, q.left);
                return byX != 0 ? byX : Double.compare(p.top, q.top);
            });
        }
        cellSize = Math.max(16, total / Math.max(1, obstacles.size()));
        for (Rect rect : obstacles) { index(rect); }
        // Descendant scopes are already laid out: their routes and labels are fixed obstacles for
        // edges that leave a group from the inside, which the group footprint itself does not stop.
        for (int i = 1; i < scopes.size(); i++) {
            Scope frame = scopes.get(i);
            for (ElkEdge edge : frame.graph.graph.getContainedEdges()) {
                for (ElkEdgeSection section : edge.getSections()) {
                    Point previous = new Point(frame.x + section.getStartX(), frame.y + section.getStartY());
                    for (int b = 0; b <= section.getBendPoints().size(); b++) {
                        Point next = b < section.getBendPoints().size()
                                ? new Point(frame.x + section.getBendPoints().get(b).getX(),
                                        frame.y + section.getBendPoints().get(b).getY())
                                : new Point(frame.x + section.getEndX(), frame.y + section.getEndY());
                        routedSegments.add(new Segment(previous, next));
                        previous = next;
                    }
                }
                for (ElkLabel label : edge.getLabels()) {
                    if (label.getWidth() <= 0 || label.getHeight() <= 0) { continue; }
                    reserveLabel(new Rect(frame.x + label.getX(), frame.y + label.getY(),
                            frame.x + label.getX() + label.getWidth(), frame.y + label.getY() + label.getHeight(),
                            null, 0));
                }
            }
        }
    }

    public void route() {
        List<ElkEdge> edges = new ArrayList<>(graph.graph.getContainedEdges());
        // Stable IDs for edges are useful even when node order is intentionally model order.
        Collections.sort(edges, Comparator.comparing(e -> e.getIdentifier() == null ? "" : e.getIdentifier()));
        for (ElkEdge edge : edges) {
            if (edge.getSources().size() != 1 || edge.getTargets().size() != 1) {
                throw new IllegalArgumentException("Geometric polyline routing requires simple edges");
            }
            directLines.add(new Segment(center(edge.getSources().get(0)), center(edge.getTargets().get(0))));
        }
        Map<String, Integer> lanes = new HashMap<>();
        for (routing = 0; routing < edges.size(); routing++) {
            ElkEdge edge = edges.get(routing);
            ElkConnectableShape source = edge.getSources().get(0);
            ElkConnectableShape target = edge.getTargets().get(0);
            Vertex sv = graph.endpoint(source);
            Vertex tv = graph.endpoint(target);
            Point sc = center(source);
            Point tc = center(target);
            int si = sv == null ? -1 : sv.index;
            int ti = tv == null ? -1 : tv.index;
            String key = Math.min(si, ti) + ":" + Math.max(si, ti);
            int lane = lanes.getOrDefault(key, 0);
            lanes.put(key, lane + 1);
            Route route = new Route(edge, source, target, sv, tv);
            route.lane = lane;
            if (GeometryGraph.endpointNode(source) == GeometryGraph.endpointNode(target)) {
                excludeAncestors(source, route.excluded);
                route.points = loop(edge, source, target, lane, route.excluded);
            } else {
                route.start = endpoint(source, tc, sv);
                route.end = endpoint(target, sc, tv);
                excludeAncestors(source, route.excluded);
                excludeAncestors(target, route.excluded);
                route.outgoing = hierarchyGateways(source, tc);
                route.incoming = hierarchyGateways(target, sc);
                Collections.reverse(route.incoming);
                List<Point> gateways = new ArrayList<>(route.outgoing);
                gateways.addAll(route.incoming);
                List<Point> points;
                try {
                    points = through(route.start.gateway, route.end.gateway, gateways, route.excluded);
                } catch (IllegalArgumentException failure) {
                    // Inflated obstacle corners can overlap even when the node clearance is valid.
                    // Free endpoints may leave from another side; fixed ports have only one candidate.
                    List<Endpoint> starts = endpoints(source, tc, sv);
                    List<Endpoint> ends = endpoints(target, sc, tv);
                    List<Point> bestPath = null;
                    double bestLength = Double.POSITIVE_INFINITY;
                    for (Endpoint candidateStart : starts) {
                        for (Endpoint candidateEnd : ends) {
                            try {
                                List<Point> path = through(candidateStart.gateway, candidateEnd.gateway, gateways,
                                        route.excluded);
                                double pathLength = distance(candidateStart.anchor, candidateStart.gateway)
                                        + distance(candidateEnd.anchor, candidateEnd.gateway);
                                for (int p = 1; p < path.size(); p++) { pathLength += distance(path.get(p - 1), path.get(p)); }
                                if (pathLength < bestLength) {
                                    bestPath = path;
                                    bestLength = pathLength;
                                    route.start = candidateStart;
                                    route.end = candidateEnd;
                                }
                            } catch (IllegalArgumentException unavailable) {
                                // Try the next visible side, in a deterministic order.
                            }
                        }
                    }
                    if (bestPath == null) {
                        throw new IllegalArgumentException("Edge " + edge.getIdentifier() + ": " + failure.getMessage());
                    }
                    points = bestPath;
                }
                if (lane > 0 && points.size() == 2) {
                    Point a = points.get(0);
                    Point b = points.get(1);
                    double length = distance(a, b);
                    double offset = lane * Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
                    Point middle = new Point((a.x + b.x) / 2 - (b.y - a.y) / Math.max(1, length) * offset,
                            (a.y + b.y) / 2 + (b.x - a.x) / Math.max(1, length) * offset);
                    if (visible(a, middle, route.excluded) && visible(middle, b, route.excluded)) {
                        points.add(1, middle);
                    }
                }
                points.add(0, route.start.anchor);
                points.add(route.end.anchor);
                route.points = points;
            }
            simplify(route.points);
            placeLabels(route);
            simplify(route.points);
            List<Point> points = route.points;
            for (int i = 1; i < points.size(); i++) {
                routedSegments.add(new Segment(points.get(i - 1), points.get(i)));
            }
            // Polyline routes have no shared hyperedge junctions; discard geometry from earlier layouts.
            edge.setProperty(CoreOptions.JUNCTION_POINTS, null);
            edge.getSections().clear();
            ElkEdgeSection section = ElkGraphUtil.createEdgeSection(edge);
            section.setIncomingShape(source);
            section.setOutgoingShape(target);
            section.setStartLocation(points.get(0).x, points.get(0).y);
            section.setEndLocation(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
            for (int i = 1; i < points.size() - 1; i++) {
                ElkGraphUtil.createBendPoint(section, points.get(i).x, points.get(i).y);
            }
        }
    }

    private void excludeAncestors(final ElkConnectableShape shape, final Set<ElkNode> excluded) {
        for (ElkNode parent = GeometryGraph.endpointNode(shape).getParent(); parent != null && parent != graph.graph;
                parent = parent.getParent()) { excluded.add(parent); }
    }

    private List<Point> hierarchyGateways(final ElkConnectableShape shape, final Point toward) {
        List<Point> points = new ArrayList<>();
        for (ElkNode parent = GeometryGraph.endpointNode(shape).getParent(); parent != null && parent != graph.graph;
                parent = parent.getParent()) {
            points.add(endpoint(parent, toward, graph.endpoint(parent)).gateway);
        }
        return points;
    }

    private List<Point> through(final Point start, final Point end, final List<Point> gateways,
            final Set<ElkNode> excluded) {
        List<Point> result = new ArrayList<>();
        Point previous = start;
        for (Point gateway : gateways) {
            List<Point> leg = shortest(previous, gateway, excluded);
            if (!result.isEmpty()) { leg.remove(0); }
            result.addAll(leg);
            previous = gateway;
        }
        List<Point> leg = shortest(previous, end, excluded);
        if (!result.isEmpty()) { leg.remove(0); }
        result.addAll(leg);
        return result;
    }

    private List<Endpoint> endpoints(final ElkConnectableShape shape, final Point toward, final Vertex vertex) {
        List<Endpoint> result = new ArrayList<>();
        result.add(endpoint(shape, toward, vertex));
        if (!(shape instanceof ElkPort)) {
            Point center = center(shape);
            for (double[] side : new double[][] {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}) {
                result.add(endpoint(shape, new Point(center.x + side[0], center.y + side[1]), vertex));
            }
        }
        return result;
    }

    private Point center(final ElkConnectableShape shape) {
        ElkNode node = GeometryGraph.endpointNode(shape);
        if (node == graph.graph) {
            return shape instanceof ElkPort ? new Point(shape.getX() + shape.getWidth() / 2,
                    shape.getY() + shape.getHeight() / 2) : new Point(shape.getWidth() / 2, shape.getHeight() / 2);
        }
        double x = shape.getX() + shape.getWidth() / 2;
        double y = shape.getY() + shape.getHeight() / 2;
        if (shape instanceof ElkPort) {
            x += node.getX();
            y += node.getY();
        }
        for (ElkNode parent = node.getParent(); parent != null && parent != graph.graph;
                parent = parent.getParent()) {
            x += parent.getX();
            y += parent.getY();
        }
        return new Point(x, y);
    }

    private Endpoint endpoint(final ElkConnectableShape shape, final Point toward, final Vertex vertex) {
        Point center = center(shape);
        double dx = toward.x - center.x;
        double dy = toward.y - center.y;
        Point anchor = center;
        if (shape instanceof ElkPort) {
            PortSide side = shape.getProperty(CoreOptions.PORT_SIDE);
            if (side == PortSide.NORTH) { dx = 0; dy = -1; }
            else if (side == PortSide.SOUTH) { dx = 0; dy = 1; }
            else if (side == PortSide.WEST) { dx = -1; dy = 0; }
            else if (side == PortSide.EAST) { dx = 1; dy = 0; }
        } else {
            double factor = Math.min(shape.getWidth() / (2 * Math.max(EPSILON, Math.abs(dx))),
                    shape.getHeight() / (2 * Math.max(EPSILON, Math.abs(dy))));
            anchor = new Point(center.x + dx * factor, center.y + dy * factor);
        }
        if (Math.abs(dx) + Math.abs(dy) < EPSILON) { dx = 1; }
        Rect rect = rectangles.get(GeometryGraph.endpointNode(shape));
        if (rect == null) { return new Endpoint(anchor, anchor); }
        double factor = Double.POSITIVE_INFINITY;
        if (dx > EPSILON) { factor = Math.min(factor, (rect.right - anchor.x) / dx); }
        if (dx < -EPSILON) { factor = Math.min(factor, (rect.left - anchor.x) / dx); }
        if (dy > EPSILON) { factor = Math.min(factor, (rect.bottom - anchor.y) / dy); }
        if (dy < -EPSILON) { factor = Math.min(factor, (rect.top - anchor.y) / dy); }
        factor = Math.max(0, factor);
        return new Endpoint(anchor, new Point(anchor.x + dx * factor, anchor.y + dy * factor));
    }

    /** Length the free segment of a self-loop needs so that its labels fit beside it. */
    private double loopExtent(final ElkEdge edge, final boolean horizontal) {
        if (edge.getLabels().isEmpty()) { return 0; }
        double[] size = EdgeLabelReservation.composite(edge.getLabels(), horizontal, margins.labelGap);
        return (horizontal ? size[0] : size[1]) + 2 * margins.edgeLabel;
    }

    /**
     * A self-loop leaves one side of its node and returns on the adjacent side, around the corner
     * between them. The top side is tried first, then right, bottom and left; only when every corner
     * is blocked does the loop rise above all obstacles, as it always did.
     */
    private List<Point> loop(final ElkEdge edge, final ElkConnectableShape source, final ElkConnectableShape target,
            final int lane, final Set<ElkNode> excluded) {
        ElkNode node = GeometryGraph.endpointNode(source);
        Point c = center(node);
        Rect rect = rectangles.get(node);
        if (rect == null) { throw new IllegalArgumentException("A scope self-loop needs an enclosing layout scope"); }
        double gap = clearance + (lane + 1) * Math.max(8, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        double width = Math.max(0, (loopExtent(edge, true) - (rect.right - rect.left + 2 * gap)) / 2);
        double height = Math.max(0, (loopExtent(edge, false) - (rect.bottom - rect.top + 2 * gap)) / 2);
        Point east = new Point(c.x + node.getWidth(), c.y);
        Point north = new Point(c.x, c.y - node.getHeight());
        Point west = new Point(c.x - node.getWidth(), c.y);
        Point south = new Point(c.x, c.y + node.getHeight());
        // Free sides first, preferring the side that blocks the fewest edges still to be routed.
        List<Object[]> candidates = new ArrayList<>();
        for (int attempt = 0; attempt < 5; attempt++) {
            int side = attempt == 4 ? 0 : attempt;
            Point leave;
            Point arrive;
            Point first;
            Point second;
            if (side == 0) {
                double top = rect.top - gap;
                if (attempt == 4) { for (Rect obstacle : obstacles) { top = Math.min(top, obstacle.top - gap); } }
                leave = east; arrive = north;
                first = new Point(rect.left - gap - width, top);
                second = new Point(rect.right + gap + width, top);
            } else if (side == 1) {
                leave = south; arrive = east;
                first = new Point(rect.right + gap, rect.top - gap - height);
                second = new Point(rect.right + gap, rect.bottom + gap + height);
            } else if (side == 2) {
                leave = west; arrive = south;
                first = new Point(rect.left - gap - width, rect.bottom + gap);
                second = new Point(rect.right + gap + width, rect.bottom + gap);
            } else {
                leave = north; arrive = west;
                first = new Point(rect.left - gap, rect.top - gap - height);
                second = new Point(rect.left - gap, rect.bottom + gap + height);
            }
            if (attempt < 4 && (blockingRectangle(first, first, excluded) != null
                    || blockingRectangle(second, second, excluded) != null)) { continue; }
            Endpoint start = endpoint(source, leave, graph.endpoint(source));
            Endpoint end = endpoint(target, arrive, graph.endpoint(target));
            boolean nearFirst = distance(start.gateway, first) <= distance(start.gateway, second);
            List<Point> via = new ArrayList<>();
            via.add(nearFirst ? first : second);
            via.add(nearFirst ? second : first);
            int pending = attempt == 4 ? Integer.MAX_VALUE : 0;
            Point previous = start.gateway;
            for (Point point : via) { pending = pendingCrossings(previous, point, pending); previous = point; }
            pending = pendingCrossings(previous, end.gateway, pending);
            candidates.add(new Object[] {pending, attempt, start, end, via});
        }
        Collections.sort(candidates, (p, q) -> {
            int byPending = Integer.compare((Integer) p[0], (Integer) q[0]);
            return byPending != 0 ? byPending : Integer.compare((Integer) p[1], (Integer) q[1]);
        });
        IllegalArgumentException failure = null;
        for (Object[] candidate : candidates) {
            Endpoint start = (Endpoint) candidate[2];
            Endpoint end = (Endpoint) candidate[3];
            @SuppressWarnings("unchecked")
            List<Point> via = (List<Point>) candidate[4];
            try {
                List<Point> result = new ArrayList<>();
                result.add(start.anchor);
                result.addAll(through(start.gateway, end.gateway, via, excluded));
                result.add(end.anchor);
                return result;
            } catch (IllegalArgumentException blocked) {
                failure = blocked;
            }
        }
        throw failure;
    }

    /** Number of direct lines of edges not yet routed that the segment would cross, plus the count so far. */
    private int pendingCrossings(final Point a, final Point b, final int count) {
        if (count == Integer.MAX_VALUE) { return count; }
        int result = count;
        for (int i = routing + 1; i < directLines.size(); i++) {
            Segment line = directLines.get(i);
            if (side(a, b, line.a) * side(a, b, line.b) < -EPSILON
                    && side(line.a, line.b, a) * side(line.a, line.b, b) < -EPSILON) { result++; }
        }
        return result;
    }

    /** A visibility graph is built only for edges whose direct segment is obstructed. */
    private List<Point> shortest(final Point start, final Point end, final Set<ElkNode> excluded) {
        Set<Rect> active = new LinkedHashSet<>();
        searchBlockers = active;
        boolean direct = visible(start, end, excluded);
        searchBlockers = null;
        if (direct) {
            List<Point> result = new ArrayList<>();
            result.add(start);
            result.add(end);
            return result;
        }
        for (int pass = 0; pass <= obstacles.size(); pass++) {
            Set<Rect> discovered = new LinkedHashSet<>();
            searchBlockers = discovered;
            List<Point> result = search(start, end, excluded, active);
            searchBlockers = null;
            if (!active.addAll(discovered)) {
                if (result != null) { return result; }
                break;
            }
        }
        throw new IllegalArgumentException("No polyline corridor from (" + start.x + ", " + start.y
                + ") to (" + end.x + ", " + end.y + "); increase node spacing");
    }

    private List<Point> search(final Point start, final Point end, final Set<ElkNode> excluded,
            final Set<Rect> active) {
        List<Point> points = new ArrayList<>();
        points.add(start);
        points.add(end);
        for (Rect rect : active) {
            if (!excluded.contains(rect.node)) {
                points.add(new Point(rect.left, rect.top));
                points.add(new Point(rect.right, rect.top));
                points.add(new Point(rect.right, rect.bottom));
                points.add(new Point(rect.left, rect.bottom));
            }
        }
        int count = points.size();
        double[] cost = new double[count];
        int[] previous = new int[count];
        int[] bends = new int[count];
        int[] crossings = new int[count];
        boolean[] closed = new boolean[count];
        java.util.Arrays.fill(cost, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(previous, -1);
        cost[0] = 0;
        for (int iteration = 0; iteration < count; iteration++) {
            int best = -1;
            double estimate = Double.POSITIVE_INFINITY;
            for (int i = 0; i < count; i++) {
                double candidate = cost[i] + distance(points.get(i), end);
                if (!closed[i] && (candidate < estimate - EPSILON
                        || Math.abs(candidate - estimate) <= EPSILON && best >= 0
                        && (bends[i] < bends[best] || bends[i] == bends[best] && crossings[i] < crossings[best]))) {
                    best = i;
                    estimate = candidate;
                }
            }
            if (best < 0 || best == 1) { break; }
            closed[best] = true;
            for (int next = 0; next < count; next++) {
                if (closed[next] || next == best) { continue; }
                double candidate = cost[best] + distance(points.get(best), points.get(next));
                if ((candidate < cost[next] - EPSILON
                        || Math.abs(candidate - cost[next]) <= EPSILON && bends[best] + 1 <= bends[next])
                        && visible(points.get(best), points.get(next), excluded)) {
                    int candidateCrossings = crossings[best] + crossingCount(points.get(best), points.get(next));
                    if (Math.abs(candidate - cost[next]) <= EPSILON && bends[best] + 1 == bends[next]
                            && candidateCrossings >= crossings[next]) { continue; }
                    cost[next] = candidate;
                    previous[next] = best;
                    bends[next] = bends[best] + 1;
                    crossings[next] = candidateCrossings;
                }
            }
        }
        if (previous[1] < 0) { return null; }
        List<Point> result = new ArrayList<>();
        for (int i = 1; i >= 0; i = previous[i]) { result.add(points.get(i)); }
        Collections.reverse(result);
        return result;
    }

    private boolean visible(final Point a, final Point b, final Set<ElkNode> excluded) {
        String key = null;
        if (excluded.isEmpty()) {
            String first = a.x + ":" + a.y;
            String second = b.x + ":" + b.y;
            key = first.compareTo(second) < 0 ? first + "/" + second : second + "/" + first;
            if (visibilityCache.containsKey(key)) {
                Rect blocker = visibilityCache.get(key);
                if (blocker != null && searchBlockers != null) { searchBlockers.add(blocker); }
                return blocker == null;
            }
        }
        Rect blocker = blockingRectangle(a, b, excluded);
        if (key != null) {
            if (visibilityCache.size() >= 50000) { visibilityCache.clear(); }
            visibilityCache.put(key, blocker);
        }
        if (blocker != null && searchBlockers != null) { searchBlockers.add(blocker); }
        return blocker == null;
    }

    private Rect blockingRectangle(final Point a, final Point b, final Set<ElkNode> excluded) {
        int minX = cell(Math.min(a.x, b.x));
        int maxX = cell(Math.max(a.x, b.x));
        int minY = cell(Math.min(a.y, b.y));
        int maxY = cell(Math.max(a.y, b.y));
        // Avoid a huge sparse grid walk for long diagonals.
        if ((long) (maxX - minX + 1) * (maxY - minY + 1) > obstacles.size() * 4L) {
            for (Rect rect : obstacles) {
                if (!excluded.contains(rect.node) && rect.crosses(a, b)) { return rect; }
            }
        } else {
            Set<Rect> tested = new HashSet<>();
            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    List<Rect> bucket = cells.get(x + ":" + y);
                    if (bucket == null) { continue; }
                    for (Rect rect : bucket) {
                        if (tested.add(rect) && !excluded.contains(rect.node) && rect.crosses(a, b)) { return rect; }
                    }
                }
            }
        }
        return null;
    }

    // ---------------------------------------------------------------------------------------------
    // Edge labels

    private void placeLabels(final Route route) {
        List<ElkLabel> edgeLabels = route.edge.getLabels();
        if (edgeLabels.isEmpty()) { return; }
        Candidate best = onRoute(route);
        if (best == null && route.start != null) { best = detour(route); }
        if (best == null) { best = corridor(route); }
        if (best.points != null) { route.points = best.points; }
        apply(best, edgeLabels);
    }

    /** Stage 1: the best label position beside an existing segment, without changing the route. */
    private Candidate onRoute(final Route route) {
        List<Point> points = route.points;
        List<Integer> segments = new ArrayList<>();
        for (int i = 1; i < points.size(); i++) { segments.add(i); }
        // Longest segments first; equal lengths keep route order.
        Collections.sort(segments, (a, b) -> Double.compare(distance(points.get(b - 1), points.get(b)),
                distance(points.get(a - 1), points.get(a))));
        for (int segment : segments) {
            Candidate candidate = onSegment(route, points.get(segment - 1), points.get(segment), segment, null);
            if (candidate != null) { return candidate; }
        }
        return null;
    }

    /**
     * Tries label positions along one segment, at the preferred position first and then sliding
     * outward in both directions, and on both sides of the segment. Returns the best feasible
     * candidate at the first position that has one, or null if the segment is too short or blocked.
     */
    private Candidate onSegment(final Route route, final Point a, final Point b, final int hostSegment,
            final List<Point> newPoints) {
        double length = distance(a, b);
        if (length < EPSILON) { return null; }
        double ux = (b.x - a.x) / length;
        double uy = (b.y - a.y) / length;
        double[] size = EdgeLabelReservation.composite(route.edge.getLabels(), Math.abs(ux) >= Math.abs(uy),
                margins.labelGap);
        double along = Math.abs(ux) * size[0] + Math.abs(uy) * size[1];
        double across = Math.abs(uy) * size[0] + Math.abs(ux) * size[1];
        double minimum = along / 2 + margins.edgeLabel;
        double maximum = length - along / 2 - margins.edgeLabel;
        if (maximum + EPSILON < minimum) { return null; }
        double preferred = placement == EdgeLabelPlacement.TAIL ? minimum
                : placement == EdgeLabelPlacement.HEAD ? maximum : length / 2;
        double stride = (maximum - minimum) / SLIDE_STEPS;
        int steps = stride > EPSILON ? SLIDE_STEPS : 0;
        List<Point> hostPoints = newPoints != null ? newPoints : route.points;
        for (int step = 0; step <= steps; step++) {
            for (int direction = 0; direction < (step == 0 ? 1 : 2); direction++) {
                double t = preferred + (direction == 0 ? step : -step) * stride;
                if (t < minimum - EPSILON || t > maximum + EPSILON) { continue; }
                t = Math.max(minimum, Math.min(maximum, t));
                Point on = new Point(a.x + ux * t, a.y + uy * t);
                Candidate best = null;
                for (int side : inline ? new int[] {0} : new int[] {-1, 1}) {
                    double offset = side == 0 ? 0 : across / 2 + margins.edgeLabel;
                    // Side -1 is tried first; placement kernels reserve room on that side.
                    double cx = on.x - side * uy * offset;
                    double cy = on.y + side * ux * offset;
                    Rect rect = new Rect(cx - size[0] / 2, cy - size[1] / 2, cx + size[0] / 2, cy + size[1] / 2,
                            null, 0);
                    Candidate candidate = evaluate(route, rect, hostPoints, hostSegment, ux, uy, size);
                    if (candidate != null && (best == null || candidate.better(best))) { best = candidate; }
                }
                if (best != null) { return best; }
            }
        }
        return null;
    }

    /** Feasibility and quality of one label rectangle for the edge currently being routed. */
    private Candidate evaluate(final Route route, final Rect rect, final List<Point> points, final int hostSegment,
            final double ux, final double uy, final double[] size) {
        // Beyond this distance a side is simply "free"; the first side then wins for consistency.
        double cap = Math.max(margins.labelNode, clearance);
        double margin = cap;
        Rect region = rect.inflate(Math.max(cap, margins.labelLabel) + EPSILON);
        Set<Rect> tested = new HashSet<>();
        for (int x = cell(region.left); x <= cell(region.right); x++) {
            for (int y = cell(region.top); y <= cell(region.bottom); y++) {
                List<Rect> bucket = cells.get(x + ":" + y);
                if (bucket == null) { continue; }
                for (Rect obstacle : bucket) {
                    if (!tested.add(obstacle)) { continue; }
                    if (obstacle.node != null) {
                        if (route.excluded.contains(obstacle.node)) { continue; }
                        double gap = rect.gap(obstacle.deflate(obstacle.inflation));
                        if (gap + EPSILON < margins.labelNode) { return null; }
                        margin = Math.min(margin, gap);
                    } else {
                        double gap = rect.gap(obstacle.deflate(obstacle.inflation));
                        if (gap + EPSILON < margins.labelLabel) { return null; }
                        margin = Math.min(margin, gap);
                    }
                }
            }
        }
        Rect guarded = rect.inflate(margins.edgeLabel);
        for (Segment segment : routedSegments) {
            if (guarded.crosses(segment.a, segment.b)) { return null; }
        }
        for (int i = 1; i < points.size(); i++) {
            if (i == hostSegment) { continue; }
            if (guarded.crosses(points.get(i - 1), points.get(i))) { return null; }
        }
        int pending = 0;
        for (int i = routing + 1; i < directLines.size(); i++) {
            Segment line = directLines.get(i);
            if (guarded.crosses(line.a, line.b)) { pending++; }
        }
        return new Candidate(rect, pending, margin, ux, uy, size);
    }

    /**
     * Stage 2: the smallest deterministic local detour that exposes a segment long enough for the
     * label. Carrier segments parallel to the direct line, horizontal and vertical are tried at
     * increasing offsets and shifts; kinks anchored at either endpoint are tried as well.
     */
    private Candidate detour(final Route route) {
        Point s = route.start.anchor;
        Point e = route.end.anchor;
        double length = distance(s, e);
        double dx = length < EPSILON ? 1 : (e.x - s.x) / length;
        double dy = length < EPSILON ? 0 : (e.y - s.y) / length;
        List<double[]> orientations = new ArrayList<>();
        orientations.add(new double[] {dx, dy});
        if (Math.abs(dy) > 1e-6) { orientations.add(new double[] {1, 0}); }
        if (Math.abs(dx) > 1e-6) { orientations.add(new double[] {0, 1}); }
        Point middle = new Point((s.x + e.x) / 2, (s.y + e.y) / 2);
        // Parallel edges keep their lane: the base carrier of a later lane is offset like its bend.
        double laneShift = route.lane * Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        List<Carrier> carriers = new ArrayList<>();
        for (int o = 0; o < orientations.size(); o++) {
            double ux = orientations.get(o)[0];
            double uy = orientations.get(o)[1];
            double[] size = EdgeLabelReservation.composite(route.edge.getLabels(), Math.abs(ux) >= Math.abs(uy),
                    margins.labelGap);
            double along = Math.abs(ux) * size[0] + Math.abs(uy) * size[1];
            double across = Math.abs(uy) * size[0] + Math.abs(ux) * size[1];
            double half = along / 2 + margins.edgeLabel;
            double unit = Math.max(clearance, across / 2 + margins.edgeLabel);
            double shift = Math.max(clearance, half);
            for (int k = 0; k <= DETOUR_OFFSETS; k++) {
                for (int sign = 0; sign < (k == 0 ? 1 : 2); sign++) {
                    double offset = k == 0 ? laneShift : (sign == 0 ? k : -k) * unit + laneShift;
                    for (int j = 0; j <= DETOUR_SHIFTS; j++) {
                        for (int jsign = 0; jsign < (j == 0 ? 1 : 2); jsign++) {
                            double tangent = (jsign == 0 ? j : -j) * shift;
                            double cx = middle.x - dy * offset + dx * tangent;
                            double cy = middle.y + dx * offset + dy * tangent;
                            carriers.add(new Carrier(new Point(cx - ux * half, cy - uy * half),
                                    new Point(cx + ux * half, cy + uy * half)));
                        }
                    }
                }
            }
            // Single-bend kinks: a carrier starting at one gateway and pointing along an axis.
            if (o > 0) {
                for (int sign = -1; sign <= 1; sign += 2) {
                    Point g = route.start.gateway;
                    carriers.add(new Carrier(g, new Point(g.x + sign * ux * 2 * half, g.y + sign * uy * 2 * half)));
                    g = route.end.gateway;
                    carriers.add(new Carrier(new Point(g.x + sign * ux * 2 * half, g.y + sign * uy * 2 * half), g));
                }
            }
        }
        List<Candidate> feasible = new ArrayList<>();
        for (int index = 0; index < carriers.size(); index++) {
            Carrier carrier = carriers.get(index);
            if (!visible(carrier.a, carrier.b, route.excluded)) { continue; }
            // Orient the carrier so that the route does not double back.
            boolean forward = distance(s, carrier.a) + distance(carrier.b, e)
                    <= distance(s, carrier.b) + distance(carrier.a, e);
            Point c1 = forward ? carrier.a : carrier.b;
            Point c2 = forward ? carrier.b : carrier.a;
            Endpoint start = route.source instanceof ElkPort ? route.start : endpoint(route.source, c1, route.sv);
            Endpoint end = route.target instanceof ElkPort ? route.end : endpoint(route.target, c2, route.tv);
            List<Point> skeleton = new ArrayList<>();
            skeleton.add(start.anchor);
            skeleton.add(start.gateway);
            skeleton.addAll(route.outgoing);
            skeleton.add(c1);
            skeleton.add(c2);
            skeleton.addAll(route.incoming);
            skeleton.add(end.gateway);
            skeleton.add(end.anchor);
            Candidate candidate = onSegment(route, c1, c2, skeleton.indexOf(c2), skeleton);
            if (candidate == null) { continue; }
            if (distance(c1, c2) < EPSILON) { continue; }
            candidate.order = index;
            candidate.start = start;
            candidate.end = end;
            candidate.carrier = new Segment(c1, c2);
            candidate.estimate = polylineLength(skeleton);
            // Anchor-to-gateway stubs lie inside their own inflated footprint and are never tested.
            boolean direct = true;
            int host = skeleton.indexOf(c2);
            for (int i = 2; i < skeleton.size() - 1 && direct; i++) {
                if (i == host || distance(skeleton.get(i - 1), skeleton.get(i)) < EPSILON) { continue; }
                direct = visible(skeleton.get(i - 1), skeleton.get(i), route.excluded);
            }
            if (direct) {
                candidate.points = skeleton;
                candidate.length = candidate.estimate;
            }
            feasible.add(candidate);
        }
        Collections.sort(feasible, (p, q) -> p.estimate < q.estimate - EPSILON ? -1
                : q.estimate < p.estimate - EPSILON ? 1 : Integer.compare(p.order, q.order));
        Candidate best = null;
        int searched = 0;
        for (Candidate candidate : feasible) {
            if (best != null && candidate.estimate > best.length + EPSILON) { break; }
            if (candidate.points == null) {
                if (searched >= DETOUR_SEARCHES) { continue; }
                searched++;
                try {
                    List<Point> points = new ArrayList<>();
                    points.add(candidate.start.anchor);
                    points.addAll(through(candidate.start.gateway, candidate.carrier.a, route.outgoing,
                            route.excluded));
                    List<Point> back = through(candidate.carrier.b, candidate.end.gateway, route.incoming,
                            route.excluded);
                    points.addAll(back);
                    points.add(candidate.end.anchor);
                    // The completed legs must not cut through the label they were routed for.
                    Rect guarded = candidate.rect.inflate(margins.edgeLabel);
                    boolean clean = true;
                    for (int i = 1; i < points.size() && clean; i++) {
                        if (points.get(i - 1) == candidate.carrier.a && points.get(i) == candidate.carrier.b) {
                            continue;
                        }
                        clean = !guarded.crosses(points.get(i - 1), points.get(i));
                    }
                    if (!clean) { continue; }
                    straighten(route, candidate, points);
                    candidate.points = points;
                    candidate.length = polylineLength(points);
                } catch (IllegalArgumentException blocked) {
                    continue;
                }
            }
            // A bend costs as much as one clearance of length; equal costs keep the earliest candidate.
            double cost = candidate.length + clearance * bendCount(candidate.points);
            double bestCost = best == null ? Double.POSITIVE_INFINITY
                    : best.length + clearance * bendCount(best.points);
            if (best == null || cost < bestCost - EPSILON
                    || Math.abs(cost - bestCost) <= EPSILON && candidate.order < best.order) {
                best = candidate;
            }
        }
        return best;
    }

    /**
     * Free endpoints of a completed detour face the first and last bend instead of the carrier, so
     * that the anchor stub is collinear with the leg it starts. Fixed ports keep their side.
     */
    private void straighten(final Route route, final Candidate candidate, final List<Point> points) {
        if (points.size() < 5) { return; }
        if (!(route.source instanceof ElkPort) && route.outgoing.isEmpty()) {
            Endpoint start = endpoint(route.source, points.get(2), route.sv);
            if (visible(start.gateway, points.get(2), route.excluded)) {
                points.set(0, start.anchor);
                points.set(1, start.gateway);
                candidate.start = start;
            }
        }
        if (!(route.target instanceof ElkPort) && route.incoming.isEmpty()) {
            int last = points.size() - 1;
            Endpoint end = endpoint(route.target, points.get(last - 2), route.tv);
            if (visible(points.get(last - 2), end.gateway, route.excluded)) {
                points.set(last, end.anchor);
                points.set(last - 1, end.gateway);
                candidate.end = end;
            }
        }
        simplify(points);
    }

    /** Stage 3: an exterior corridor beyond the nearest side of the drawing. */
    private Candidate corridor(final Route route) {
        List<Point> points = route.points;
        Point start = points.get(0);
        Point end = points.get(points.size() - 1);
        Point startGate = points.get(Math.min(1, points.size() - 1));
        Point endGate = points.get(Math.max(0, points.size() - 2));
        Set<ElkNode> excluded = route.excluded;
        double[] size = EdgeLabelReservation.composite(route.edge.getLabels(), true, margins.labelGap);
        double[] vertical = EdgeLabelReservation.composite(route.edge.getLabels(), false, margins.labelGap);
        Rect extent = outerExtent();
        Candidate best = null;
        for (int side = 0; side < 4; side++) {
            boolean horizontal = side < 2;
            double[] composite = horizontal ? size : vertical;
            double half = (horizontal ? composite[0] : composite[1]) / 2 + margins.edgeLabel;
            double across = horizontal ? composite[1] : composite[0];
            double coordinate;
            Point first;
            Point second;
            if (side == 0) { coordinate = extent.top - clearance - 8; }
            else if (side == 1) { coordinate = extent.bottom + clearance + 8; }
            else if (side == 2) { coordinate = extent.left - clearance - 8; }
            else { coordinate = extent.right + clearance + 8; }
            if (horizontal) {
                double middle = (start.x + end.x) / 2;
                first = new Point(middle - half, coordinate);
                second = new Point(middle + half, coordinate);
                if (start.x > end.x) { Point swap = first; first = second; second = swap; }
            } else {
                double middle = (start.y + end.y) / 2;
                first = new Point(coordinate, middle - half);
                second = new Point(coordinate, middle + half);
                if (start.y > end.y) { Point swap = first; first = second; second = swap; }
            }
            List<Point> via = new ArrayList<>();
            via.add(first);
            via.add(second);
            List<Point> rerouted;
            try {
                rerouted = through(startGate, endGate, via, excluded);
            } catch (IllegalArgumentException blocked) {
                continue;
            }
            List<Point> candidatePoints = new ArrayList<>();
            candidatePoints.add(start);
            candidatePoints.addAll(rerouted);
            candidatePoints.add(end);
            double cx = (first.x + second.x) / 2;
            double cy = (first.y + second.y) / 2;
            double offset = across / 2 + margins.edgeLabel;
            if (side == 0) { cy -= offset; } else if (side == 1) { cy += offset; }
            else if (side == 2) { cx -= offset; } else { cx += offset; }
            Rect rect = new Rect(cx - composite[0] / 2, cy - composite[1] / 2, cx + composite[0] / 2,
                    cy + composite[1] / 2, null, 0);
            Candidate candidate = new Candidate(rect, 0, margins.labelNode,
                    horizontal ? 1 : 0, horizontal ? 0 : 1, composite);
            candidate.points = candidatePoints;
            candidate.length = polylineLength(candidatePoints);
            candidate.order = side;
            if (best == null || candidate.length < best.length - EPSILON) { best = candidate; }
        }
        if (best == null) {
            throw new IllegalArgumentException("Edge " + route.edge.getIdentifier()
                    + ": no exterior label corridor; increase node spacing");
        }
        return best;
    }

    /** Positions every label of the group inside the composite rectangle and reserves it. */
    private void apply(final Candidate candidate, final List<ElkLabel> edgeLabels) {
        boolean sideBySide = Math.abs(candidate.ux) >= Math.abs(candidate.uy);
        double x = candidate.rect.left;
        double y = candidate.rect.top;
        for (ElkLabel label : edgeLabels) {
            double width = Math.max(0, label.getWidth());
            double height = Math.max(0, label.getHeight());
            double lx = sideBySide ? x : candidate.rect.left + (candidate.size[0] - width) / 2;
            double ly = sideBySide ? candidate.rect.top + (candidate.size[1] - height) / 2 : y;
            label.setLocation(lx, ly);
            reserveLabel(new Rect(lx, ly, lx + width, ly + height, null, 0));
            if (sideBySide) { x += width + margins.labelGap; } else { y += height + margins.labelGap; }
        }
    }

    private Rect outerExtent() {
        Rect extent = null;
        for (Rect rect : obstacles) {
            extent = extent == null ? new Rect(rect.left, rect.top, rect.right, rect.bottom, null, 0)
                    : new Rect(Math.min(extent.left, rect.left), Math.min(extent.top, rect.top),
                            Math.max(extent.right, rect.right), Math.max(extent.bottom, rect.bottom), null, 0);
        }
        if (extent == null) { extent = new Rect(0, 0, 0, 0, null, 0); }
        for (Segment segment : routedSegments) {
            extent = new Rect(Math.min(extent.left, Math.min(segment.a.x, segment.b.x)),
                    Math.min(extent.top, Math.min(segment.a.y, segment.b.y)),
                    Math.max(extent.right, Math.max(segment.a.x, segment.b.x)),
                    Math.max(extent.bottom, Math.max(segment.a.y, segment.b.y)), null, 0);
        }
        return extent;
    }

    private int crossingCount(final Point a, final Point b) {
        int count = 0;
        for (Segment segment : routedSegments) {
            if (side(a, b, segment.a) * side(a, b, segment.b) < -EPSILON
                    && side(segment.a, segment.b, a) * side(segment.a, segment.b, b) < -EPSILON) { count++; }
        }
        return count;
    }

    private static double side(final Point a, final Point b, final Point p) {
        return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    }

    private void reserveLabel(final Rect label) {
        // Later edges keep the edge-label spacing from placed labels.
        Rect obstacle = new Rect(label.left - margins.edgeLabel, label.top - margins.edgeLabel,
                label.right + margins.edgeLabel, label.bottom + margins.edgeLabel, null, margins.edgeLabel);
        obstacles.add(obstacle);
        index(obstacle);
        visibilityCache.clear();
    }

    private void index(final Rect rect) {
        for (int x = cell(rect.left); x <= cell(rect.right); x++) {
            for (int y = cell(rect.top); y <= cell(rect.bottom); y++) {
                cells.computeIfAbsent(x + ":" + y, key -> new ArrayList<>()).add(rect);
            }
        }
    }

    private static void simplify(final List<Point> points) {
        for (int i = points.size() - 2; i >= 0; i--) {
            if (distance(points.get(i), points.get(i + 1)) < EPSILON) { points.remove(i + 1); }
        }
        for (int i = points.size() - 2; i > 0; i--) {
            Point a = points.get(i - 1);
            Point b = points.get(i);
            Point c = points.get(i + 1);
            if (Math.abs(distance(a, b) + distance(b, c) - distance(a, c)) < EPSILON) { points.remove(i); }
        }
    }

    private static double polylineLength(final List<Point> points) {
        double length = 0;
        for (int i = 1; i < points.size(); i++) { length += distance(points.get(i - 1), points.get(i)); }
        return length;
    }

    private static int bendCount(final List<Point> points) {
        int bends = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            Point a = points.get(i - 1);
            Point b = points.get(i);
            Point c = points.get(i + 1);
            if (Math.abs(distance(a, b) + distance(b, c) - distance(a, c)) >= EPSILON) { bends++; }
        }
        return bends;
    }

    private int cell(final double coordinate) { return (int) Math.floor(coordinate / cellSize); }
    private static double distance(final Point a, final Point b) { return Math.hypot(a.x - b.x, a.y - b.y); }

    private static final class Endpoint {
        final Point anchor;
        final Point gateway;
        Endpoint(final Point anchor, final Point gateway) { this.anchor = anchor; this.gateway = gateway; }
    }
    private static final class Scope {
        final GeometryGraph graph;
        final double x;
        final double y;
        Scope(final GeometryGraph graph, final double x, final double y) { this.graph = graph; this.x = x; this.y = y; }
    }
    private static final class Point {
        final double x;
        final double y;
        Point(final double x, final double y) { this.x = x; this.y = y; }
    }
    private static final class Segment {
        final Point a;
        final Point b;
        Segment(final Point a, final Point b) { this.a = a; this.b = b; }
    }
    private static final class Carrier {
        final Point a;
        final Point b;
        Carrier(final Point a, final Point b) { this.a = a; this.b = b; }
    }
    /** Everything known about the edge currently being routed. */
    private static final class Route {
        final ElkEdge edge;
        final ElkConnectableShape source;
        final ElkConnectableShape target;
        final Vertex sv;
        final Vertex tv;
        final Set<ElkNode> excluded = new HashSet<>();
        List<Point> outgoing = new ArrayList<>();
        List<Point> incoming = new ArrayList<>();
        Endpoint start;
        Endpoint end;
        List<Point> points;
        int lane;
        Route(final ElkEdge edge, final ElkConnectableShape source, final ElkConnectableShape target,
                final Vertex sv, final Vertex tv) {
            this.edge = edge; this.source = source; this.target = target; this.sv = sv; this.tv = tv;
        }
    }
    /** A feasible label rectangle, optionally with the route that exposes it. */
    private static final class Candidate {
        final Rect rect;
        final int pending;
        final double margin;
        final double ux;
        final double uy;
        final double[] size;
        List<Point> points;
        double length;
        double estimate;
        int order;
        Endpoint start;
        Endpoint end;
        Segment carrier;
        Candidate(final Rect rect, final int pending, final double margin, final double ux, final double uy,
                final double[] size) {
            this.rect = rect; this.pending = pending; this.margin = margin; this.ux = ux; this.uy = uy; this.size = size;
        }
        boolean better(final Candidate other) {
            return pending < other.pending || pending == other.pending && margin > other.margin + EPSILON;
        }
    }
    private static final class Rect {
        final double left;
        final double top;
        final double right;
        final double bottom;
        final ElkNode node;
        /** Uniform clearance this rectangle was inflated by, relative to the real footprint. */
        final double inflation;
        Rect(final double left, final double top, final double right, final double bottom, final ElkNode node,
                final double inflation) {
            this.left = left; this.top = top; this.right = right; this.bottom = bottom; this.node = node;
            this.inflation = inflation;
        }
        Rect inflate(final double amount) {
            return new Rect(left - amount, top - amount, right + amount, bottom + amount, node, inflation + amount);
        }
        Rect deflate(final double amount) {
            return new Rect(left + amount, top + amount, right - amount, bottom - amount, node, inflation - amount);
        }
        boolean overlaps(final Rect other) {
            return left < other.right && right > other.left && top < other.bottom && bottom > other.top;
        }
        /** Euclidean distance between two rectangles; zero when they touch or overlap. */
        double gap(final Rect other) {
            double dx = Math.max(0, Math.max(left - other.right, other.left - right));
            double dy = Math.max(0, Math.max(top - other.bottom, other.top - bottom));
            if (overlaps(other)) { return -1; }
            return Math.hypot(dx, dy);
        }
        boolean crosses(final Point a, final Point b) {
            double low = 0;
            double high = 1;
            double[] origin = {a.x, a.y};
            double[] delta = {b.x - a.x, b.y - a.y};
            double[] min = {left + EPSILON, top + EPSILON};
            double[] max = {right - EPSILON, bottom - EPSILON};
            for (int axis = 0; axis < 2; axis++) {
                if (Math.abs(delta[axis]) < EPSILON) {
                    if (origin[axis] <= min[axis] || origin[axis] >= max[axis]) { return false; }
                } else {
                    double first = (min[axis] - origin[axis]) / delta[axis];
                    double last = (max[axis] - origin[axis]) / delta[axis];
                    low = Math.max(low, Math.min(first, last));
                    high = Math.min(high, Math.max(first, last));
                    if (low >= high) { return false; }
                }
            }
            return low < high;
        }
    }
}
