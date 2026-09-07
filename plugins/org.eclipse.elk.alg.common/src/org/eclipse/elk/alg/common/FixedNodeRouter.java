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

import org.eclipse.elk.alg.common.GeometryGraph.Vertex;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.ElkConnectableShape;
import org.eclipse.elk.graph.ElkEdge;
import org.eclipse.elk.graph.ElkEdgeSection;
import org.eclipse.elk.graph.ElkLabel;
import org.eclipse.elk.graph.ElkNode;
import org.eclipse.elk.graph.ElkPort;
import org.eclipse.elk.graph.util.ElkGraphUtil;

/** Deterministic polyline routing around fixed rectangular footprints. No node is moved. */
public final class FixedNodeRouter {
    private static final double EPSILON = 1e-8;
    private final GeometryGraph graph;
    private final List<Rect> obstacles = new ArrayList<>();
    private final Map<ElkNode, Rect> rectangles = new HashMap<>();
    private final Map<String, List<Rect>> cells = new HashMap<>();
    private final Map<String, Rect> visibilityCache = new HashMap<>();
    private Set<Rect> searchBlockers;
    private final List<Rect> labels = new ArrayList<>();
    private final List<Segment> routedSegments = new ArrayList<>();
    private final double clearance;
    private final double cellSize;

    public FixedNodeRouter(final ElkNode scope) {
        graph = new GeometryGraph(scope, false);
        clearance = Math.max(0, scope.getProperty(CoreOptions.SPACING_EDGE_NODE));
        double total = 0;
        List<Scope> scopes = new ArrayList<>();
        scopes.add(new Scope(graph, 0, 0));
        for (int i = 0; i < scopes.size(); i++) {
            Scope frame = scopes.get(i);
            for (Vertex vertex : frame.graph.vertices) {
                Rect rect = new Rect(vertex.x + vertex.left + frame.x - clearance,
                        vertex.y + vertex.top + frame.y - clearance,
                        vertex.x + vertex.right + frame.x + clearance,
                        vertex.y + vertex.bottom + frame.y + clearance, vertex.node);
                obstacles.add(rect);
                rectangles.put(vertex.node, rect);
                total += Math.max(rect.right - rect.left, rect.bottom - rect.top);
                if (!vertex.node.getChildren().isEmpty()) {
                    scopes.add(new Scope(new GeometryGraph(vertex.node, false),
                            frame.x + vertex.node.getX(), frame.y + vertex.node.getY()));
                }
            }
        }
        cellSize = Math.max(16, total / Math.max(1, obstacles.size()));
        for (Rect rect : obstacles) {
            for (int x = cell(rect.left); x <= cell(rect.right); x++) {
                for (int y = cell(rect.top); y <= cell(rect.bottom); y++) {
                    cells.computeIfAbsent(x + ":" + y, key -> new ArrayList<>()).add(rect);
                }
            }
        }
    }

    public void route() {
        List<ElkEdge> edges = new ArrayList<>(graph.graph.getContainedEdges());
        // Stable IDs for edges are useful even when node order is intentionally model order.
        Collections.sort(edges, Comparator.comparing(e -> e.getIdentifier() == null ? "" : e.getIdentifier()));
        Map<String, Integer> lanes = new HashMap<>();
        for (ElkEdge edge : edges) {
            if (edge.getSources().size() != 1 || edge.getTargets().size() != 1) {
                throw new IllegalArgumentException("Geometric polyline routing requires simple edges");
            }
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
            List<Point> points;
            if (GeometryGraph.endpointNode(source) == GeometryGraph.endpointNode(target)) {
                points = loop(source, target, sc, lane);
            } else {
                Endpoint start = endpoint(source, tc, sv);
                Endpoint end = endpoint(target, sc, tv);
                Set<ElkNode> excluded = new HashSet<>();
                excludeAncestors(source, excluded);
                excludeAncestors(target, excluded);
                List<Point> gateways = hierarchyGateways(source, tc);
                List<Point> incomingGateways = hierarchyGateways(target, sc);
                Collections.reverse(incomingGateways);
                gateways.addAll(incomingGateways);
                try {
                    points = through(start.gateway, end.gateway, gateways, excluded);
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
                                List<Point> path = through(candidateStart.gateway, candidateEnd.gateway, gateways, excluded);
                                double pathLength = distance(candidateStart.anchor, candidateStart.gateway)
                                        + distance(candidateEnd.anchor, candidateEnd.gateway);
                                for (int p = 1; p < path.size(); p++) { pathLength += distance(path.get(p - 1), path.get(p)); }
                                if (pathLength < bestLength) {
                                    bestPath = path;
                                    bestLength = pathLength;
                                    start = candidateStart;
                                    end = candidateEnd;
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
                    if (visible(a, middle, excluded) && visible(middle, b, excluded)) {
                        points.add(1, middle);
                    }
                }
                points.add(0, start.anchor);
                points.add(end.anchor);
            }
            placeLabels(edge, points);
            simplify(points);
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

    private List<Point> loop(final ElkConnectableShape source, final ElkConnectableShape target,
            final Point center, final int lane) {
        ElkNode node = GeometryGraph.endpointNode(source);
        Point nodeCenter = center(node);
        Rect rect = rectangles.get(node);
        if (rect == null) { throw new IllegalArgumentException("A scope self-loop needs an enclosing layout scope"); }
        double gap = clearance + (lane + 1) * Math.max(8, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        Endpoint start = endpoint(source, new Point(nodeCenter.x + node.getWidth(), nodeCenter.y), graph.endpoint(source));
        Endpoint end = endpoint(target, new Point(nodeCenter.x, nodeCenter.y - node.getHeight()), graph.endpoint(target));
        Set<ElkNode> excluded = new HashSet<>();
        excludeAncestors(source, excluded);
        double top = rect.top - gap;
        Point left = new Point(rect.left - gap, top);
        Point right = new Point(rect.right + gap, top);
        if (blockingRectangle(left, left, excluded) != null || blockingRectangle(right, right, excluded) != null) {
            for (Rect obstacle : obstacles) { top = Math.min(top, obstacle.top - gap); }
            left = new Point(rect.left - gap, top);
            right = new Point(rect.right + gap, top);
        }
        List<Point> via = new ArrayList<>();
        boolean fromLeft = start.gateway.x < nodeCenter.x;
        via.add(fromLeft ? left : right);
        via.add(fromLeft ? right : left);
        List<Point> result = new ArrayList<>();
        result.add(start.anchor);
        result.addAll(through(start.gateway, end.gateway, via, excluded));
        result.add(end.anchor);
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

    private void placeLabels(final ElkEdge edge, final List<Point> points) {
        if (edge.getLabels().size() > 1) {
            placeLabelGroup(edge, points);
            return;
        }
        for (ElkLabel label : edge.getLabels()) {
            boolean placed = false;
            List<Integer> segments = new ArrayList<>();
            for (int i = 1; i < points.size(); i++) { segments.add(i); }
            Collections.sort(segments, (a, b) -> Double.compare(distance(points.get(b - 1), points.get(b)),
                    distance(points.get(a - 1), points.get(a))));
            for (int segment : segments) {
                Point a = points.get(segment - 1);
                Point b = points.get(segment);
                double length = distance(a, b);
                if (length < EPSILON) { continue; }
                double ux = (b.x - a.x) / length;
                double uy = (b.y - a.y) / length;
                double projection = Math.abs(ux) * label.getWidth() + Math.abs(uy) * label.getHeight();
                if (length + EPSILON < projection + 4) { continue; }
                double offset = (Math.abs(uy) * label.getWidth() + Math.abs(ux) * label.getHeight()) / 2 + 2;
                for (int side : new int[] {-1, 1}) {
                    double x = (a.x + b.x - label.getWidth()) / 2 - side * uy * offset;
                    double y = (a.y + b.y - label.getHeight()) / 2 + side * ux * offset;
                    Rect candidate = new Rect(x, y, x + label.getWidth(), y + label.getHeight(), null);
                    if (labelFits(candidate)) {
                        label.setLocation(x, y);
                        reserveLabel(candidate);
                        placed = true;
                        break;
                    }
                }
                if (placed) { break; }
            }
            if (!placed) {
                // A wide label gets its own exterior horizontal corridor. Placement stays fixed.
                double top = outerTop();
                double y = top - label.getHeight() - clearance - 8;
                Point start = points.get(0);
                Point end = points.get(points.size() - 1);
                Point startGate = points.get(Math.min(1, points.size() - 1));
                Point endGate = points.get(Math.max(0, points.size() - 2));
                double middle = (start.x + end.x) / 2;
                Point left = new Point(middle - label.getWidth() / 2 - 4, y);
                Point right = new Point(middle + label.getWidth() / 2 + 4, y);
                List<Point> via = new ArrayList<>();
                via.add(start.x <= end.x ? left : right);
                via.add(start.x <= end.x ? right : left);
                Set<ElkNode> excluded = new HashSet<>();
                excludeAncestors(edge.getSources().get(0), excluded);
                excludeAncestors(edge.getTargets().get(0), excluded);
                List<Point> rerouted = through(startGate, endGate, via, excluded);
                points.clear();
                points.add(start);
                points.addAll(rerouted);
                points.add(end);
                Rect candidate = new Rect(middle - label.getWidth() / 2, y - label.getHeight() - 2,
                        middle + label.getWidth() / 2, y - 2, null);
                label.setLocation(candidate.left, candidate.top);
                reserveLabel(candidate);
            }
        }
    }

    private void placeLabelGroup(final ElkEdge edge, final List<Point> points) {
        double width = -8;
        double height = 0;
        for (ElkLabel label : edge.getLabels()) {
            width += label.getWidth() + 8;
            height = Math.max(height, label.getHeight());
        }
        double top = outerTop();
        double y = top - height - clearance - 8;
        Point start = points.get(0);
        Point end = points.get(points.size() - 1);
        double middle = (start.x + end.x) / 2;
        Point left = new Point(middle - width / 2 - 4, y);
        Point right = new Point(middle + width / 2 + 4, y);
        List<Point> via = new ArrayList<>();
        via.add(start.x <= end.x ? left : right);
        via.add(start.x <= end.x ? right : left);
        Set<ElkNode> excluded = new HashSet<>();
        excludeAncestors(edge.getSources().get(0), excluded);
        excludeAncestors(edge.getTargets().get(0), excluded);
        List<Point> rerouted = through(points.get(Math.min(1, points.size() - 1)),
                points.get(Math.max(0, points.size() - 2)), via, excluded);
        points.clear();
        points.add(start);
        points.addAll(rerouted);
        points.add(end);
        double x = middle - width / 2;
        for (ElkLabel label : edge.getLabels()) {
            label.setLocation(x, y - label.getHeight() - 2);
            reserveLabel(new Rect(x, label.getY(), x + label.getWidth(), y - 2, null));
            x += label.getWidth() + 8;
        }
    }

    private boolean labelFits(final Rect candidate) {
        for (Rect rect : obstacles) { if (candidate.overlaps(rect)) { return false; } }
        for (Segment segment : routedSegments) { if (candidate.crosses(segment.a, segment.b)) { return false; } }
        return true;
    }

    private double outerTop() {
        double top = 0;
        for (Rect rect : obstacles) { top = Math.min(top, rect.top); }
        for (Segment segment : routedSegments) { top = Math.min(top, Math.min(segment.a.y, segment.b.y)); }
        return top;
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
        labels.add(label);
        obstacles.add(label);
        for (int x = cell(label.left); x <= cell(label.right); x++) {
            for (int y = cell(label.top); y <= cell(label.bottom); y++) {
                cells.computeIfAbsent(x + ":" + y, key -> new ArrayList<>()).add(label);
            }
        }
        visibilityCache.clear();
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
    private static final class Rect {
        final double left;
        final double top;
        final double right;
        final double bottom;
        final ElkNode node;
        Rect(final double left, final double top, final double right, final double bottom, final ElkNode node) {
            this.left = left; this.top = top; this.right = right; this.bottom = bottom; this.node = node;
        }
        boolean overlaps(final Rect other) {
            return left < other.right && right > other.left && top < other.bottom && bottom > other.top;
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
