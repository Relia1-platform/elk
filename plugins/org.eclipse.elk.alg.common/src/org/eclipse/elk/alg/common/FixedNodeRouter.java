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
import org.eclipse.elk.core.math.KVector;
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
    /** Preferred label segment marker for the last segment, which survives route simplification. */
    private static final int LAST_SEGMENT = Integer.MAX_VALUE;
    /** Cells of dense bucket arrays kept around the obstacles, and the most cells they may hold. */
    private static final int GRID_MARGIN = 4;
    private static final int GRID_LIMIT = 1 << 18;

    private final GeometryGraph graph;
    private final List<Rect> obstacles = new ArrayList<>();
    private final Map<ElkNode, Rect> rectangles = new HashMap<>();
    /** Obstacle and segment buckets by cell: dense arrays around the obstacles, maps beyond them. */
    private final Map<Integer, List<Rect>> cells = new HashMap<>();
    private final List<Rect>[] rectGrid;
    private final List<Segment>[] segmentGrid;
    private final int gridMinX;
    private final int gridMinY;
    private final int gridWidth;
    private final int gridHeight;
    private final Map<SegmentKey, Rect> visibilityCache = new HashMap<>();
    /** Cache value for a free line of sight, so that one lookup answers a visibility query. */
    private static final Rect NO_BLOCKER = new Rect(0, 0, 0, 0, null, 0);
    private Set<Rect> searchBlockers;
    private final List<Segment> routedSegments = new ArrayList<>();
    /** Routed segments by cell, so crossing counts only test nearby connectors. */
    private final Map<Integer, List<Segment>> segmentCells = new HashMap<>();
    private final List<Segment> directLines = new ArrayList<>();
    private final Map<ElkEdge, List<Point>> prerouted = new HashMap<>();
    private final Map<ElkEdge, List<Segment>> fixedSegments = new HashMap<>();
    private final Map<ElkEdge, Integer> preferredSegments = new HashMap<>();
    private ElkEdge current;
    private boolean lenient;
    private boolean orthogonal;
    /** Spread anchors for free endpoints in orthogonal mode: edge id + role to {sideDx, sideDy, offset}. */
    private final Map<String, double[]> spreads = new HashMap<>();
    /** Sides chosen by probing in orthogonal mode: edge id + role to the outward side vector. */
    private final Map<String, double[]> chosenSides = new HashMap<>();
    private int routing;
    /** Query stamp for bucket walks: an object marked with the current stamp was already tested. */
    private int stamp;
    // Scratch space of the orthogonal search, reused across searches; an entry is valid only when
    // its stamp matches the current search. States are grid points times four entry directions,
    // grid edges are two per point (towards +x and towards +y).
    private int searchStamp;
    private int[] stateStamp = new int[0];
    private double[] stateCost = new double[0];
    private int[] stateBends = new int[0];
    private int[] stateCrossings = new int[0];
    private int[] statePrevious = new int[0];
    private boolean[] stateClosed = new boolean[0];
    private int[] edgeStamp = new int[0];
    private boolean[] edgeOpen = new boolean[0];
    private int[] edgeCrossings = new int[0];
    private double[] edgeLengths = new double[0];
    /** Binary heap of open search states, ordered by cost, bends, crossings, and state. */
    private double[] heapCost = new double[0];
    private int[] heapBends = new int[0];
    private int[] heapCrossings = new int[0];
    private int[] heapState = new int[0];
    private int heapSize;
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
        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE;
        int maxY = Integer.MIN_VALUE;
        for (Rect rect : obstacles) {
            minX = Math.min(minX, cell(rect.left));
            maxX = Math.max(maxX, cell(rect.right));
            minY = Math.min(minY, cell(rect.top));
            maxY = Math.max(maxY, cell(rect.bottom));
        }
        long width = obstacles.isEmpty() ? 0 : (long) maxX - minX + 1 + 2 * GRID_MARGIN;
        long height = obstacles.isEmpty() ? 0 : (long) maxY - minY + 1 + 2 * GRID_MARGIN;
        boolean dense = width > 0 && width * height <= GRID_LIMIT;
        gridMinX = dense ? minX - GRID_MARGIN : 0;
        gridMinY = dense ? minY - GRID_MARGIN : 0;
        gridWidth = dense ? (int) width : 0;
        gridHeight = dense ? (int) height : 0;
        rectGrid = newBuckets(gridWidth * gridHeight);
        segmentGrid = newBuckets(gridWidth * gridHeight);
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
                        addSegment(new Segment(previous, next));
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

    /**
     * Uses the given polyline, in scope coordinates, for an edge of this scope instead of routing
     * it. The polyline is never changed; its labels are placed on it and it is an obstacle for the
     * labels and a crossing for the routes of every other edge.
     */
    public void prerouted(final ElkEdge edge, final List<KVector> points) {
        prerouted(edge, points, -1);
    }

    /**
     * Like {@link #prerouted(ElkEdge, List)}, naming the segment that carries the labels first:
     * the segment ending at point {@code preferredSegment}, or -1 for the longest segment.
     */
    public void prerouted(final ElkEdge edge, final List<KVector> points, final int preferredSegment) {
        preferredSegments.put(edge, preferredSegment == points.size() - 1 ? LAST_SEGMENT : preferredSegment);
        List<Point> converted = new ArrayList<>();
        for (KVector point : points) { converted.add(new Point(point.x, point.y)); }
        prerouted.put(edge, converted);
        List<Segment> segments = new ArrayList<>();
        for (int i = 1; i < converted.size(); i++) { segments.add(new Segment(converted.get(i - 1), converted.get(i))); }
        fixedSegments.put(edge, segments);
    }

    /** With lenient routing, an edge that cannot be routed falls back to its direct segment. */
    public void setLenient(final boolean lenient) { this.lenient = lenient; }

    /**
     * Orthogonal connectors: free endpoints leave the center of the facing side, spread along it
     * when several edges share a side, routes follow a sparse orthogonal grid with a bend penalty,
     * overlapping parallel segments are nudged apart, and labels are placed after all routes exist.
     */
    public void setOrthogonal(final boolean orthogonal) { this.orthogonal = orthogonal; }

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
        if (orthogonal) {
            chooseSides(edges);
            computeSpreads(edges);
        }
        List<Route> pending = new ArrayList<>();
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
            current = edge;
            List<Point> fixed = prerouted.get(edge);
            if (fixed != null) {
                route.fixed = true;
                excludeAncestors(source, route.excluded);
                excludeAncestors(target, route.excluded);
                route.points = new ArrayList<>(fixed);
            } else if (GeometryGraph.endpointNode(source) == GeometryGraph.endpointNode(target)) {
                excludeAncestors(source, route.excluded);
                route.points = loop(edge, source, target, lane, route.excluded);
            } else {
                try {
                    free(route, sc, tc, lane);
                } catch (IllegalArgumentException failure) {
                    if (!lenient) { throw failure; }
                    // Route-only layouts of arbitrary positions keep going with a direct connector.
                    route.start = endpoint(source, tc, sv);
                    route.end = endpoint(target, sc, tv);
                    route.outgoing = new ArrayList<>();
                    route.incoming = new ArrayList<>();
                    route.points = new ArrayList<>();
                    route.points.add(route.start.anchor);
                    if (orthogonal) { route.points.add(new Point(route.end.anchor.x, route.start.anchor.y)); }
                    route.points.add(route.end.anchor);
                }
            }
            simplify(route.points);
            if (orthogonal) {
                // Labels that fit the route now are reserved so that later routes and self-loops
                // leave them room; they are placed again only when nudging moves the route.
                route.index = routing;
                pending.add(route);
                if (!route.fixed) { addSegments(route); }
                if (!edge.getLabels().isEmpty()) {
                    Candidate tentative = onRoute(route);
                    if (tentative != null) { route.reserved = apply(tentative, edge.getLabels()); }
                }
                continue;
            }
            labels(route);
            emit(route);
        }
        if (orthogonal) {
            // Tentative labels steered the routes; they are withdrawn so that nudging is free to
            // move segments, and every label is placed again on the final routes.
            for (Route route : pending) {
                if (route.reserved != null) { unreserve(route.reserved); route.reserved = null; }
            }
            nudge(pending);
            // Nudging replaced route points; rebuild the segment index from the final routes.
            List<Segment> inherited = new ArrayList<>();
            for (Segment segment : routedSegments) { if (segment.owner == null) { inherited.add(segment); } }
            reindexSegments(inherited);
            for (Route route : pending) { if (!route.fixed) { addSegments(route); } }
            for (Route route : pending) {
                routing = route.index;
                current = route.edge;
                if (!route.edge.getLabels().isEmpty()) {
                    if (!route.fixed) { removeSegments(route.edge); }
                    labels(route);
                    if (!route.fixed) { addSegments(route); }
                }
                emitSection(route);
            }
        }
        current = null;
    }

    private void labels(final Route route) {
        try {
            placeLabels(route);
        } catch (IllegalArgumentException failure) {
            if (!lenient) { throw failure; }
            Candidate fallback = bestEffort(route);
            if (fallback != null) { apply(fallback, route.edge.getLabels()); }
        }
        simplify(route.points);
    }

    private void addSegments(final Route route) {
        List<Point> points = route.points;
        for (int i = 1; i < points.size(); i++) {
            addSegment(new Segment(points.get(i - 1), points.get(i), route.edge));
        }
    }

    private void addSegment(final Segment segment) {
        routedSegments.add(segment);
        for (int x = cell(Math.min(segment.a.x, segment.b.x)); x <= cell(Math.max(segment.a.x, segment.b.x)); x++) {
            for (int y = cell(Math.min(segment.a.y, segment.b.y)); y <= cell(Math.max(segment.a.y, segment.b.y)); y++) {
                segmentBucket(x, y, true).add(segment);
            }
        }
    }

    private void removeSegments(final ElkEdge edge) {
        List<Segment> kept = new ArrayList<>();
        for (Segment segment : routedSegments) {
            if (segment.owner != edge) { kept.add(segment); continue; }
            for (int x = cell(Math.min(segment.a.x, segment.b.x)); x <= cell(Math.max(segment.a.x, segment.b.x)); x++) {
                for (int y = cell(Math.min(segment.a.y, segment.b.y)); y <= cell(Math.max(segment.a.y, segment.b.y)); y++) {
                    List<Segment> bucket = segmentBucket(x, y, false);
                    while (bucket != null && bucket.remove(segment)) { }
                }
            }
        }
        routedSegments.clear();
        routedSegments.addAll(kept);
    }

    /** Whether a routed segment of another edge, or an inherited one, crosses the rectangle. */
    private boolean crossesRoutedSegment(final Rect rect, final ElkEdge owner) {
        int visit = ++stamp;
        for (int x = cell(rect.left); x <= cell(rect.right); x++) {
            for (int y = cell(rect.top); y <= cell(rect.bottom); y++) {
                List<Segment> bucket = segmentBucket(x, y, false);
                if (bucket == null) { continue; }
                for (Segment segment : bucket) {
                    if (segment.mark == visit) { continue; }
                    segment.mark = visit;
                    if (segment.owner != owner && rect.crosses(segment.a, segment.b)) { return true; }
                }
            }
        }
        return false;
    }

    private void reindexSegments(final List<Segment> segments) {
        routedSegments.clear();
        segmentCells.clear();
        java.util.Arrays.fill(segmentGrid, null);
        for (Segment segment : segments) { addSegment(segment); }
    }

    private void emit(final Route route) {
        if (!route.fixed) {
            addSegments(route);
            // Polyline routes have no shared hyperedge junctions; discard geometry from earlier layouts.
            route.edge.setProperty(CoreOptions.JUNCTION_POINTS, null);
        }
        emitSection(route);
    }

    private void emitSection(final Route route) {
        ElkEdge edge = route.edge;
        List<Point> points = route.points;
        if (!route.fixed && orthogonal) { edge.setProperty(CoreOptions.JUNCTION_POINTS, null); }
        edge.getSections().clear();
        ElkEdgeSection section = ElkGraphUtil.createEdgeSection(edge);
        section.setIncomingShape(route.source);
        section.setOutgoingShape(route.target);
        section.setStartLocation(points.get(0).x, points.get(0).y);
        section.setEndLocation(points.get(points.size() - 1).x, points.get(points.size() - 1).y);
        for (int i = 1; i < points.size() - 1; i++) {
            ElkGraphUtil.createBendPoint(section, points.get(i).x, points.get(i).y);
        }
    }

    /** Routes an edge between two different nodes: gateways, hierarchy waypoints, lane offsets. */
    private void free(final Route route, final Point sc, final Point tc, final int lane) {
        ElkConnectableShape source = route.source;
        ElkConnectableShape target = route.target;
        Vertex sv = route.sv;
        Vertex tv = route.tv;
        ElkEdge edge = route.edge;
        route.start = spread(endpoint(source, toward(edge, "s", sc, tc), sv), edge, "s");
        route.end = spread(endpoint(target, toward(edge, "t", tc, sc), tv), edge, "t");
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
        if (lane > 0 && points.size() == 2 && !orthogonal) {
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
        if (orthogonal) { dejog(route.points, source, target); }
    }

    /** The direction an endpoint faces: the side chosen by probing, or the other endpoint. */
    private Point toward(final ElkEdge edge, final String role, final Point own, final Point other) {
        double[] side = chosenSides.get(edge.getIdentifier() + ":" + role);
        return side == null ? other : new Point(own.x + side[0], own.y + side[1]);
    }

    /**
     * Orthogonal mode, before any route exists: every free edge is probed from the sides facing
     * the other endpoint, and when that connector has to turn right after leaving a node, the
     * sides facing the first and last turn are tried. An alternative wins only by a clear margin,
     * so hubs keep their natural fan-out. Chosen sides drive the anchor spreading and the routes.
     */
    private void chooseSides(final List<ElkEdge> edges) {
        for (routing = 0; routing < edges.size(); routing++) {
            ElkEdge edge = edges.get(routing);
            if (prerouted.containsKey(edge)) { continue; }
            ElkConnectableShape source = edge.getSources().get(0);
            ElkConnectableShape target = edge.getTargets().get(0);
            if (GeometryGraph.endpointNode(source) == GeometryGraph.endpointNode(target)) { continue; }
            Point sc = center(source);
            Point tc = center(target);
            if (!hierarchyGateways(source, tc).isEmpty() || !hierarchyGateways(target, sc).isEmpty()) { continue; }
            current = edge;
            Set<ElkNode> excluded = new HashSet<>();
            excludeAncestors(source, excluded);
            excludeAncestors(target, excluded);
            Endpoint start = endpoint(source, tc, graph.endpoint(source));
            Endpoint end = endpoint(target, sc, graph.endpoint(target));
            if (straightPossible(source, target, start, end) && visible(start.gateway, end.gateway, excluded)) {
                // Facing sides that overlap get a straight connector once the anchors are aligned.
                chosenSides.put(edge.getIdentifier() + ":s", sideVector(source, start));
                chosenSides.put(edge.getIdentifier() + ":t", sideVector(target, end));
                continue;
            }
            List<Point> points;
            try {
                points = shortest(start.gateway, end.gateway, excluded);
            } catch (IllegalArgumentException blocked) {
                continue;
            }
            points.add(0, start.anchor);
            points.add(end.anchor);
            dejog(points, source, target);
            Endpoint[] best = reconsiderSides(source, target, start, end, points, excluded);
            chosenSides.put(edge.getIdentifier() + ":s", sideVector(source, best[0]));
            chosenSides.put(edge.getIdentifier() + ":t", sideVector(target, best[1]));
        }
        current = null;
    }

    /** Facing sides whose extents overlap enough to hold an aligned anchor on both nodes. */
    private boolean straightPossible(final ElkConnectableShape source, final ElkConnectableShape target,
            final Endpoint start, final Endpoint end) {
        if (source instanceof ElkPort || target instanceof ElkPort) { return false; }
        double[] a = sideVector(source, start);
        double[] b = sideVector(target, end);
        if (a[0] != -b[0] || a[1] != -b[1]) { return false; }
        boolean vertical = a[0] != 0;
        Point sc = center(source);
        Point tc = center(target);
        double sHalf = (vertical ? source.getHeight() : source.getWidth()) / 2;
        double tHalf = (vertical ? target.getHeight() : target.getWidth()) / 2;
        double sMine = vertical ? sc.y : sc.x;
        double tMine = vertical ? tc.y : tc.x;
        double low = Math.max(sMine - sHalf, tMine - tHalf);
        double high = Math.min(sMine + sHalf, tMine + tHalf);
        double margin = Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        return high - low >= 2 * margin;
    }

    private double[] sideVector(final ElkConnectableShape shape, final Endpoint endpoint) {
        Point c = center(shape);
        double ax = endpoint.anchor.x - c.x;
        double ay = endpoint.anchor.y - c.y;
        boolean horizontal = Math.abs(ax) >= Math.abs(ay);
        return new double[] {horizontal ? (ax >= 0 ? 1 : -1) : 0, horizontal ? 0 : (ay >= 0 ? 1 : -1)};
    }

    /**
     * Probes alternative sides for a connector that turns right after leaving a node. Returns the
     * start and end endpoints of the cheapest connector by length plus bend penalty; an alternative
     * must beat the facing sides by half a bend penalty.
     */
    private Endpoint[] reconsiderSides(final ElkConnectableShape source, final ElkConnectableShape target,
            final Endpoint start, final Endpoint end, final List<Point> points, final Set<ElkNode> excluded) {
        Endpoint[] result = {start, end};
        if (bendCount(points) < 2 || points.size() < 4) { return result; }
        double bendPenalty = 2 * Math.max(clearance, 4);
        Point altStart = alternative(source, start, points.get(1), points.get(2));
        Point altEnd = alternative(target, end, points.get(points.size() - 2), points.get(points.size() - 3));
        double bestCost = polylineLength(points) + bendPenalty * bendCount(points) - bendPenalty / 2;
        for (int option = 1; option < 4; option++) {
            boolean useStart = (option & 1) != 0;
            boolean useEnd = (option & 2) != 0;
            if (useStart && altStart == null || useEnd && altEnd == null) { continue; }
            Endpoint candidateStart = useStart ? endpoint(source, altStart, graph.endpoint(source)) : start;
            Endpoint candidateEnd = useEnd ? endpoint(target, altEnd, graph.endpoint(target)) : end;
            try {
                List<Point> candidate = shortest(candidateStart.gateway, candidateEnd.gateway, excluded);
                candidate.add(0, candidateStart.anchor);
                candidate.add(candidateEnd.anchor);
                dejog(candidate, source, target);
                double cost = polylineLength(candidate) + bendPenalty * bendCount(candidate);
                if (cost < bestCost - EPSILON) {
                    bestCost = cost;
                    result[0] = candidateStart;
                    result[1] = candidateEnd;
                }
            } catch (IllegalArgumentException blocked) {
                // Keep the connector from the facing sides.
            }
        }
        return result;
    }

    /**
     * A point beyond the side of the shape that faces the direction of a route's first turn, or
     * null when that is the side the route already uses or the endpoint is a port.
     */
    private Point alternative(final ElkConnectableShape shape, final Endpoint current, final Point from,
            final Point to) {
        if (shape instanceof ElkPort) { return null; }
        Point c = center(shape);
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        if (Math.abs(dx) < EPSILON && Math.abs(dy) < EPSILON) { return null; }
        boolean horizontal = Math.abs(dx) >= Math.abs(dy);
        double sx = horizontal ? (dx > 0 ? 1 : -1) : 0;
        double sy = horizontal ? 0 : (dy > 0 ? 1 : -1);
        double ax = current.anchor.x - c.x;
        double ay = current.anchor.y - c.y;
        boolean currentHorizontal = Math.abs(ax) >= Math.abs(ay);
        double cx = currentHorizontal ? (ax > 0 ? 1 : -1) : 0;
        double cy = currentHorizontal ? 0 : (ay > 0 ? 1 : -1);
        if (cx == sx && cy == sy) { return null; }
        return new Point(c.x + sx, c.y + sy);
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
            if (orthogonal) {
                // Orthogonal connectors leave through the center of the side facing the other end.
                if (Math.abs(dx) >= Math.abs(dy)) { dx = dx >= 0 ? 1 : -1; dy = 0; } else { dy = dy >= 0 ? 1 : -1; dx = 0; }
            }
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
        if (orthogonal) { return sideLoop(edge, source, target, rect, gap, excluded); }
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

    /**
     * Orthogonal self-loops hang on one side of the node: they leave that side a little below or
     * right of its center, run out by the loop gap, turn, and come back into the same side the same
     * distance above or left of the center. Labels sit beside the far segment, outside the loop.
     * Sides are tried east, south, west, north, preferring free sides that block no pending edge.
     */
    private List<Point> sideLoop(final ElkEdge edge, final ElkConnectableShape source,
            final ElkConnectableShape target, final Rect rect, final double gap, final Set<ElkNode> excluded) {
        ElkNode node = GeometryGraph.endpointNode(source);
        Point c = center(node);
        double edgeGap = Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        List<Object[]> candidates = new ArrayList<>();
        for (int side = 0; side < 4; side++) {
            boolean vertical = side == 0 || side == 2;
            double halfSide = (vertical ? node.getHeight() : node.getWidth()) / 2;
            double extent = loopExtent(edge, !vertical);
            double delta = Math.max(edgeGap, extent / 2);
            delta = Math.min(delta, Math.max(2, halfSide - 2));
            Point toward = side == 0 ? new Point(c.x + node.getWidth(), c.y) : side == 1 ? new Point(c.x, c.y + node.getHeight())
                    : side == 2 ? new Point(c.x - node.getWidth(), c.y) : new Point(c.x, c.y - node.getHeight());
            double tx = vertical ? 0 : 1;
            double ty = vertical ? 1 : 0;
            Endpoint centerStart = endpoint(source, toward, graph.endpoint(source));
            Endpoint centerEnd = endpoint(target, toward, graph.endpoint(target));
            Endpoint start = new Endpoint(new Point(centerStart.anchor.x + tx * delta, centerStart.anchor.y + ty * delta),
                    new Point(centerStart.gateway.x + tx * delta, centerStart.gateway.y + ty * delta));
            Endpoint end = new Endpoint(new Point(centerEnd.anchor.x - tx * delta, centerEnd.anchor.y - ty * delta),
                    new Point(centerEnd.gateway.x - tx * delta, centerEnd.gateway.y - ty * delta));
            double far = side == 0 ? rect.right + gap : side == 1 ? rect.bottom + gap : side == 2 ? rect.left - gap : rect.top - gap;
            List<Point> via = new ArrayList<>();
            via.add(vertical ? new Point(far, start.gateway.y) : new Point(start.gateway.x, far));
            via.add(vertical ? new Point(far, end.gateway.y) : new Point(end.gateway.x, far));
            boolean blocked = false;
            for (Point point : via) { blocked |= blockingRectangle(point, point, excluded) != null; }
            int pending = 0;
            Point previous = start.gateway;
            for (Point point : via) { pending = pendingCrossings(previous, point, pending); previous = point; }
            pending = pendingCrossings(previous, end.gateway, pending);
            candidates.add(new Object[] {blocked ? 1 : 0, pending, side, start, end, via});
        }
        Collections.sort(candidates, (p, q) -> {
            int byBlocked = Integer.compare((Integer) p[0], (Integer) q[0]);
            if (byBlocked != 0) { return byBlocked; }
            int byPending = Integer.compare((Integer) p[1], (Integer) q[1]);
            return byPending != 0 ? byPending : Integer.compare((Integer) p[2], (Integer) q[2]);
        });
        IllegalArgumentException failure = null;
        for (Object[] candidate : candidates) {
            Endpoint start = (Endpoint) candidate[3];
            Endpoint end = (Endpoint) candidate[4];
            @SuppressWarnings("unchecked")
            List<Point> via = (List<Point>) candidate[5];
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
        if (orthogonal) { return orthogonalShortest(start, end, excluded); }
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
        SegmentKey key = null;
        if (excluded.isEmpty()) {
            key = new SegmentKey(a, b);
            Rect cached = visibilityCache.get(key);
            if (cached != null) {
                if (cached != NO_BLOCKER && searchBlockers != null) { searchBlockers.add(cached); }
                return cached == NO_BLOCKER;
            }
        }
        Rect blocker = blockingRectangle(a, b, excluded);
        if (key != null) {
            if (visibilityCache.size() >= 50000) { visibilityCache.clear(); }
            visibilityCache.put(key, blocker == null ? NO_BLOCKER : blocker);
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
            int visit = ++stamp;
            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    List<Rect> bucket = rectBucket(x, y, false);
                    if (bucket == null) { continue; }
                    for (Rect rect : bucket) {
                        if (rect.mark == visit) { continue; }
                        rect.mark = visit;
                        if (!excluded.contains(rect.node) && rect.crosses(a, b)) { return rect; }
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
        if (best == null && route.fixed) { best = bestEffort(route); }
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
        // A pre-routed connector names the segment that is its own, such as the drop of a bus.
        Integer preferred = route.fixed ? preferredSegments.get(route.edge) : null;
        if (preferred != null && preferred == LAST_SEGMENT) { preferred = points.size() - 1; }
        if (preferred != null && preferred >= 1 && preferred < points.size() && segments.remove(preferred)) {
            segments.add(0, preferred);
        }
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
        int visit = ++stamp;
        for (int x = cell(region.left); x <= cell(region.right); x++) {
            for (int y = cell(region.top); y <= cell(region.bottom); y++) {
                List<Rect> bucket = rectBucket(x, y, false);
                if (bucket == null) { continue; }
                for (Rect obstacle : bucket) {
                    if (obstacle.mark == visit) { continue; }
                    obstacle.mark = visit;
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
        if (crossesRoutedSegment(guarded, route.edge)) { return null; }
        for (Map.Entry<ElkEdge, List<Segment>> entry : fixedSegments.entrySet()) {
            if (entry.getKey() == route.edge) { continue; }
            for (Segment segment : entry.getValue()) {
                if (guarded.crosses(segment.a, segment.b)) { return null; }
            }
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
        if (!orthogonal || Math.abs(dy) < 1e-6 || Math.abs(dx) < 1e-6) { orientations.add(new double[] {dx, dy}); }
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
                Point p = skeleton.get(i - 1);
                Point q = skeleton.get(i);
                direct = (!orthogonal || Math.abs(p.x - q.x) < EPSILON || Math.abs(p.y - q.y) < EPSILON)
                        && visible(p, q, route.excluded);
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
            if (axisAligned(start.gateway, points.get(2)) && visible(start.gateway, points.get(2), route.excluded)) {
                points.set(0, start.anchor);
                points.set(1, start.gateway);
                candidate.start = start;
            }
        }
        if (!(route.target instanceof ElkPort) && route.incoming.isEmpty()) {
            int last = points.size() - 1;
            Endpoint end = endpoint(route.target, points.get(last - 2), route.tv);
            if (axisAligned(points.get(last - 2), end.gateway) && visible(points.get(last - 2), end.gateway, route.excluded)) {
                points.set(last, end.anchor);
                points.set(last - 1, end.gateway);
                candidate.end = end;
            }
        }
        simplify(points);
    }

    /** A label beside the middle of the longest segment, accepting conflicts; the route is kept. */
    private Candidate bestEffort(final Route route) {
        List<Point> points = route.points;
        if (points.size() < 2) { return null; }
        int host = 1;
        double longest = -1;
        for (int i = 1; i < points.size(); i++) {
            double length = distance(points.get(i - 1), points.get(i));
            if (length > longest) { longest = length; host = i; }
        }
        Point a = points.get(host - 1);
        Point b = points.get(host);
        double length = Math.max(EPSILON, distance(a, b));
        double ux = (b.x - a.x) / length;
        double uy = (b.y - a.y) / length;
        double[] size = EdgeLabelReservation.composite(route.edge.getLabels(), Math.abs(ux) >= Math.abs(uy),
                margins.labelGap);
        double across = Math.abs(uy) * size[0] + Math.abs(ux) * size[1];
        double offset = inline ? 0 : across / 2 + margins.edgeLabel;
        double cx = (a.x + b.x) / 2 + uy * offset;
        double cy = (a.y + b.y) / 2 - ux * offset;
        Rect rect = new Rect(cx - size[0] / 2, cy - size[1] / 2, cx + size[0] / 2, cy + size[1] / 2, null, 0);
        return new Candidate(rect, 0, 0, ux, uy, size);
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
            } else {
                double middle = (start.y + end.y) / 2;
                first = new Point(coordinate, middle - half);
                second = new Point(coordinate, middle + half);
            }
            // Traverse the corridor in the direction that does not double back on itself.
            if (distance(startGate, first) + distance(second, endGate) > distance(startGate, second) + distance(first, endGate)) {
                Point swap = first; first = second; second = swap;
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
    private List<Rect> apply(final Candidate candidate, final List<ElkLabel> edgeLabels) {
        boolean sideBySide = Math.abs(candidate.ux) >= Math.abs(candidate.uy);
        double x = candidate.rect.left;
        double y = candidate.rect.top;
        List<Rect> reserved = new ArrayList<>();
        for (ElkLabel label : edgeLabels) {
            double width = Math.max(0, label.getWidth());
            double height = Math.max(0, label.getHeight());
            double lx = sideBySide ? x : candidate.rect.left + (candidate.size[0] - width) / 2;
            double ly = sideBySide ? candidate.rect.top + (candidate.size[1] - height) / 2 : y;
            label.setLocation(lx, ly);
            reserved.add(reserveLabel(new Rect(lx, ly, lx + width, ly + height, null, 0)));
            if (sideBySide) { x += width + margins.labelGap; } else { y += height + margins.labelGap; }
        }
        return reserved;
    }

    /** Withdraws tentative label obstacles before their labels are placed again. */
    private void unreserve(final List<Rect> reserved) {
        for (Rect rect : reserved) {
            obstacles.remove(rect);
            for (int x = cell(rect.left); x <= cell(rect.right); x++) {
                for (int y = cell(rect.top); y <= cell(rect.bottom); y++) {
                    List<Rect> bucket = rectBucket(x, y, false);
                    while (bucket != null && bucket.remove(rect)) { }
                }
            }
        }
        visibilityCache.clear();
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
        int minX = cell(Math.min(a.x, b.x));
        int maxX = cell(Math.max(a.x, b.x));
        int minY = cell(Math.min(a.y, b.y));
        int maxY = cell(Math.max(a.y, b.y));
        if ((long) (maxX - minX + 1) * (maxY - minY + 1) > routedSegments.size() * 4L) {
            for (Segment segment : routedSegments) {
                if (segment.owner != null && segment.owner == current) { continue; }
                if (side(a, b, segment.a) * side(a, b, segment.b) < -EPSILON
                        && side(segment.a, segment.b, a) * side(segment.a, segment.b, b) < -EPSILON) { count++; }
            }
        } else {
            int visit = ++stamp;
            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    List<Segment> bucket = segmentBucket(x, y, false);
                    if (bucket == null) { continue; }
                    for (Segment segment : bucket) {
                        if (segment.mark == visit) { continue; }
                        segment.mark = visit;
                        if (segment.owner != null && segment.owner == current) { continue; }
                        if (side(a, b, segment.a) * side(a, b, segment.b) < -EPSILON
                                && side(segment.a, segment.b, a) * side(segment.a, segment.b, b) < -EPSILON) { count++; }
                    }
                }
            }
        }
        for (Map.Entry<ElkEdge, List<Segment>> entry : fixedSegments.entrySet()) {
            if (entry.getKey() == current) { continue; }
            for (Segment segment : entry.getValue()) {
                if (side(a, b, segment.a) * side(a, b, segment.b) < -EPSILON
                        && side(segment.a, segment.b, a) * side(segment.a, segment.b, b) < -EPSILON) { count++; }
            }
        }
        return count;
    }

    private static double side(final Point a, final Point b, final Point p) {
        return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    }

    private Rect reserveLabel(final Rect label) {
        // Later edges keep the edge-label spacing from placed labels.
        Rect obstacle = new Rect(label.left - margins.edgeLabel, label.top - margins.edgeLabel,
                label.right + margins.edgeLabel, label.bottom + margins.edgeLabel, null, margins.edgeLabel);
        obstacles.add(obstacle);
        index(obstacle);
        visibilityCache.clear();
        return obstacle;
    }

    private void index(final Rect rect) {
        for (int x = cell(rect.left); x <= cell(rect.right); x++) {
            for (int y = cell(rect.top); y <= cell(rect.bottom); y++) {
                rectBucket(x, y, true).add(rect);
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

    /** In orthogonal mode a leg must be axis-aligned; polyline mode accepts any straight leg. */
    private boolean axisAligned(final Point a, final Point b) {
        return !orthogonal || Math.abs(a.x - b.x) < EPSILON || Math.abs(a.y - b.y) < EPSILON;
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

    // ---------------------------------------------------------------------------------------------
    // Orthogonal connectors

    /** Assigns every free endpoint of every edge a side and an offset along it, evenly spread. */
    private void computeSpreads(final List<ElkEdge> edges) {
        Map<ElkConnectableShape, List<List<Object[]>>> sides = new HashMap<>();
        for (ElkEdge edge : edges) {
            if (prerouted.containsKey(edge)) { continue; }
            ElkConnectableShape source = edge.getSources().get(0);
            ElkConnectableShape target = edge.getTargets().get(0);
            if (GeometryGraph.endpointNode(source) == GeometryGraph.endpointNode(target)) { continue; }
            for (int role = 0; role < 2; role++) {
                ElkConnectableShape shape = role == 0 ? source : target;
                ElkConnectableShape other = role == 0 ? target : source;
                if (shape instanceof ElkPort) { continue; }
                Point c = center(shape);
                Point o = center(other);
                double dx = o.x - c.x;
                double dy = o.y - c.y;
                double[] chosen = chosenSides.get(edge.getIdentifier() + ":" + (role == 0 ? "s" : "t"));
                if (chosen != null) { dx = chosen[0]; dy = chosen[1]; }
                int side = Math.abs(dx) >= Math.abs(dy) ? (dx >= 0 ? 1 : 3) : (dy >= 0 ? 2 : 0);
                List<List<Object[]>> perSide = sides.get(shape);
                if (perSide == null) {
                    perSide = new ArrayList<>();
                    for (int i = 0; i < 4; i++) { perSide.add(new ArrayList<>()); }
                    sides.put(shape, perSide);
                }
                double along = side == 1 || side == 3 ? o.y : o.x;
                perSide.get(side).add(new Object[] {edge, role == 0 ? "s" : "t", along});
            }
        }
        double gap = Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        for (Map.Entry<ElkConnectableShape, List<List<Object[]>>> entry : sides.entrySet()) {
            ElkConnectableShape shape = entry.getKey();
            for (int side = 0; side < 4; side++) {
                List<Object[]> list = entry.getValue().get(side);
                if (list.isEmpty()) { continue; }
                Collections.sort(list, (p, q) -> {
                    int byAlong = Double.compare((Double) p[2], (Double) q[2]);
                    if (byAlong != 0) { return byAlong; }
                    String pid = ((ElkEdge) p[0]).getIdentifier();
                    String qid = ((ElkEdge) q[0]).getIdentifier();
                    int byId = (pid == null ? "" : pid).compareTo(qid == null ? "" : qid);
                    return byId != 0 ? byId : ((String) p[1]).compareTo((String) q[1]);
                });
                double length = side == 1 || side == 3 ? shape.getHeight() : shape.getWidth();
                int n = list.size();
                double step = Math.min(2 * gap, length / (n + 1));
                double sideDx = side == 1 ? 1 : side == 3 ? -1 : 0;
                double sideDy = side == 2 ? 1 : side == 0 ? -1 : 0;
                for (int i = 0; i < n; i++) {
                    Object[] item = list.get(i);
                    double offset = (i - (n - 1) / 2.0) * step;
                    if (n == 1) { offset = aligned(shape, (ElkEdge) item[0], (String) item[1], side); }
                    spreads.put(((ElkEdge) item[0]).getIdentifier() + ":" + item[1], new double[] {sideDx, sideDy, offset});
                }
            }
        }
    }

    /**
     * A lone connector on a side runs straight when the two nodes overlap along that side: its
     * anchor moves to the middle of the overlap, kept inside the side, instead of the side center.
     */
    private double aligned(final ElkConnectableShape shape, final ElkEdge edge, final String role, final int side) {
        ElkConnectableShape other = "s".equals(role) ? edge.getTargets().get(0) : edge.getSources().get(0);
        if (other instanceof ElkPort) { return 0; }
        Point c = center(shape);
        Point o = center(other);
        boolean vertical = side == 1 || side == 3;
        double mine = vertical ? c.y : c.x;
        double half = (vertical ? shape.getHeight() : shape.getWidth()) / 2;
        double theirs = vertical ? o.y : o.x;
        double theirHalf = (vertical ? other.getHeight() : other.getWidth()) / 2;
        double low = Math.max(mine - half, theirs - theirHalf);
        double high = Math.min(mine + half, theirs + theirHalf);
        if (high - low < EPSILON) { return 0; }
        double target = (low + high) / 2;
        double margin = Math.min(half, Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE)));
        return Math.max(-(half - margin), Math.min(half - margin, target - mine));
    }

    /** Moves a side-center endpoint along its side by the spread offset computed for this edge. */
    private Endpoint spread(final Endpoint endpoint, final ElkEdge edge, final String role) {
        if (!orthogonal) { return endpoint; }
        double[] spreadOffset = spreads.get(edge.getIdentifier() + ":" + role);
        if (spreadOffset == null || Math.abs(spreadOffset[2]) < EPSILON) { return endpoint; }
        // The tangent of a horizontal side is x, of a vertical side y.
        double tx = spreadOffset[1] != 0 ? 1 : 0;
        double ty = spreadOffset[0] != 0 ? 1 : 0;
        return new Endpoint(new Point(endpoint.anchor.x + tx * spreadOffset[2], endpoint.anchor.y + ty * spreadOffset[2]),
                new Point(endpoint.gateway.x + tx * spreadOffset[2], endpoint.gateway.y + ty * spreadOffset[2]));
    }

    /**
     * Removes a jog shorter than the edge spacing right after the first or before the last stub
     * by sliding that anchor along its side, as long as the anchor stays on the side. Such jogs
     * come from anchors that are a few pixels off the line the connector needs, and they cost two
     * bends that would otherwise distort side probing and nudging.
     */
    private void dejog(final List<Point> points, final ElkConnectableShape source, final ElkConnectableShape target) {
        double limit = Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        for (int attempt = 0; attempt < 2; attempt++) {
            simplify(points);
            boolean changed = false;
            if (!(source instanceof ElkPort) && points.size() >= 4) { changed |= dejogEnd(points, source, limit, false); }
            if (!(target instanceof ElkPort) && points.size() >= 4) { changed |= dejogEnd(points, target, limit, true); }
            if (!changed) { break; }
        }
    }

    private boolean dejogEnd(final List<Point> points, final ElkConnectableShape shape, final double limit,
            final boolean atEnd) {
        int n = points.size();
        Point anchor = points.get(atEnd ? n - 1 : 0);
        Point gateway = points.get(atEnd ? n - 2 : 1);
        Point jog = points.get(atEnd ? n - 3 : 2);
        Point beyond = points.get(atEnd ? n - 4 : 3);
        boolean stubVertical = Math.abs(anchor.x - gateway.x) < EPSILON;
        double dx = jog.x - gateway.x;
        double dy = jog.y - gateway.y;
        // The jog must be perpendicular to the stub and the next segment parallel to it.
        if (stubVertical ? Math.abs(dy) > EPSILON || Math.abs(dx) >= limit : Math.abs(dx) > EPSILON || Math.abs(dy) >= limit) {
            return false;
        }
        if (stubVertical ? Math.abs(beyond.x - jog.x) > EPSILON : Math.abs(beyond.y - jog.y) > EPSILON) { return false; }
        Point c = center(shape);
        double half = (stubVertical ? shape.getWidth() : shape.getHeight()) / 2;
        double along = stubVertical ? anchor.x + dx - c.x : anchor.y + dy - c.y;
        if (Math.abs(along) > half - 2) { return false; }
        points.set(atEnd ? n - 1 : 0, new Point(anchor.x + dx, anchor.y + dy));
        points.set(atEnd ? n - 2 : 1, new Point(gateway.x + dx, gateway.y + dy));
        points.remove(atEnd ? n - 3 : 2);
        return true;
    }

    /** Adaptive orthogonal search: the grid grows with the obstacles that blocked earlier probes. */
    private List<Point> orthogonalShortest(final Point start, final Point end, final Set<ElkNode> excluded) {
        Set<Rect> active = new LinkedHashSet<>();
        for (int pass = 0; pass <= obstacles.size(); pass++) {
            Set<Rect> discovered = new LinkedHashSet<>();
            searchBlockers = discovered;
            List<Point> result = orthogonalSearch(start, end, excluded, active);
            searchBlockers = null;
            if (!active.addAll(discovered)) {
                if (result != null) { return result; }
                break;
            }
        }
        throw new IllegalArgumentException("No orthogonal corridor from (" + start.x + ", " + start.y
                + ") to (" + end.x + ", " + end.y + "); increase node spacing");
    }

    private static void addCoordinate(final List<Double> coordinates, final double value) {
        for (Double existing : coordinates) { if (Math.abs(existing - value) < EPSILON) { return; } }
        coordinates.add(value);
    }

    private static int indexOf(final List<Double> coordinates, final double value) {
        for (int i = 0; i < coordinates.size(); i++) { if (Math.abs(coordinates.get(i) - value) < EPSILON) { return i; } }
        return -1;
    }

    /** Dijkstra over the grid spanned by the endpoints and the active obstacle borders. */
    private List<Point> orthogonalSearch(final Point start, final Point end, final Set<ElkNode> excluded,
            final Set<Rect> active) {
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();
        addCoordinate(xs, start.x); addCoordinate(xs, end.x);
        addCoordinate(ys, start.y); addCoordinate(ys, end.y);
        for (Rect rect : active) {
            if (excluded.contains(rect.node)) { continue; }
            addCoordinate(xs, rect.left); addCoordinate(xs, rect.right);
            addCoordinate(ys, rect.top); addCoordinate(ys, rect.bottom);
        }
        Collections.sort(xs);
        Collections.sort(ys);
        int width = xs.size();
        int height = ys.size();
        int count = width * height;
        int startIndex = indexOf(xs, start.x) * height + indexOf(ys, start.y);
        int endIndex = indexOf(xs, end.x) * height + indexOf(ys, end.y);
        double bendPenalty = 2 * Math.max(clearance, 4);
        double crossingPenalty = Math.max(clearance, 4) / 2;
        // A state is a grid point plus the direction it was entered from: 0 +x, 1 -x, 2 +y, 3 -y.
        int stamp = ++searchStamp;
        ensureSearchCapacity(count);
        heapSize = 0;
        for (int direction = 0; direction < 4; direction++) {
            int state = startIndex * 4 + direction;
            touch(state, stamp);
            stateCost[state] = 0;
            push(0, 0, 0, state);
        }
        int reached = -1;
        while (heapSize > 0) {
            double itemCost = heapCost[0];
            int state = heapState[0];
            pop();
            if (stateClosed[state] || itemCost > stateCost[state] + EPSILON) { continue; }
            stateClosed[state] = true;
            int point = state / 4;
            int direction = state % 4;
            if (point == endIndex) { reached = state; break; }
            int i = point / height;
            int j = point % height;
            for (int next = 0; next < 4; next++) {
                int ni = i + (next == 0 ? 1 : next == 1 ? -1 : 0);
                int nj = j + (next == 2 ? 1 : next == 3 ? -1 : 0);
                if (ni < 0 || nj < 0 || ni >= width || nj >= height) { continue; }
                int neighbor = ni * height + nj;
                int nextState = neighbor * 4 + next;
                touch(nextState, stamp);
                if (stateClosed[nextState]) { continue; }
                // Every grid edge is tested once per search, whichever state reaches it first.
                int edge = 2 * (next == 0 || next == 2 ? point : neighbor) + (next < 2 ? 0 : 1);
                if (edgeStamp[edge] != stamp) {
                    edgeStamp[edge] = stamp;
                    Point a = new Point(xs.get(i), ys.get(j));
                    Point b = new Point(xs.get(ni), ys.get(nj));
                    boolean open = visible(a, b, excluded);
                    edgeOpen[edge] = open;
                    if (open) {
                        edgeCrossings[edge] = crossingCount(a, b);
                        edgeLengths[edge] = distance(a, b);
                    }
                }
                if (!edgeOpen[edge]) { continue; }
                int crossed = edgeCrossings[edge];
                double candidate = stateCost[state] + edgeLengths[edge] + (next != direction ? bendPenalty : 0)
                        + crossingPenalty * crossed;
                int candidateBends = stateBends[state] + (next != direction ? 1 : 0);
                int candidateCrossings = stateCrossings[state] + crossed;
                if (candidate < stateCost[nextState] - EPSILON || Math.abs(candidate - stateCost[nextState]) <= EPSILON
                        && (candidateBends < stateBends[nextState] || candidateBends == stateBends[nextState]
                        && candidateCrossings < stateCrossings[nextState])) {
                    stateCost[nextState] = candidate;
                    stateBends[nextState] = candidateBends;
                    stateCrossings[nextState] = candidateCrossings;
                    statePrevious[nextState] = state;
                    push(candidate, candidateBends, candidateCrossings, nextState);
                }
            }
        }
        if (reached < 0) { return null; }
        List<Point> result = new ArrayList<>();
        for (int state = reached; state >= 0; state = statePrevious[state]) {
            int point = state / 4;
            result.add(new Point(xs.get(point / height), ys.get(point % height)));
            if (point == startIndex) { break; }
        }
        Collections.reverse(result);
        simplify(result);
        return result;
    }

    private void ensureSearchCapacity(final int points) {
        if (stateStamp.length >= points * 4) { return; }
        int states = Math.max(points * 4, 2 * stateStamp.length);
        stateStamp = new int[states];
        stateCost = new double[states];
        stateBends = new int[states];
        stateCrossings = new int[states];
        statePrevious = new int[states];
        stateClosed = new boolean[states];
        edgeStamp = new int[states / 2];
        edgeOpen = new boolean[states / 2];
        edgeCrossings = new int[states / 2];
        edgeLengths = new double[states / 2];
    }

    /** Initializes a state the first time the current search reaches it. */
    private void touch(final int state, final int stamp) {
        if (stateStamp[state] != stamp) {
            stateStamp[state] = stamp;
            stateCost[state] = Double.POSITIVE_INFINITY;
            stateBends[state] = 0;
            stateCrossings[state] = 0;
            statePrevious[state] = -1;
            stateClosed[state] = false;
        }
    }

    private static boolean before(final double cost, final int bends, final int crossings, final int state,
            final double otherCost, final int otherBends, final int otherCrossings, final int otherState) {
        if (cost != otherCost) { return cost < otherCost; }
        if (bends != otherBends) { return bends < otherBends; }
        if (crossings != otherCrossings) { return crossings < otherCrossings; }
        return state < otherState;
    }

    private void push(final double cost, final int bends, final int crossings, final int state) {
        if (heapSize == heapState.length) {
            int capacity = Math.max(64, 2 * heapSize);
            heapCost = java.util.Arrays.copyOf(heapCost, capacity);
            heapBends = java.util.Arrays.copyOf(heapBends, capacity);
            heapCrossings = java.util.Arrays.copyOf(heapCrossings, capacity);
            heapState = java.util.Arrays.copyOf(heapState, capacity);
        }
        int index = heapSize++;
        while (index > 0) {
            int parent = (index - 1) / 2;
            if (!before(cost, bends, crossings, state,
                    heapCost[parent], heapBends[parent], heapCrossings[parent], heapState[parent])) { break; }
            heapCost[index] = heapCost[parent];
            heapBends[index] = heapBends[parent];
            heapCrossings[index] = heapCrossings[parent];
            heapState[index] = heapState[parent];
            index = parent;
        }
        heapCost[index] = cost;
        heapBends[index] = bends;
        heapCrossings[index] = crossings;
        heapState[index] = state;
    }

    /** Removes the minimum, which is read from index 0 before the call. */
    private void pop() {
        heapSize--;
        if (heapSize == 0) { return; }
        double cost = heapCost[heapSize];
        int bends = heapBends[heapSize];
        int crossings = heapCrossings[heapSize];
        int state = heapState[heapSize];
        int index = 0;
        while (true) {
            int child = 2 * index + 1;
            if (child >= heapSize) { break; }
            if (child + 1 < heapSize && before(heapCost[child + 1], heapBends[child + 1], heapCrossings[child + 1],
                    heapState[child + 1], heapCost[child], heapBends[child], heapCrossings[child], heapState[child])) {
                child++;
            }
            if (!before(heapCost[child], heapBends[child], heapCrossings[child], heapState[child],
                    cost, bends, crossings, state)) { break; }
            heapCost[index] = heapCost[child];
            heapBends[index] = heapBends[child];
            heapCrossings[index] = heapCrossings[child];
            heapState[index] = heapState[child];
            index = child;
        }
        heapCost[index] = cost;
        heapBends[index] = bends;
        heapCrossings[index] = crossings;
        heapState[index] = state;
    }

    /**
     * Separates collinear interior segments of different routes that overlap, so that parallel
     * connectors run side by side one edge spacing apart. Pre-routed connectors are never moved,
     * and a nudge that would enter an obstacle or fold a neighbor segment is dropped.
     */
    private void nudge(final List<Route> routes) {
        double gap = Math.max(4, graph.graph.getProperty(CoreOptions.SPACING_EDGE_EDGE));
        for (int axis = 0; axis < 2; axis++) {
            boolean horizontalSegments = axis == 0;
            List<Object[]> entries = new ArrayList<>();
            for (Route route : routes) {
                if (route.fixed) { continue; }
                List<Point> points = route.points;
                for (int s = 2; s <= points.size() - 2; s++) {
                    Point a = points.get(s - 1);
                    Point b = points.get(s);
                    boolean horizontal = Math.abs(a.y - b.y) < EPSILON;
                    if (horizontal != horizontalSegments || distance(a, b) < EPSILON) { continue; }
                    double coordinate = horizontal ? a.y : a.x;
                    double low = horizontal ? Math.min(a.x, b.x) : Math.min(a.y, b.y);
                    double high = horizontal ? Math.max(a.x, b.x) : Math.max(a.y, b.y);
                    Point before = points.get(s - 2);
                    Point after = points.get(s + 1);
                    double key = horizontal ? (before.y + after.y) / 2 - coordinate : (before.x + after.x) / 2 - coordinate;
                    entries.add(new Object[] {route, s, coordinate, low, high, key});
                }
            }
            Collections.sort(entries, (p, q) -> {
                int byCoordinate = Double.compare((Double) p[2], (Double) q[2]);
                if (byCoordinate != 0) { return byCoordinate; }
                int byLow = Double.compare((Double) p[3], (Double) q[3]);
                return byLow != 0 ? byLow : Integer.compare(((Route) p[0]).index, ((Route) q[0]).index);
            });
            int i = 0;
            while (i < entries.size()) {
                List<Object[]> cluster = new ArrayList<>();
                cluster.add(entries.get(i));
                double coordinate = (Double) entries.get(i)[2];
                double reach = (Double) entries.get(i)[4];
                int j = i + 1;
                while (j < entries.size() && Math.abs((Double) entries.get(j)[2] - coordinate) < EPSILON
                        && (Double) entries.get(j)[3] < reach - EPSILON) {
                    cluster.add(entries.get(j));
                    reach = Math.max(reach, (Double) entries.get(j)[4]);
                    j++;
                }
                i = j;
                if (cluster.size() < 2) { continue; }
                Collections.sort(cluster, (p, q) -> {
                    int byKey = Double.compare((Double) p[5], (Double) q[5]);
                    return byKey != 0 ? byKey : Integer.compare(((Route) p[0]).index, ((Route) q[0]).index);
                });
                for (int k = 0; k < cluster.size(); k++) {
                    double offset = (k - (cluster.size() - 1) / 2.0) * gap;
                    shiftSegment((Route) cluster.get(k)[0], (Integer) cluster.get(k)[1], horizontalSegments, offset);
                }
            }
        }
    }

    /** Moves segment s of a route perpendicular by offset if the result stays valid. */
    private void shiftSegment(final Route route, final int s, final boolean horizontal, final double offset) {
        if (Math.abs(offset) < EPSILON) { return; }
        List<Point> points = route.points;
        Point a = points.get(s - 1);
        Point b = points.get(s);
        Point before = points.get(s - 2);
        Point after = points.get(s + 1);
        Point na = horizontal ? new Point(a.x, a.y + offset) : new Point(a.x + offset, a.y);
        Point nb = horizontal ? new Point(b.x, b.y + offset) : new Point(b.x + offset, b.y);
        // Neighbors keep their direction: the moved segment must not pass its neighbors' far ends.
        double beforeOld = horizontal ? a.y - before.y : a.x - before.x;
        double beforeNew = horizontal ? na.y - before.y : na.x - before.x;
        double afterOld = horizontal ? after.y - b.y : after.x - b.x;
        double afterNew = horizontal ? after.y - nb.y : after.x - nb.x;
        if (beforeOld * beforeNew <= EPSILON || afterOld * afterNew <= EPSILON) { return; }
        // Neighbor segments may be the stubs inside the endpoints' own clearance bands.
        Set<ElkNode> own = new HashSet<>(route.excluded);
        own.add(GeometryGraph.endpointNode(route.source));
        own.add(GeometryGraph.endpointNode(route.target));
        if (!visible(na, nb, route.excluded) || !visible(before, na, own) || !visible(nb, after, own)) { return; }
        points.set(s - 1, na);
        points.set(s, nb);
        route.moved = true;
    }

    private int cell(final double coordinate) { return (int) Math.floor(coordinate / cellSize); }
    /** Packs a cell coordinate pair into one key; a collision beyond the packed range only widens a bucket. */
    private static Integer cellKey(final int x, final int y) { return Integer.valueOf(x * 65536 + y); }

    @SuppressWarnings("unchecked")
    private static <T> List<T>[] newBuckets(final int count) { return (List<T>[]) new List[count]; }

    private List<Rect> rectBucket(final int x, final int y, final boolean create) {
        int gx = x - gridMinX;
        int gy = y - gridMinY;
        if (gx >= 0 && gy >= 0 && gx < gridWidth && gy < gridHeight) {
            int index = gx * gridHeight + gy;
            List<Rect> bucket = rectGrid[index];
            if (bucket == null && create) {
                bucket = new ArrayList<>();
                rectGrid[index] = bucket;
            }
            return bucket;
        }
        Integer key = cellKey(x, y);
        List<Rect> bucket = cells.get(key);
        if (bucket == null && create) {
            bucket = new ArrayList<>();
            cells.put(key, bucket);
        }
        return bucket;
    }

    private List<Segment> segmentBucket(final int x, final int y, final boolean create) {
        int gx = x - gridMinX;
        int gy = y - gridMinY;
        if (gx >= 0 && gy >= 0 && gx < gridWidth && gy < gridHeight) {
            int index = gx * gridHeight + gy;
            List<Segment> bucket = segmentGrid[index];
            if (bucket == null && create) {
                bucket = new ArrayList<>();
                segmentGrid[index] = bucket;
            }
            return bucket;
        }
        Integer key = cellKey(x, y);
        List<Segment> bucket = segmentCells.get(key);
        if (bucket == null && create) {
            bucket = new ArrayList<>();
            segmentCells.put(key, bucket);
        }
        return bucket;
    }
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
    /** An unordered pair of points, the key of the visibility cache. */
    private static final class SegmentKey {
        final double ax;
        final double ay;
        final double bx;
        final double by;
        SegmentKey(final Point a, final Point b) {
            boolean ordered = a.x < b.x || a.x == b.x && a.y <= b.y;
            ax = ordered ? a.x : b.x;
            ay = ordered ? a.y : b.y;
            bx = ordered ? b.x : a.x;
            by = ordered ? b.y : a.y;
        }
        @Override
        public int hashCode() {
            int hash = (int) (ax * 8);
            hash = hash * 31 + (int) (ay * 8);
            hash = hash * 31 + (int) (bx * 8);
            return hash * 31 + (int) (by * 8);
        }
        @Override
        public boolean equals(final Object other) {
            if (!(other instanceof SegmentKey)) { return false; }
            SegmentKey key = (SegmentKey) other;
            return ax == key.ax && ay == key.ay && bx == key.bx && by == key.by;
        }
    }
    private static final class Segment {
        final Point a;
        final Point b;
        final ElkEdge owner;
        /** Stamp of the last query that tested this segment, so bucket walks skip duplicates. */
        int mark;
        Segment(final Point a, final Point b) { this(a, b, null); }
        Segment(final Point a, final Point b, final ElkEdge owner) { this.a = a; this.b = b; this.owner = owner; }
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
        int index;
        boolean fixed;
        boolean moved;
        List<Rect> reserved;
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
        /** Stamp of the last query that tested this rectangle, so bucket walks skip duplicates. */
        int mark;
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
