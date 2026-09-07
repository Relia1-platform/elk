/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.geometric;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.eclipse.elk.alg.common.GeometryGraph;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;

/** Linear, iterative Tarjan decomposition of the simple undirected structural graph. */
public final class TopologyAnalysis {
    public final List<List<Vertex>> blocks = new ArrayList<>();
    public final Set<Vertex> articulations = new HashSet<>();
    public final List<Vertex[]> bridges = new ArrayList<>();

    public TopologyAnalysis(final GeometryGraph graph) {
        int size = graph.vertices.size();
        int[] discovery = new int[size];
        int[] low = new int[size];
        int[] parent = new int[size];
        int[] next = new int[size];
        int[] children = new int[size];
        java.util.Arrays.fill(parent, -1);
        int time = 0;
        ArrayDeque<Vertex> stack = new ArrayDeque<>();
        ArrayDeque<Vertex[]> edges = new ArrayDeque<>();
        for (Vertex root : graph.vertices) {
            if (discovery[root.index] != 0) { continue; }
            discovery[root.index] = low[root.index] = ++time;
            stack.push(root);
            while (!stack.isEmpty()) {
                Vertex vertex = stack.peek();
                int v = vertex.index;
                if (next[v] < vertex.neighbors.size()) {
                    Vertex neighbor = vertex.neighbors.get(next[v]++);
                    int w = neighbor.index;
                    if (discovery[w] == 0) {
                        children[v]++;
                        parent[w] = v;
                        edges.push(new Vertex[] {vertex, neighbor});
                        discovery[w] = low[w] = ++time;
                        stack.push(neighbor);
                    } else if (w != parent[v] && discovery[w] < discovery[v]) {
                        edges.push(new Vertex[] {vertex, neighbor});
                        low[v] = Math.min(low[v], discovery[w]);
                    }
                } else {
                    stack.pop();
                    if (parent[v] >= 0) {
                        int p = parent[v];
                        Vertex parentVertex = graph.vertices.get(p);
                        low[p] = Math.min(low[p], low[v]);
                        if (low[v] >= discovery[p]) {
                            if (parent[p] >= 0 || children[p] > 1) { articulations.add(parentVertex); }
                            Set<Vertex> members = new HashSet<>();
                            Vertex[] edge;
                            do {
                                edge = edges.pop();
                                members.add(edge[0]);
                                members.add(edge[1]);
                            } while (edge[0] != parentVertex || edge[1] != vertex);
                            List<Vertex> block = new ArrayList<>(members);
                            Collections.sort(block, Comparator.comparingInt(n -> n.index));
                            blocks.add(block);
                        }
                        if (low[v] > discovery[p]) { bridges.add(new Vertex[] {parentVertex, vertex}); }
                    } else if (vertex.neighbors.isEmpty()) {
                        blocks.add(Collections.singletonList(vertex));
                    }
                }
            }
        }
        Collections.sort(blocks, (a, b) -> {
            int sizeOrder = Integer.compare(b.size(), a.size());
            return sizeOrder != 0 ? sizeOrder : Integer.compare(a.get(0).index, b.get(0).index);
        });
    }

    public static int edgeCount(final List<Vertex> vertices) {
        Set<Vertex> allowed = new HashSet<>(vertices);
        int count = 0;
        for (Vertex vertex : vertices) {
            for (Vertex neighbor : vertex.neighbors) { if (allowed.contains(neighbor)) { count++; } }
        }
        return count / 2;
    }

    /** A deterministic fundamental-cycle basis, bounded by the number of non-tree edges. */
    public static List<Vertex> longestBasisCycle(final List<Vertex> vertices) {
        if (vertices.size() < 3) { return Collections.emptyList(); }
        Set<Vertex> allowed = new HashSet<>(vertices);
        java.util.Map<Vertex, Vertex> parent = new java.util.HashMap<>();
        java.util.Map<Vertex, Integer> depth = new java.util.HashMap<>();
        java.util.Map<Vertex, Integer> next = new java.util.HashMap<>();
        ArrayDeque<Vertex> stack = new ArrayDeque<>();
        List<Vertex> best = new ArrayList<>();
        for (Vertex root : vertices) {
            if (depth.containsKey(root)) { continue; }
            stack.push(root);
            depth.put(root, 0);
            while (!stack.isEmpty()) {
                Vertex vertex = stack.peek();
                int index = next.getOrDefault(vertex, 0);
                if (index == vertex.neighbors.size()) { stack.pop(); continue; }
                Vertex neighbor = vertex.neighbors.get(index);
                next.put(vertex, index + 1);
                if (!allowed.contains(neighbor) || neighbor == parent.get(vertex)) { continue; }
                if (!depth.containsKey(neighbor)) {
                    parent.put(neighbor, vertex);
                    depth.put(neighbor, depth.get(vertex) + 1);
                    stack.push(neighbor);
                } else if (depth.get(neighbor) < depth.get(vertex)) {
                    List<Vertex> cycle = new ArrayList<>();
                    Vertex cursor = vertex;
                    while (cursor != null && cursor != neighbor) { cycle.add(cursor); cursor = parent.get(cursor); }
                    if (cursor == neighbor) {
                        cycle.add(neighbor);
                        cycle = canonical(cycle);
                        if (cycle.size() > best.size() || cycle.size() == best.size() && compare(cycle, best) < 0) {
                            best = cycle;
                        }
                    }
                }
            }
        }
        return best;
    }

    public static boolean qualifiesAsRing(final List<Vertex> block, final List<Vertex> cycle) {
        int chords = edgeCount(block) - block.size();
        return block.size() >= 3 && cycle.size() == block.size()
                && chords >= 0 && chords <= Math.max(1, block.size() / 4);
    }

    static List<Vertex> canonical(final List<Vertex> cycle) {
        int first = 0;
        for (int i = 1; i < cycle.size(); i++) {
            if (cycle.get(i).index < cycle.get(first).index) { first = i; }
        }
        List<Vertex> result = new ArrayList<>();
        int step = cycle.get((first + 1) % cycle.size()).index
                < cycle.get((first + cycle.size() - 1) % cycle.size()).index ? 1 : -1;
        for (int i = 0; i < cycle.size(); i++) {
            result.add(cycle.get((first + step * i + cycle.size()) % cycle.size()));
        }
        return result;
    }

    private static int compare(final List<Vertex> a, final List<Vertex> b) {
        for (int i = 0; i < a.size(); i++) {
            if (a.get(i).index != b.get(i).index) { return Integer.compare(a.get(i).index, b.get(i).index); }
        }
        return 0;
    }
}
