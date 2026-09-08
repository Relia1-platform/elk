/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;

/**
 * Shelf packing translates whole components without changing their internal geometry. The shelf
 * width follows the requested aspect ratio. In model order components fill a row and wrap; the
 * compact variant takes them by decreasing height, first fit into the shelves opened so far, so
 * that tops align, shelves are full, and the result does not depend on the input order.
 */
public final class GeometryPacking {
    private GeometryPacking() { }

    public static void pack(final List<List<Vertex>> components, final double spacing, final Double aspectRatio) {
        pack(components, spacing, aspectRatio, false);
    }

    public static void pack(final List<List<Vertex>> components, final double spacing, final Double aspectRatio,
            final boolean compact) {
        if (components.size() < 2) {
            return;
        }
        List<GeometryBounds> boxes = new ArrayList<>();
        double area = 0;
        double widest = 0;
        for (List<Vertex> component : components) {
            GeometryBounds box = bounds(component);
            boxes.add(box);
            double width = box.maxX - box.minX;
            double height = box.maxY - box.minY;
            area += (width + spacing) * (height + spacing);
            widest = Math.max(widest, width);
        }
        double rowWidth = Math.max(widest, Math.sqrt(area * Math.max(0.1, aspectRatio == null ? 1.6 : aspectRatio)));
        if (compact) {
            packCompact(components, boxes, spacing, rowWidth);
            return;
        }
        double x = 0;
        double y = 0;
        double rowHeight = 0;
        for (int i = 0; i < components.size(); i++) {
            GeometryBounds box = boxes.get(i);
            double width = box.maxX - box.minX;
            double height = box.maxY - box.minY;
            if (x > 0 && x + width > rowWidth) {
                x = 0;
                y += rowHeight + spacing;
                rowHeight = 0;
            }
            translate(components.get(i), x - box.minX, y - box.minY);
            x += width + spacing;
            rowHeight = Math.max(rowHeight, height);
        }
    }

    /** First fit decreasing height: tallest first, each into the first shelf with room, else a new shelf. */
    private static void packCompact(final List<List<Vertex>> components, final List<GeometryBounds> boxes,
            final double spacing, final double rowWidth) {
        List<Integer> order = new ArrayList<>();
        for (int i = 0; i < components.size(); i++) { order.add(i); }
        Collections.sort(order, (p, q) -> {
            GeometryBounds a = boxes.get(p);
            GeometryBounds b = boxes.get(q);
            int byHeight = Double.compare(b.maxY - b.minY, a.maxY - a.minY);
            if (byHeight != 0) { return byHeight; }
            int byWidth = Double.compare(b.maxX - b.minX, a.maxX - a.minX);
            return byWidth != 0 ? byWidth : firstIdentifier(components.get(p)).compareTo(firstIdentifier(components.get(q)));
        });
        List<double[]> shelves = new ArrayList<>();
        for (int i : order) {
            GeometryBounds box = boxes.get(i);
            double width = box.maxX - box.minX;
            double height = box.maxY - box.minY;
            double[] shelf = null;
            for (double[] candidate : shelves) {
                if (candidate[1] + width <= rowWidth + 1e-9) { shelf = candidate; break; }
            }
            if (shelf == null) {
                double y = shelves.isEmpty() ? 0 : shelves.get(shelves.size() - 1)[0]
                        + shelves.get(shelves.size() - 1)[2] + spacing;
                shelf = new double[] {y, 0, height};
                shelves.add(shelf);
            }
            translate(components.get(i), shelf[1] - box.minX, shelf[0] - box.minY);
            shelf[1] += width + spacing;
        }
    }

    /** The smallest node identifier of a component, the order-independent tie-break. */
    private static String firstIdentifier(final List<Vertex> component) {
        String best = null;
        for (Vertex vertex : component) {
            String id = GeometryGraph.identifier(vertex.node);
            if (best == null || id.compareTo(best) < 0) { best = id; }
        }
        return best == null ? "" : best;
    }

    public static GeometryBounds bounds(final List<Vertex> component) {
        GeometryBounds box = new GeometryBounds();
        for (Vertex vertex : component) {
            box.include(vertex.x + vertex.left, vertex.y + vertex.top);
            box.include(vertex.x + vertex.right, vertex.y + vertex.bottom);
        }
        if (component.isEmpty()) {
            box.include(0, 0);
        }
        return box;
    }

    public static void translate(final List<Vertex> component, final double x, final double y) {
        for (Vertex vertex : component) {
            vertex.x += x;
            vertex.y += y;
        }
    }
}
