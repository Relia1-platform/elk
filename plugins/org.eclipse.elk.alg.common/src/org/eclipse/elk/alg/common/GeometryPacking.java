/* SPDX-License-Identifier: EPL-2.0 OR GPL-3.0-or-later */
package org.eclipse.elk.alg.common;

import java.util.ArrayList;
import java.util.List;
import org.eclipse.elk.alg.common.GeometryGraph.Vertex;

/** Shelf packing translates whole components without changing their internal geometry. */
public final class GeometryPacking {
    private GeometryPacking() { }

    public static void pack(final List<List<Vertex>> components, final double spacing, final Double aspectRatio) {
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
