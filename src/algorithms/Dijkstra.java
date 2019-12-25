/*
 * Several implementation of Dijkstra's algorithm.
 */

package algorithms;

import datastructures.Heap;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

/**
 * Several implementations of Dijkstra's algorithm.
 */
public class Dijkstra {

    /**
     * Dijkstra's algorithm, implemented with a binary minimum heap.
     * @param g the graph, in adjacency matrix form
     * @param s the source vertex
     * @return an integer array where int[x] is the vertex x's distance from s
     */
    public static int[] binaryMinHeapDijkstra(int[][] g, int s) {
        LinkedList vertices = new LinkedList<Vertex>();
        HashSet explored = new HashSet<Integer>();
        Heap h = new Heap<Vertex>(Vertex.class);
        int[] dist = new int[g.length];

        for (int i = 0; i < g.length; i++) {
            Vertex v = new Vertex(i);
            vertices.add(v);
            v.setDijkstraScore(i != s ? Integer.MAX_VALUE : 0);
            h.insert(v);
        }

        while (h.getElementCount() != 0) {
            Vertex min = (Vertex) h.extractMin();
            explored.add(min.getVertexNum());
            dist[min.getVertexNum()] = min.getDijkstraScore();

            for (int n = 0; n < g[min.getVertexNum()].length; n++) {
                // not a real neighbor or not an edge crossing the frontier
                if (g[min.getVertexNum()][n] == -1 || explored.contains(n)) continue;

                Vertex neighbor = (Vertex) vertices.get(n);
                h.delete(neighbor);
                int dScore = dist[min.getVertexNum()] + g[min.getVertexNum()][n];
                neighbor.setDijkstraScore(Math.min(dScore, neighbor.getDijkstraScore()));
                h.insert(neighbor);
            }
        }

        return dist;
    }

    /**
     * Dijkstra's algorithm, computes the single source shorted path distance
     * for a directed graph with non-negative edge lengths.
     * @complexity O(n^2 + m)
     * @param s the souce vertex
     * @param g the graph, in adjacency matrix form
     * @return an integer array where int[x] is the vertex x's distance from s
     */
    public static int[] trivialDijkstra(int[][] g, int s) {
        int[] dist = new int[g.length];
        LinkedList queue = new LinkedList<Integer>();
        for (int i = 0; i < g.length; i++) {
            queue.add(i);
            dist[i] = Integer.MAX_VALUE;
        }

        dist[s] = 0;

        while (!queue.isEmpty()) {

            int least = getMinVertex(queue, dist);
            queue.remove(Integer.valueOf(least));

            for (int n = 0; n < g[least].length; n++) {
                int d = dist[least] + g[least][n];
                if (d < dist[n] && g[least][n] != -1) dist[n] = d;
            }
        }

        return dist;
    }

    /**
     * A helper method for trivialDijkstra, naively iterates through every vertex in the
     * queue and returns the vertex with the lowest Dijkstra score
     * @param queue the queue of vertices that have not been processed
     * @param dist the array in which dist[i] is the shortest path from s to vertex i
     * @return the vertex with the lowest Dijkstra score
     */
    private static int getMinVertex(LinkedList<Integer> queue, int[] dist) {
        int least = queue.getFirst();
        for (int v : queue) {
            if (dist[v] < dist[least]) least = v;
        }
        return least;
    }

    /**
     * An alternative implementation of the straighforward Dijkstra's algorithm.
     * @omplexity O(mn)
     * @param g a 2D integer array corresponding to the matrix representation of the graph
     * @param s the source vertex
     * @return an integer array where int[x] is the vertex x's distance from s
     */
    public static int[] trivialDijkstraTwo(int[][] g, int s) {
        int[] dist = new int[g.length];
        LinkedList queue = new LinkedList<Integer>();
        for (int i = 0; i < g.length; i++) {
            dist[i] = Integer.MAX_VALUE;
        }

        dist[s] = 0;
        int least = s; // source should be processed first

        do {
            queue.add(least);

            for (int n = 0; n < g[least].length; n++) {
                int d = dist[least] + g[least][n];
                if (d < dist[n] && g[least][n] != -1) dist[n] = d;
            }

            least = getMinVertexTwo(g, dist, queue);
        } while (queue.size() < g.length);

        return dist;
    }

    /**
     * Iterates through all edges of the graph and finds the vertex v such that
     * v is an endpoint of an edge where w is in X and v is not, and len(w, v)
     * has the lowest dijkstra score (least distance to the source). X is the queue
     * of processed vertices.
     * @complexity O(m)
     * @return the vertex with the lowest dijkstra score
     */
    private static int getMinVertexTwo(int[][] g, int[] dist, List<Integer> q) {
        int least = 0;
        int currMin = Integer.MAX_VALUE;
        for (int i = 0; i < g.length; i++) {
            if (!q.contains(i)) continue;
            for (int j = 0; j < g[i].length; j++) {
                if (q.contains(j)) continue; // already processed
                if (dist[i] + g[i][j] < currMin && g[i][j] != -1) least = j;
            }
        }

        return least;
    }

    /**
     * A Comparable class to allow Vertices to be placed into a heap
     * for binaryMinDijkstra
     */
    private static class Vertex implements Comparable<Vertex> {

        /** This vertex's index. */
        private final int vertexNum;
        /** This vertex's dijkstra score (it's currently known minimum distance) */
        private int dijkstraScore;

        public Vertex(int v) {
            vertexNum = v;
        }

        public int getVertexNum() { return vertexNum; }
        public int getDijkstraScore() { return dijkstraScore; }

        public void setDijkstraScore(int d) { dijkstraScore = d; }

        @Override
        public int compareTo(Vertex vertex) {
            return dijkstraScore - vertex.dijkstraScore;
        }
    }
}
