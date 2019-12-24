/*
 * A set of unit tests for the Dijkstra algorithms.
 */

package test;

import algorithms.Dijkstra;
import org.junit.Assert;
import org.junit.Test;

import java.util.Random;

/**
 * A set of tests covering all implementations of the Dijkstra algorithm.
 */
public class DijkstraTest {

    private int[][] g1 =
            {{-1,3,2},
            {2,-1,0},
            {-1,0,-1}};
    private int s1 = 0;
    private int[] d1 = {0, 2, 2};
    private int[][] g2 =
            {{-1,1,4},
            {1,-1,4},
            {4,4,-1}};
    private int s2 = 0;
    private int[] d2 = {0, 1, 4};
    private int[][] g3 =
            {{-1,5,2,15},
            {2,-1,0,3},
            {1,-1,-1,9},
            {0,0,0,-1}};
    private int s3 = 2;
    private int[] d3 = {1, 6, 0, 9};



    @Test
    public void testTrivialDijkstra() {
        int[] dist = Dijkstra.trivialDijkstra(g1, s1);
        Assert.assertArrayEquals(dist, d1);
        dist = Dijkstra.trivialDijkstra(g2, s2);
        Assert.assertArrayEquals(dist, d2);
        dist = Dijkstra.trivialDijkstra(g3, s3);
        Assert.assertArrayEquals(dist, d3);
    }

    @Test
    public void testTrivialDijkstraTwo() {
        int[] dist = Dijkstra.trivialDijkstraTwo(g1, s1);
        Assert.assertArrayEquals(dist, d1);
        dist = Dijkstra.trivialDijkstraTwo(g2, s2);
        Assert.assertArrayEquals(dist, d2);
        dist = Dijkstra.trivialDijkstraTwo(g3, s3);
        Assert.assertArrayEquals(dist, d3);
    }




}
