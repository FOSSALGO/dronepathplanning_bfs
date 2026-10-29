package drone.dronepathplanning_bfs;

import java.util.*;

/** Breadth-First Search (BFS) baseline; single drone, static occupancy grid. */
public final class DronePathPlanning {
    private final Point start, finish;
    private final boolean success;
    private final double pathCost;
    private final int expandedNodes;
    private final long elapsedNanos;
    private final int steps;
    private final String terminationReason;
    private static final int[][] N8 = {{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1}};

    public DronePathPlanning(int[][] environment, Drone drone, Point finish) {
        if (environment == null || environment.length == 0 || environment[0].length == 0 || drone == null || finish == null)
            throw new IllegalArgumentException("Grid, drone, and finish must be provided");
        this.start = new Point(drone.initialRowPosition, drone.initialColPosition);
        this.finish = finish;
        int rows = environment.length, cols = environment[0].length;
        if (!free(environment, start.row(), start.col()) || !free(environment, finish.row(), finish.col()))
            throw new IllegalArgumentException("Start or finish is outside grid or blocked");
        long begin = System.nanoTime();

        // =====================================================================
        // BEGIN REPLACEABLE ALGORITHM: BREADTH-FIRST SEARCH (BFS)
        // This is the ONLY algorithm section intended for student modification.
        // 8-connected movement: orthogonal cost = 1; diagonal cost = sqrt(2).
        // BFS minimizes the NUMBER OF STEPS, not necessarily weighted path cost.
        // Diagonal corner cutting is forbidden: both side cells must be free.
        // Neighbor traversal order is fixed (N, NE, E, SE, S, SW, W, NW).
        // =====================================================================
        boolean[][] seen = new boolean[rows][cols];
        Point[][] parent = new Point[rows][cols];
        ArrayDeque<Point> frontier = new ArrayDeque<>(); // FIFO queue
        frontier.addLast(start);
        seen[start.row()][start.col()] = true;

        int expanded = 0;
        boolean reached = false;

        while (!frontier.isEmpty()) {
            Point current = frontier.removeFirst();
            expanded++;

            if (current.equals(finish)) {
                reached = true;
                break;
            }

            for (int[] delta : N8) {
                int nr = current.row() + delta[0];
                int nc = current.col() + delta[1];
                if (!free(environment, nr, nc) || seen[nr][nc]) continue;

                // Prevent a diagonal move through the corner of any wall.
                if (delta[0] != 0 && delta[1] != 0 &&
                        (!free(environment, current.row(), nc) ||
                         !free(environment, nr, current.col()))) continue;

                seen[nr][nc] = true; // mark when enqueued, so each cell is queued once
                parent[nr][nc] = current;
                frontier.addLast(new Point(nr, nc));
            }
        }

        ArrayList<Point> resultPath = new ArrayList<>();
        double cost = Double.NaN;
        if (reached) {
            LinkedList<Point> chain = new LinkedList<>();
            Point current = finish;
            while (current != null) {
                chain.addFirst(current);
                if (current.equals(start)) break;
                current = parent[current.row()][current.col()];
            }
            resultPath.addAll(chain);

            // Metric only: geometrical travel cost of the BFS result path.
            cost = 0.0;
            for (int i = 1; i < resultPath.size(); i++) {
                Point p = resultPath.get(i - 1), q = resultPath.get(i);
                cost += (p.row() != q.row() && p.col() != q.col())
                        ? Math.sqrt(2.0) : 1.0;
            }
        }
        // =====================================================================
        // END REPLACEABLE ALGORITHM: BREADTH-FIRST SEARCH (BFS)
        // =====================================================================

        elapsedNanos = System.nanoTime() - begin;
        success = reached;
        expandedNodes = expanded;
        pathCost = cost;
        terminationReason = reached ? "GOAL_REACHED" : "NO_PATH";
        drone.path = reached ? resultPath : new ArrayList<>(List.of(start));
        drone.frameIndex = 0;
        steps = drone.path.size() - 1;
    }
    private static boolean free(int[][] env, int r, int c) {
        return r >= 0 && r < env.length && c >= 0 && c < env[0].length && env[r][c] != -1;
    }
    public Point getStart() { return start; }
    public Point getFinish() { return finish; }
    public boolean isSuccess() { return success; }
    public double getPathCost() { return pathCost; }
    public int getExpandedNodes() { return expandedNodes; }
    public long getElapsedNanos() { return elapsedNanos; }
    public int getSteps() { return steps; }
    public String getTerminationReason() { return terminationReason; }
}
