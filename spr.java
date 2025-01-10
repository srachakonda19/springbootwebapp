import java.util.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class Graph<T extends Comparable<T>> {
    private final Map<T, List<Edge<T>>> adjacencyList;
    private final ReentrantReadWriteLock lock;

    public Graph() {
        this.adjacencyList = new HashMap<>();
        this.lock = new ReentrantReadWriteLock();
    }

    public void addVertex(T vertex) {
        lock.writeLock().lock();
        try {
            adjacencyList.putIfAbsent(vertex, new ArrayList<>());
        } finally {
            lock.writeLock().unlock();
        }
    }

    public void addEdge(T from, T to, double weight, boolean isDirected) {
        lock.writeLock().lock();
        try {
            adjacencyList.get(from).add(new Edge<>(to, weight));
            if (!isDirected) {
                adjacencyList.get(to).add(new Edge<>(from, weight));
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    public Map<T, Double> dijkstraShortestPath(T source) {
        lock.readLock().lock();
        try {
            Map<T, Double> distances = new HashMap<>();
            PriorityQueue<Edge<T>> priorityQueue = new PriorityQueue<>(Comparator.comparingDouble(e -> e.weight));
            Set<T> visited = new HashSet<>();

            distances.put(source, 0.0);
            priorityQueue.offer(new Edge<>(source, 0.0));

            while (!priorityQueue.isEmpty()) {
                Edge<T> current = priorityQueue.poll();
                if (!visited.add(current.target)) continue;

                for (Edge<T> edge : adjacencyList.getOrDefault(current.target, Collections.emptyList())) {
                    if (visited.contains(edge.target)) continue;

                    double newDist = distances.get(current.target) + edge.weight;
                    if (newDist < distances.getOrDefault(edge.target, Double.MAX_VALUE)) {
                        distances.put(edge.target, newDist);
                        priorityQueue.offer(new Edge<>(edge.target, newDist));
                    }
                }
            }
            return distances;
        } finally {
            lock.readLock().unlock();
        }
    }

    private static class Edge<T> {
        T target;
        double weight;

        public Edge(T target, double weight) {
            this.target = target;
            this.weight = weight;
        }
    }

    public static void main(String[] args) {
        Graph<String> graph = new Graph<>();
        graph.addVertex("A");
        graph.addVertex("B");
        graph.addVertex("C");
        graph.addVertex("D");

        graph.addEdge("A", "B", 1, true);
        graph.addEdge("A", "C", 4, true);
        graph.addEdge("B", "C", 2, true);
        graph.addEdge("C", "D", 1, true);

        System.out.println("Shortest Paths from A: " + graph.dijkstraShortestPath("A"));
    }
}
