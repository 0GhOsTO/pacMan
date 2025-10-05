package src.pas.pacman.agents;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.PriorityQueue;
// SYSTEM IMPORTS
import java.util.Random;
import java.util.Set;
import java.util.Stack;

// JAVA PROJECT IMPORTS
import edu.bu.pas.pacman.agents.Agent;
import edu.bu.pas.pacman.agents.SearchAgent;
import edu.bu.pas.pacman.interfaces.ThriftyPelletEater;
import edu.bu.pas.pacman.game.Action;
import edu.bu.pas.pacman.game.DefaultBoard.Cell;
import edu.bu.pas.pacman.game.DefaultBoard.CellState;
import edu.bu.pas.pacman.game.Game.GameView;
import edu.bu.pas.pacman.graph.Path;
import edu.bu.pas.pacman.graph.PelletGraph.PelletVertex;
import edu.bu.pas.pacman.utils.Coordinate;
import edu.bu.pas.pacman.utils.Pair;

public class PacmanAgent
        extends SearchAgent
        implements ThriftyPelletEater {

    private final Random random;
    private final HashMap<Coordinate, HashMap<Coordinate, Integer>> moveCost = new HashMap<>();

    public PacmanAgent(int myUnitId,
            int pacmanId,
            int ghostChaseRadius) {
        super(myUnitId, pacmanId, ghostChaseRadius);
        this.random = new Random();
    }

    public final Random getRandom() {
        return this.random;
    }

    // Think Vertex = One huge game including the location of Pacman and Pellets
    // getOutoingNeighbors returns every scenarios of eating the pellets and save
    // the distance from it.
    @Override
    public Set<PelletVertex> getOutoingNeighbors(final PelletVertex vertex,
            final GameView game) {
        // Run the BFS take all the outgoing vertecies
        // Grab the map of the location of all the pellets.
        // Get the outgoing neightbors
        // Gameview is read only structure. -> returns Set<Coordinate>
        // Do not stay in the current position.
        // Basically creating the islands of the Pellets
        //
        // THIS IS MAINLY BUILT FOR THE STOPPING THE BFS LOOP. ONCE IT'S EQUAL, IT STOP
        // ===========Save the location of the pellets===========
        // Set<Coordinate> pelletWorld = new HashSet<>();
        // for (int i = 0; i < game.getXBoardDimension(); i++) {
        // for (int j = 0; j < game.getYBoardDimension(); j++) {
        // if (game.getCell(new Coordinate(i, j)).getCellState() == CellState.PELLET) {
        // pelletWorld.add(new Coordinate(i, j));
        // }
        // }
        // }
        // // ===========Save the location of the pellets===========

        // Set<PelletVertex> res = new HashSet<>();
        // Set<Coordinate> seen = new HashSet<>();
        // Queue<Coordinate> q = new ArrayDeque<>();
        // // right, left, up, down
        // // Check if it's illegal move or not
        Coordinate src = vertex.getPacmanCoordinate();
        // int x = src.getXCoordinate();
        // int y = src.getYCoordinate();
        // // Initializing the start of the BFS
        // q.add(src);
        // // Run through the BFS first
        // Coordinate cur = src;
        // // Receive all the neighbors coming out from the source.
        // Set<Coordinate> neighbors;

        // while (!seen.equals(pelletWorld)) {

        // // One island creation
        // Set<Coordinate> island = new HashSet<>();
        // while (!q.isEmpty()) {
        // neighbors = getOutgoingNeighbors(q.remove(), game);
        // for (Coordinate temp : neighbors) {
        // q.add(temp);
        // seen.add(temp);
        // island.add(temp);
        // }
        // }
        // res.add(new PelletVertex(island, src));

        // }

        // return res;

        Set<PelletVertex> res = new HashSet<>();
        HashMap<Coordinate, Integer> cacheVal = moveCost.get(src);
        if (cacheVal == null) {
            cacheVal = new HashMap<>();
            moveCost.put(src, cacheVal);
        }
        // Take the rest over pellets
        for (Coordinate pellet : vertex.getRemainingPelletCoordinates()) {
            // If it does not exist...
            if (!cacheVal.containsKey(pellet)) {
                final Path<Coordinate> path = graphSearch(src, pellet, game);
                if (path == null) {
                    continue;
                }
                final int dist = Math.max(0, path.getNumVertices() - 1);
                moveCost.get(src).put(pellet, dist);
                // same in opposite.
                moveCost.get(pellet).put(src, dist);
            }

            PelletVertex next = vertex.removePellet(pellet);
            res.add(next);
        }

        return res;

    }

    @Override
    public float getEdgeWeight(final PelletVertex src,
            final PelletVertex dst) {
        // Find the average of the both of the coordinate.
        // float srcCount = 0;
        // float srcX = 0;
        // float srcY = 0;
        // for(Coordinate srcCor: src.getRemainingPelletCoordinates()){
        // srcCount++;
        // srcX += srcCor.getXCoordinate();
        // srcY += srcCor.getYCoordinate();
        // }
        // srcX = srcX/srcCount;
        // srcY = srcY/srcCount;

        // float dstCount = 0;
        // float dstX = 0;
        // float dstY = 0;
        // for(Coordinate dstCor: dst.getRemainingPelletCoordinates()){
        // dstCount++;
        // dstX += dstCor.getXCoordinate();
        // dstY += dstCor.getYCoordinate();
        // }
        // dstX = dstX/dstCount;
        // dstY = dstY/dstCount;
        // Using the manhatten distance to keep the units.
        // 1 = 1 tile

        // Basically the difference between which are eaten and which are not.

        // <RULE>
        // 1 Vertex = 1 difference
        Coordinate srcPac = src.getPacmanCoordinate();
        Coordinate taken = null;
        Set<Coordinate> dstSet = dst.getRemainingPelletCoordinates();
        for (Coordinate srcCoor : src.getRemainingPelletCoordinates()) {
            if (!dstSet.contains(srcCoor)) {
                taken = srcCoor;
            }
        }

        // Look up the cached board
        int movingCost = moveCost.get(srcPac).get(taken);

        return (float) movingCost;
    }

    // CURRENTLY NOT USING IT
    // Return the minimum distance between two tiles.
    public float getMinDistTile(final PelletVertex src, final PelletVertex tgt, final GameView game) {
        // Running the BFS with the multi-source.
        Coordinate cur = src.getPacmanCoordinate();
        PelletVertex srcCopy = src;
        Queue<Coordinate> q = new LinkedList<>();
        HashSet<Coordinate> seen = new HashSet<>();
        Set<Coordinate> tgtSet = tgt.getRemainingPelletCoordinates();
        float minDistBetweenVertex = Float.POSITIVE_INFINITY;
        // =======variable creation=======
        // =======Initilization step=======
        q.add(cur);
        seen.add(cur);
        // ================================
        for (Coordinate temp : srcCopy.getRemainingPelletCoordinates()) {
            q.add(temp);
            seen.add(temp);
        }
        while (!q.isEmpty()) {
            Coordinate temp = q.poll();
            Set<Coordinate> neighbors = getOutgoingNeighbors(temp, game);

            // Already seen
            for (Coordinate neighbor : neighbors) {
                // Run the BFS and found the closest one.
                if (tgtSet.contains(neighbor)) {
                    if (minDistBetweenVertex > graphSearch(cur, temp, game).getNumVertices())
                        minDistBetweenVertex = graphSearch(cur, temp, game).getNumVertices();
                }
                if (seen.contains(neighbor)) {
                    continue;
                } else {
                    q.add(neighbor);
                    seen.add(neighbor);
                }
            }

        }

        return minDistBetweenVertex;
    }

    // public int getPathLength(Path<Coordinate> p){
    // int length = 0;
    // Path<Coordinate> cur = p;

    // }

    @Override
    public float getHeuristic(final PelletVertex src,
            final GameView game) {
        // Some of the ideas for the Heuristics
        // 1. Closest = High number(MST)
        // 2. Building the shortest path.
        // 3. Grouping the nearby pellets into "clusters"

        // getEdgeWeight()
        // <RULE>
        // 1 Vertex = 1
        // 1 tile = 1

        // Best distance calculation over.
        // =======Example of heuristics in MANHATTEN DISTANCE =======
        // if (src.getRemainingPelletCoordinates().isEmpty()) {
        // return 0f;
        // }

        // Coordinate testing = src.getPacmanCoordinate();
        // int srcX = testing.getXCoordinate();
        // int srcY = testing.getYCoordinate();
        // int best = Integer.MAX_VALUE;

        // for (Coordinate coor : src.getRemainingPelletCoordinates()) {
        // int distance = Math.abs(coor.getXCoordinate() - srcX) +
        // Math.abs(coor.getYCoordinate() - srcY);
        // if (distance < best) {
        // best = distance;
        // }
        // }
        // // =======Example of heuristics in MANHATTEN DISTANCE =======

        // SMALLER THE NUMBER, WE ARE MOVING ON TO THE PLACE WHERE IT IS
        // LOCATED(Djaikstra)

        // ####### REAL STARTS HERE #######
        Set<Coordinate> pellets = src.getRemainingPelletCoordinates();
        if (pellets.isEmpty())
            return 0f;
        Coordinate pacMan = src.getPacmanCoordinate();

        // Receive all the possible movings out there that is cached.
        HashMap<Coordinate, Integer> fromSrc = moveCost.get(pacMan);
        if (fromSrc == null) {
            fromSrc = new HashMap<>();
            moveCost.put(pacMan, fromSrc);
        }

        float nearest = Float.MAX_VALUE;
        for (Coordinate location : pellets) {
            float dist = fromSrc.get(location);
            if (dist < nearest) {
                nearest = dist;
            }
        }

        // Just return the nearest saved pellet's value
        return nearest;
    }

    @Override
    public Path<PelletVertex> findPathToEatAllPelletsTheFastest(final GameView game) {
        PelletVertex start = new PelletVertex(game);

        //======AstarAlgs======
        PriorityQueue<Coordinate> openSet = new PriorityQueue<>();
        HashMap<Coordinate, Integer> gScore = new HashMap<>();
        openSet.add(src);
        gScore.put(src, 0);

        while (!openSet.isEmpty()) {
            Coordinate cur = openSet.poll();
            if (src.equals(tgt)) {
                return;
            }
            for (Coordinate neighbor : getOutgoingNeighbors(cur, game)) {
                int newG = gScore.get(cur) /* + Neightbor */;
                if( newG < gScore.){
                    gScore.put(neighbor, newG);
                    openSet.add(neighbor);
                }
            }
        }

        return null;
        //======AstarAlgs======
    }

    @Override
    public Set<Coordinate> getOutgoingNeighbors(final Coordinate src,
            final GameView game) {

        // Get the outgoing neightbors
        // Gameview is read only structure. -> returns Set<Coordinate>
        // Do not stay in the current position.

        Set<Coordinate> res = new HashSet<>();
        // right, left, up, down
        // Check if it's illegal move or not
        int x = src.getXCoordinate();
        int y = src.getYCoordinate();
        // Check if it's in the bound or not
        if (game.isInBounds(new Coordinate(x + 1, y)) && game.isLegalPacmanMove(src, Action.EAST)) {
            res.add(new Coordinate(x + 1, y));
        }
        if (game.isInBounds(new Coordinate(x, y - 1)) && game.isLegalPacmanMove(src, Action.NORTH)) {
            res.add(new Coordinate(x, y - 1));
        }
        if (game.isInBounds(new Coordinate(x - 1, y)) && game.isLegalPacmanMove(src, Action.WEST)) {
            res.add(new Coordinate(x - 1, y));
        }
        if (game.isInBounds(new Coordinate(x, y + 1)) && game.isLegalPacmanMove(src, Action.SOUTH)) {
            res.add(new Coordinate(x, y + 1));
        }

        return res;
    }

    @Override
    public Path<Coordinate> graphSearch(final Coordinate src,
            final Coordinate tgt,
            final GameView game) {
        // Use the path datatype.
        // Find the shortest path to the place.
        // Will use the BFS to find the shortest path to target.

        if (src.equals(tgt)) {
            return new Path<>(src);
        }

        // Will use the Queue
        // Put it to the Queue to the path itself.
        Queue<Path<Coordinate>> queue = new LinkedList<>();
        // Created the path to src.
        Path<Coordinate> path = new Path<>(src);
        Set<Coordinate> seen = new HashSet<>();

        // Add the current path to src. Initialization.
        queue.add(path);
        seen.add(src);

        while (!queue.isEmpty()) {
            Path<Coordinate> curPath = queue.poll();
            Coordinate cur = curPath.getDestination();
            for (Coordinate temp : getOutgoingNeighbors(cur, game)) {
                // Return the path once it hits the target.
                if (temp.equals(tgt)) {
                    return new Path<>(temp, 1.0f, curPath);
                }
                if (!seen.contains(temp)) {
                    // Add it to the seen
                    seen.add(temp);
                    queue.add(new Path<>(temp, 1.0f, curPath));
                }
            }

        }

        // unable to reach
        return null;

    }

    @Override
    public void makePlan(final GameView game) {
        Coordinate src = game.getEntity(game.getPacmanId()).getCurrentCoordinate();
        Coordinate tgt = this.getTargetCoordinate();

        // Receive the target path.
        // Path<Coordinate> tgtPath = graphSearch(src,tgt, game);

        // System.out.println("src: "+src);
        // //Scan the whole map
        // Set<Coordinate> pellets = new HashSet<>();

        // for (int x= 0; x<game.getXBoardDimension(); x++){
        // for (int y = 0; y<game.getYBoardDimension(); y++){
        // Coordinate coor = new Coordinate(x,y);
        // Cell tempCell = game.getCell(coor);
        // if(game.isInBounds(coor) && tempCell.getCellState() == CellState.PELLET){
        // pellets.add(coor);
        // }
        // }
        // }
        // Grabbed all the location of pellets
        // Creationg of the result Path with the new source coordinate
        Path<Coordinate> resPath = graphSearch(src, tgt, game);

        // BRAINDEAD BRUTEFORCING WITHOUT OPTIMIZATION
        // for(Coordinate tempCoor: pellets){
        // //graphSearch the fastest location
        // Path<Coordinate> temPath = graphSearch(src, tempCoor, game);
        // //move the src to the place
        // src = temPath.getDestination();
        // //Combine previous path and current path
        // resPath = combinePath(resPath, temPath);
        // }

        List<Coordinate> coords = new ArrayList<>();
        for (Path<Coordinate> cur = resPath; cur != null; cur = cur.getParentPath()) {
            coords.add(cur.getDestination());
        }
        Collections.reverse(coords);

        Stack<Coordinate> finPlan = new Stack<>();
        for (int i = coords.size() - 1; i >= 0; i--) {
            finPlan.push(coords.get(i));
        }

        // System.out.println("stack: " + finPlan);
        finPlan.pop();
        this.setPlanToGetToTarget(finPlan);

    }

    @Override
    public Action makeMove(final GameView game) {
        System.out.println("values: " + Action.values());
        System.out.println("1makeMove called");
        // pop form teh received Stack and move the Pacman.
        Stack<Coordinate> plan = this.getPlanToGetToTarget();
        System.out.println("2plan: " + plan);
        Coordinate src = game.getEntity(game.getPacmanId()).getCurrentCoordinate();

        // Edgecase:
        if (plan == null || plan.isEmpty()) {
            // it does not have the plan
            return null;
        }

        // pop the current coordinate.
        // Eliminate the first one so that it "only" focus on "moving"
        Coordinate cur = plan.pop();
        // update the method
        this.setPlanToGetToTarget(plan);
        System.out.println("3cur: " + cur);
        System.out.println("4after pop: " + this.getPlanToGetToTarget());

        int srcX = src.getXCoordinate();
        int srcY = src.getYCoordinate();
        int curX = cur.getXCoordinate();
        int curY = cur.getYCoordinate();

        System.out.println("src(X,Y): " + srcX + " " + srcY);
        System.out.println("cur(X,Y): " + curX + " " + curY);

        // When they are same, just return nothing
        if (srcX == curX && srcY == curY) {
            System.out.println("NULL");
            return null;
        }

        if (srcX - 1 == curX) {
            System.out.println("WEST");
            return Action.WEST;
        }

        if (srcX + 1 == curX) {
            System.out.println("EAST");
            return Action.EAST;
        }

        if (srcY - 1 == curY) {
            System.out.println("NORTH");
            return Action.NORTH;
        }

        if (srcY + 1 == curY) {
            System.out.println("SOUTH");
            return Action.SOUTH;
        }
        // Example
        // return Action.values()[this.getRandom().nextInt(Action.values().length)];

        // Do not know what to do.
        System.out.println("5DUNNO WHAT TO DO");
        return null;
    }

    @Override
    public void afterGameEnds(final GameView game) {

    }
}
