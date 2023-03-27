package org.firstinspires.ftc.teamcode.pathfinding;

import org.firstinspires.ftc.teamcode.position_handler.pose;

import java.util.List;

public class aStarPathfinder {
    int field_height = 144; // inches for now
    int field_width = 144;
    int cols = 12; //override with grid manager
    int rows = 12;

    private node[][] grid = new node[cols-1][rows-1];

    private List<node> openSet;
    private List<node> closedSet;


    public List<node> path;

    private void initialize_grid(){ //
        for (int i = 0; i < cols-1; ++i){
            for (int j = 0; j < rows-1; ++j){
                grid[i][j].i = i;
                grid[i][j].j = j;
            }
        }
        for (int i = 0; i < cols-1; ++i){
            for (int j = 0; j < rows-1; ++j){
                grid[i][j].addNeighbors();
            }
        }
    }
    private class node{
        int i;
        int j;

        double f;
        double g;
        double h;

        node previous;

        node[] neighbors = new node[7];

        public void addNeighbors(){
            int x = this.i;
            int y = this.j; // renamed for sanity

            if (x < cols-1) {
                this.neighbors[0] = grid[x+1][y];
            }
            if(x < cols-1 && y < rows-1){
                this.neighbors[1] = grid[x+1][y+1];
            }
            if(y < rows-1){
                this.neighbors[2] = grid[x][y+1];
            }
            if(x > 0 && y < rows-1){
                this.neighbors[3] = grid[x-1][y+1];
            }
            if(x > 0){
                this.neighbors[4] = grid[x-1][y];
            }
            if(x > 0 && y > 0){
                this.neighbors[5] = grid[x-1][y-1];
            }
            if(y > 0){
                this.neighbors[6] = grid[x][y-1];
            }
            if(x < cols-1 && y > 0){
                this.neighbors[7] = grid[x+1][y-1];
            }

        }

        public double distanceTo(node target) {
            this.h = Math.hypot(target.i - this.i, target.j - this.j);
            return this.h;
        }

    }

    public void rawGridAStar(int x, int y, int i, int j){ //TODO do binary search for searches for optimization
        initialize_grid();

        node start = grid[x][y];
        node end = grid[i][j];

        openSet.add(start);

        while (!openSet.isEmpty()){
            int winner = 0; // assume winner is the lowest number in the index to start.
            for (i = 0; i < openSet.size(); ++i){
                node eval = openSet.get(i);
                node winning_node = openSet.get(winner);
                if (eval.f < winning_node.f){
                    winner = i;
                }
            }
            node current = openSet.get(winner);

            openSet.remove(current);
            closedSet.add(current);


            for (i = 0; i < current.neighbors.length; ++i){
                node neighbor = current.neighbors[i];
                if (!closedSet.contains(neighbor)){
                    double tempG = current.g + neighbor.distanceTo(current);
                    if (openSet.contains(neighbor)){
                        if (tempG < neighbor.g){
                            neighbor.g = tempG;
                            neighbor.previous = current;
                            neighbor.h = neighbor.distanceTo(end);
                            neighbor.f = neighbor.g + neighbor.h;
                        }
                    } else {
                        neighbor.g = tempG;
                        openSet.add(neighbor);
                        neighbor.previous = current;
                        neighbor.h = neighbor.distanceTo(end);
                        neighbor.f = neighbor.g + neighbor.h;
                    }



                }
            }

            if (current == end){

                node temp = current;

                path.add(temp);
                while (temp.previous != null){
                    path.add(temp.previous);
                    temp = temp.previous;
                }

              //TODO create the finished task
            }

        }



    }

    public void translate_coords_to_poses(pose pos, int x, int y){
        //TODO make this function translate a given coord to a pose.
    }

}
