package frc.robot.commands;

import java.util.ArrayList;
import frc.robot.utils.FieldImage;
import frc.robot.utils.PathFollowTargetPos;
import edu.wpi.first.wpilibj2.command.Command;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PathGenCmd extends Command {
  //creates a new field image object (array that stores the dimensions and obstables on the field, 1 is obsticle 0 is clear)
  FieldImage fieldImage = new FieldImage();
  //variable to store the current position of the robot
  int[] currentPos;
  //varibale to store the target position of the robot
  int[] targetPos;
  //the g value of the heuristic (distance already travelled to reach the current position)
  double g;
  //assigns the array from the field position to a member variable called field, essentailly just making a copy of it local to this file
  int[][] field = fieldImage.field;
  //three dimensional array to store connections of a specific coordinate, the first two dimensions are the x and y values of the coordinate being checked, and the last dimension stores the x value (index 0) and y value (index 1) of the neighbour
  int[][][] connections;
  //2d array to store the gvalues of coordinates
  double[][] gVals;
  //a string to store the path as a set of coordinates
  String pathString;
  //the length of the field in ft
  int fieldLength = 57;
  //ft over 'pixles' (length of array), used to derive field with to ensure consistancy
  int conversionFactor = fieldLength / field.length;

  //calculates the f value of the heuristic given a coordinate (x is stored at the index 0, y is stored at the index 1 of pos)
  public double calcF(int[] pos) {
    //f is calculated by adding the g and h values
    return gVals[pos[0]][pos[1]] + calcH(pos);
  }

  //calculates the h value of the heuristic given a coordinate (x is stored at the index 0, y is stored at the index 1 of pos)
  public double calcH(int[] pos) {
    //h is calculated by getting the euclidian distance between the current position and the goal position (imagine drawing a straight line between the two coords and getting its length)
    return Math.sqrt(Math.pow(pos[0] - targetPos[0], 2) + Math.pow(pos[1] - targetPos[1], 2));
  }

  //generates an arraylist of neighbouring coordinates surrounding the current position (all 8 coords around it)
  public ArrayList<int[]> getNeighbours(int[] pos, ArrayList<int[]> processed) {
    //creates the arraylist where the neighbours will be stored
    ArrayList<int[]> neighbours = new ArrayList<int[]>();
    //iterates over numbers from -1 to 1 inclusive (for x value)
    for(int i = -1; i < 2; i++) {
      //iterates over numbers from -1 to 1 inclusive (for y value)
      for(int j = -1; j < 2; j++) {
        //creates a temporary position variable to represent the neighbour being checked for validity (adds i and j values to x and y values, leading to 9 overall checks encompassing all the surrounding coordinates)
        int[] temp = {pos[0] + i, pos[1] + j};
        //Checks if the neighbour being evaluated is inside the field perimeter, is a 0 value (not an occupied 1 space), and is not the robot's current coordinate (i and j are both 0)
        if((pos[0] + i >= 0 && pos[0] + i <= field.length && pos[1] + j >= 0 && pos[1] + j <= field[0].length
            && field[pos[0] + i][pos[1] + j] == 0 && (i != 0 || j != 0))) {
          //sets fail to false, meaning that the niegbour has been successfully evaluated as a possible candidate for the next move 
          boolean fail = false;
          //iterates over the already processed coordinates
          for(int[] f : processed) {
            //If the coordinate has already been processed, it is not a possible move and fair is set to true
            if(((temp[0]) == (f[0]) && (temp[1]) == (f[1]))) {
              fail = true;
            }
          //If fail is false, add it to the list fo neighbour
          }
          if(fail == false) {
            neighbours.add(temp);
          }
        }
      }
    }
    //return the list of neighbours
    return neighbours;
  }

  //Gets the euclidian distance between two points
  public double getDistance(int[] start, int[] end) {
    return Math.sqrt(Math.pow(start[0] - end[0], 2) + Math.pow(start[1] - end[1], 2));
  }

  //Generates the path to be followed from the start to end coordinates
  public ArrayList<int[]> generatePath(int[] targetPos) {
    //creates an array to hold the starting position, calculated by taking the current robot position and multiplying it by the conversion factor
    int[] startingPos = {(int) Math.round(PathFollowTargetPos.startingPos[0] * conversionFactor),
        (int) Math.round(PathFollowTargetPos.startingPos[0] * conversionFactor)};
    //creates a variable for the target positon of the robot
    this.targetPos = targetPos;
    //Creates a variable to store the robot's connections
    this.connections = new int[field.length][field[0].length][2];
    //Sets the g vals to a blank array of the correct length
    gVals = new double[field.length][field[0].length];

    //creates a new arraylist to store coordinates which have yet to be searched
    ArrayList<int[]> toSearch = new ArrayList<int[]>();
    //creates a new arraylist to store coordinates which have already been searched
    ArrayList<int[]> processed = new ArrayList<int[]>();
    toSearch.add(startingPos);

    while(!toSearch.isEmpty()) {
      currentPos = toSearch.get(0);

      for(int[] coord : toSearch) {
        if(calcF(coord) < calcF(currentPos)
            || Math.round((calcF(coord) * 100000)) == Math.round(calcF(currentPos) * 100000)
                && Math.round(calcH(coord) * 100000) < Math.round(calcH(currentPos) * 100000)) {
          currentPos = coord.clone();
        }
      }

      processed.add(currentPos);
      for(int r = 0; r < toSearch.size(); r++) {
        if(toSearch.get(r)[0] == currentPos[0] && toSearch.get(r)[1] == currentPos[1]) {
          toSearch.remove(r);
        }
        if(toSearch.size() > 0) {
        }
      }

      if((currentPos[0]) == (targetPos[0]) && (currentPos[1]) == (targetPos[1])) {
        int[] currentPathCoord = targetPos.clone();
        ArrayList<int[]> path = new ArrayList<int[]>();
        int count = 1000;
        while(!(currentPathCoord[0] == startingPos[0] && currentPathCoord[1] == startingPos[1])) {
          path.add(currentPathCoord);
          int[] tempCoord = currentPathCoord.clone();
          tempCoord[0] = connections[currentPathCoord[0]][currentPathCoord[1]][0];
          tempCoord[1] = connections[currentPathCoord[0]][currentPathCoord[1]][1];
          System.out.println(tempCoord[0] + ", " + tempCoord[1]);
          currentPathCoord = tempCoord.clone();
          count--;
          if(count < 0) {
            //it dies but i dont want to deal with this yet
          }
        }
        //path.add(currentPathCoord);
        return path;

      }

      for(int[] neighbour : getNeighbours(currentPos, processed)) {
        boolean inToSearch = toSearch.contains(neighbour);

        double costToNeighbour = gVals[currentPos[0]][currentPos[1]] + getDistance(currentPos, neighbour);

        if(!inToSearch || costToNeighbour < gVals[neighbour[0]][neighbour[1]]) {
          gVals[neighbour[0]][neighbour[1]] = costToNeighbour;
          connections[neighbour[0]][neighbour[1]][0] = currentPos[0];
          connections[neighbour[0]][neighbour[1]][1] = currentPos[1];

          if(!inToSearch) {
            toSearch.add(neighbour);
          }
        }
      }
    }
    ArrayList<int[]> nothing = new ArrayList<int[]>();
    return nothing;
  }


  public String printPoints(int[] targetCoords) {
    String coords = "";
    for(int[] n : generatePath(targetCoords)) {
      coords = coords + (n[0] + ", " + n[1] + "\n");
    }
    return coords;
  }

  public int[] printNextPoint(int[] targetCoords) {
    int[] coords = new int[2];
    for(int[] n : generatePath(targetCoords)) {
      coords = n;
    }
    return coords;
  }

  @Override
  public void execute() {
    PathFollowTargetPos.currentTarget = printNextPoint(PathFollowTargetPos.finalPos);
    System.out.println(PathFollowTargetPos.finalPos[0] + ", " + PathFollowTargetPos.finalPos[1]);
  }

  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
