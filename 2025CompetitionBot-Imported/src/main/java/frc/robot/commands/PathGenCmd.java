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
    boolean failCheck1 = false;
    boolean failCheck2 = false;
    //creates the arraylist where the neighbours will be stored
    ArrayList<int[]> neighbours = new ArrayList<int[]>();
    //iterates over numbers from -1 to 1 inclusive (for x value)
    for(int i = -1; i < 2; i++) {
      //iterates over numbers from -1 to 1 inclusive (for y value)
      for(int j = -1; j < 2; j++) {
        //creates a temporary position variable to represent the neighbour being checked for validity (adds i and j values to x and y values, leading to 9 overall checks encompassing all the surrounding coordinates)
        int[] temp = {pos[0] + i, pos[1] + j};
        //iterates over the already processed coordinates
        for(int[] f : processed) {
          //If the coordinate has already been processed, it is not a possible move and fail is set to true
          if(((temp[0]) == (f[0]) && (temp[1]) == (f[1]))) {
            failCheck1 = true;
          }
        }
        //iterates over numbers from -15 to 15 inclusive (for x value)
        for(int k = -15; k < 16; k++) {
          //iterates over numbers from -15 to 15 inclusive (for y value)
          for(int l = -15; l < 16; l++) {
            //Checks if the neighbour being evaluated is inside the field perimeter, is a 0 value (not an occupied 1 space), and is not the robot's current coordinate (i and j are both 0)
            if(!(pos[0] + i + k >= 0 && pos[0] + i + k <= field.length && pos[1] + j + l >= 0 && pos[1] + j + l <= field[0].length
                && field[pos[0] + i][pos[1] + j] == 0 && (i != 0 || j != 0))) {
              //sets fail to ture, meaning that this neighbour is not a possible candidate
              failCheck2 = true;
              continue;
            }
            if(failCheck2){
              continue;
            }
          }
        }
        //If fail 1 and 2 are false, add it to the list of neighbours
        if(!failCheck1 && !failCheck2) {
          neighbours.add(temp);
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
    //adds the starting pos to the toSeach arraylist
    toSearch.add(startingPos);

    //runs a loop while toSearch is not empty (there are still more coordinates to travel to before reaching the finish)
    while(!toSearch.isEmpty()) {
      //sets currentPos to the starting pos of the robot (currentPos will change as the path is generated, it is what moves during geenration)
      currentPos = toSearch.get(0);

      //runs a loop for every coordinate in toSearch
      for(int[] coord : toSearch) {
        //checks if the calcf (euclidian distacne to end point) of the next coordinate to search is less than the calcf of the current coordinate or if the calcf is the same but the calch is less
        if(calcF(coord) < calcF(currentPos)
            || Math.round((calcF(coord) * 100000)) == Math.round(calcF(currentPos) * 100000)
                && Math.round(calcH(coord) * 100000) < Math.round(calcH(currentPos) * 100000)) {
          //sets the currentPos to the coordinate just checkd
          currentPos = coord.clone();
        }
      }

      //adds currentPos to processed
      processed.add(currentPos);
      //runs a loop for the amount of indices in toSearch
      for(int r = 0; r < toSearch.size(); r++) {
        //checks if the coordinate in toSeach is the same as the currentPos
        if(toSearch.get(r)[0] == currentPos[0] && toSearch.get(r)[1] == currentPos[1]) {
          //removes the coordinate from toSearch
          toSearch.remove(r);
        }
        //checks if to search is not empty
        if(toSearch.size() > 0) {
        }
      }

      //Checks if currentPos (the varibale used as the furthest value in teh generated path) is equal to the target pos
      if((currentPos[0]) == (targetPos[0]) && (currentPos[1]) == (targetPos[1])) {
        //creates a varibale to store the coordinate  of teh current path, and sets it to the target pos
        int[] currentPathCoord = targetPos.clone();
        //creates an array list of int arrays to store the complete path
        ArrayList<int[]> path = new ArrayList<int[]>();
        //sets count (used for a timeout if the path is too long)
        int count = 1000;
        //runs a loop while teh current path coord isnt equal to teh starting pos (parses backwards through the path)
        while(!(currentPathCoord[0] == startingPos[0] && currentPathCoord[1] == startingPos[1])) {
          //adds currentPathCoord to the path arraylist
          path.add(currentPathCoord);
          //creates a temp coord which is equal to the current path coord (not really necessary)
          int[] tempCoord = currentPathCoord.clone();
          //sets the temp coord equal to the neighbour of the current path coord
          tempCoord[0] = connections[currentPathCoord[0]][currentPathCoord[1]][0];
          tempCoord[1] = connections[currentPathCoord[0]][currentPathCoord[1]][1];
          //Prints teh temp coord (for debugging)
          System.out.println(tempCoord[0] + ", " + tempCoord[1]);
          //sets the currentpathcoord to the value of the temp coord
          currentPathCoord = tempCoord.clone();
          //reduces count by 1
          count--;
          //checks if count is 0
          if(count < 0) {
            //it dies but i dont want to deal with this yet
          }
        }
        //path.add(currentPathCoord);
        //return the path
        return path;

      }

      //loop through all neighbours of the current pos
      for(int[] neighbour : getNeighbours(currentPos, processed)) {
        //sets a boolean to true if neighbour is included in tosearch
        boolean inToSearch = toSearch.contains(neighbour);

        //sets the cost to get to the neighbour to the distance betwen the current pos and the neighbour (1 for vetical/horizntal, 1.41 for diagonal) + the gvals of current pos
        double costToNeighbour = gVals[currentPos[0]][currentPos[1]] + getDistance(currentPos, neighbour);

        //checks if the nieghbour being checked is in tosearch or if the variable above is less than the gvals of the neigbour (in which case there would be a faster way to reach tne nighbouring pos)
        if(!inToSearch || costToNeighbour < gVals[neighbour[0]][neighbour[1]]) {
          //sets the gvals of the neighbour to the calculated cost to reach it so far
          gVals[neighbour[0]][neighbour[1]] = costToNeighbour;
          //adds the the current pos as a connection of the neighbour
          connections[neighbour[0]][neighbour[1]][0] = currentPos[0];
          connections[neighbour[0]][neighbour[1]][1] = currentPos[1];

          //checks if in to search is false
          if(!inToSearch) {
            //adds the neighbour to tosearch
            toSearch.add(neighbour);
          }
        }
      }
    }
    //returns an empty array list of no other conditions are satisfied that provide a return (e.g. there are no points provided to search)
    ArrayList<int[]> nothing = new ArrayList<int[]>();
    return nothing;
  }


  //testing
  public String printPoints(int[] targetCoords) {
    String coords = "";
    for(int[] n : generatePath(targetCoords)) {
      coords = coords + (n[0] + ", " + n[1] + "\n");
    }
    return coords;
  }

  //testing
  public int[] printNextPoint(int[] targetCoords) {
    int[] coords = new int[2];
    for(int[] n : generatePath(targetCoords)) {
      coords = n;
    }
    return coords;
  }

  @Override
  public void execute() {
    //test prints
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
