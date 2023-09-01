#include "eigen_01_point_loading.h"
#include "eigen_kdtree.h"
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

using ContainerType = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f> >;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

using Point2D = Vector2f;
using ContainerType2D = std::list<Point2D>;

int main(int argc, char** argv) {
  // generate 1000 random points
  int num_points  = 1000;
  ContainerType kd_points(num_points);
  for (auto& v: kd_points) {
    v=Vector3f::Random()*100;
  }
  
  // construct a kd_tree, with leaf size 10
  // the construction reshuffles the items in the container
  TreeNodeType kd_tree(kd_points.begin(), kd_points.end(), 10);

  float ball_radius=10;
  
  // we search each point in the input set. We need to find a match
  for (auto p: kd_points) {
    TreeNodeType::AnswerType neighbors;
    kd_tree.fastSearch(neighbors, p, ball_radius);
    int num_fast=neighbors.size();

    neighbors.clear();
    kd_tree.fullSearch(neighbors, p, ball_radius);
    int num_full=neighbors.size();

    neighbors.clear();
    bruteForceSearch(neighbors,
                     kd_points.begin(),
                     kd_points.end(),
                     p,
                     ball_radius);
    int num_true=neighbors.size();
    
    neighbors.clear();
    cerr << "(" << num_fast << "/" << num_full << "/" << num_true << ")";

    if (num_full!=num_true) {
      cerr << "Something wrong" << endl;
      exit(0);
    }
  }
  cerr << endl;

  /* 
     todo for you:
     1. modify this program to operate on 2D points stored in std::lists;
     2. modify this program so that it loads two sets of 2D points, and looks for the
        best match of each point in the second set and in the first set,
        within a user specified range.
        The program should output a text file in the form

        pa1.x pa1.y
        pb1.x pb1.y
        <emptyline>
        pa2.x pa2.y
        pb2.x pb2.y
        <emptyline>
        ...
        
        if no match is found no pair is written.
        Here pa and pb are the points in the first and second set
   */

    // Generate random points for set1 and set2 (2D sets)
    int numPoints = 1000;
    
    ContainerType2D set1, set2;

    for (int i = 0; i < numPoints; ++i) {
        set1.push_back(Point2D::Random() * 100);
        set2.push_back(Point2D::Random() * 100);
    }

    cerr << "-------------------------------" << endl;
    cerr << "Successfully created 2D points." << endl;

    // Open a text file for output
    ofstream outputFile("point_matches.txt");
    if (!outputFile.is_open()) {
        cerr << "Error: Could not open output file." << endl;
        return 1;
    }

    // Find the best matches and write them to the output file.
    //
    // This is needed in order to find the best match of each point
    // in the 1st and 2nd set, within a user specified range.
    //
    for (const Point2D& pa : set1) { // We take one single point from the set1 and match it against each point in set2
        Point2D bestMatch; // Let's define a bestMatch point that we would like to output (in case)
        float bestDistance = ball_radius; // Initialize the best distance with the ball radius.

        for (const Point2D& pb : set2) { // For each point in set2
            float distance = (pa - pb).norm(); // Euclidean distance

            // If this distance is lower than the radius we specified and also lower than the best distance recorded
            // Let's define pb as the bestMatch and let's update the best distance.
            if (distance <= ball_radius && distance < bestDistance) {
                bestMatch = pb;
                bestDistance = distance;
            }
        }

        // We want to save the points in the file only if we successfully found one.
        if (bestDistance < ball_radius) {
            outputFile << pa.x() << " " << pa.y() << endl;
            outputFile << bestMatch.x() << " " << bestMatch.y() << endl;
        } else {
            outputFile << "<emptyline>" << endl;

        }
    }

    // Close the output file
    outputFile.close();

    cerr << "Successfully found best matches. Output saved in 'point_matches.txt'." << endl;

    return 0;

}



