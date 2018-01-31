#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
  MatrixXd T_matrix(3, 3);
  T_matrix << pow(T, 3), pow(T, 4), pow(T, 5),
              3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
              6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

  MatrixXd S_matrix(3, 1);
  S_matrix << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
              end[1] - (start[1] + start[2] * T),
              end[2] - start[2];

  MatrixXd T_matrix_i = T_matrix.inverse();

  MatrixXd Accel = T_matrix_i * S_matrix;

  vector< double> result;
  for (int i = 0; i < start.size(); ++i)
  {
    if (i == 2) 
    {
      result.push_back(start[i] * 0.5);
    }
    else
    {
      result.push_back(start[i]);
    }
  }
  for (int i = 0; i < Accel.size(); ++i) {
    result.push_back(Accel(i));
  }

  /* for (int i = 0; i < result.size(); ++i) { */
  /*   cout << result[i] << endl; */
  /* } */
  /* cout << "DONE!" << endl; */
  return result;

    /* return {1,2,3,4,5,6}; */

}
