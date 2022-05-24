#include <set_cover_solver/set_cover_solver.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

using namespace std;
using namespace fast_planner;

int main(int argc, char *argv[]) {
  std::ifstream infile(
      "/home/eason/workspace/plan_ws/src/FUEL/fuel_planner/utils/set_cover_solver/"
      "data/sc_1534_4");
  int num_elements, num_sets;
  string line;

  getline(infile, line);
  std::istringstream iss(line);
  iss >> num_elements >> num_sets;

  vector<double> costs;
  vector<pair<int, vector<int>>> sets;

  int line_count = 0;
  while (getline(infile, line)) {
    std::istringstream iss(line);
    double cost;
    iss >> cost;
    costs.push_back(cost);

    int element;
    vector<int> set;
    while (iss >> element) set.push_back(element);
    sets.push_back(make_pair(line_count++, set));
  }

  if (sets.size() != num_sets) {
    cout << "ERROR: number of sets does not match, size: " << sets.size() << ", num_sets: " << num_sets << endl;
    return 1;
  }

  SetCoverProblem problem(num_sets, num_elements, 1, costs, sets);
  SetCoverSolution solution;

  SetCoverSolver solver(problem);
  solver.setThreshold(0);

  auto start = std::chrono::system_clock::now();

  solver.solve();

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = end - start;
  // cout << "Selected duration: " << solver.duration.count() << "s" << endl;
  cout << "Program duration: " << duration.count() << "s" << endl;

  return 0;
}