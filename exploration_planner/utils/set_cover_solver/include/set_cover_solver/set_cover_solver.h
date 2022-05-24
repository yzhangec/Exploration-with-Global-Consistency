#ifndef SET_COVER_SOLVER_H
#define SET_COVER_SOLVER_H

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <vector>

using namespace std;
namespace fast_planner {
struct SetCoverProblem {
  typedef std::shared_ptr<SetCoverProblem> Ptr;
  typedef std::shared_ptr<const SetCoverProblem> ConstPtr;

  int num_sets_;      // number of frames
  int num_elements_;  // number of grids, the Universe
  int num_visb_;
  vector<bool> masks_;                                    // to record if the frame is selected
  std::shared_ptr<vector<double> const> costs_;                    // c1, c2, ..., ck
  std::shared_ptr<vector<pair<int, vector<int>>>> sets_;  // S1, S2, ..., Sk

  SetCoverProblem(const int &num_sets, const int &num_elements, const int &num_visb,
                  const vector<double> &costs, vector<pair<int, vector<int>>> &sets)
      : num_sets_(num_sets), num_elements_(num_elements), num_visb_(num_visb) {
    masks_.resize(num_sets, false);

    costs_ = make_shared<vector<double>>(costs);
    sets_ = make_shared<vector<pair<int, vector<int>>>>(sets);

    // cout << "Initialize problem with: " << endl;
    // cout << "Number of elements: " << num_elements_ << endl;
    // cout << "Number of sets: " << num_sets_ << endl;
    // cout << "Number of visibility for each grid: " << num_visb_ << endl;
  }
};

struct SetCoverSolution {
  typedef std::shared_ptr<SetCoverSolution> Ptr;
  typedef std::shared_ptr<const SetCoverSolution> ConstPtr;

  int num_selected_sets_;
  double cost_;
  // shared_ptr<vector<vector<int>>> sets_;
  vector<int> set_indices_;  // idx of selected sets
  vector<bool> masks_;       // boolean indicators for which sets are selected

  SetCoverSolution() : num_selected_sets_(0), cost_(0) {}
};

class SetCoverSolver {
 public:
  SetCoverSolver() {}
  ~SetCoverSolver() {}

  SetCoverSolver(const SetCoverProblem &problem);

  void initProblem(const SetCoverProblem &problem);
  void setThreshold(const double &th);
  void solve();
  void nextSet(int &idx);
  void getSolution(SetCoverSolution &solution);
  void outputSolution();
  std::chrono::duration<double> duration;

 private:
  double gain_th_;
  vector<int> grid_visb_;
  // vector<int> covered_grids_;
  SetCoverProblem::Ptr problem_;
  SetCoverSolution::Ptr solution_;
};
}  // namespace fast_planner

#endif