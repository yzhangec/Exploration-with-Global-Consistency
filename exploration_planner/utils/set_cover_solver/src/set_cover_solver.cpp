#include <set_cover_solver/set_cover_solver.h>

namespace fast_planner {
SetCoverSolver::SetCoverSolver(const SetCoverProblem &problem) {
  problem_ = make_shared<SetCoverProblem>(problem);
  solution_.reset(new SetCoverSolution);
  grid_visb_.assign(problem_->num_elements_, problem_->num_visb_);
}

void SetCoverSolver::initProblem(const SetCoverProblem &problem) {
  problem_ = make_shared<SetCoverProblem>(problem);
  solution_.reset(new SetCoverSolution);
  grid_visb_.assign(problem_->num_elements_, problem_->num_visb_);
}

void SetCoverSolver::setThreshold(const double &th) { gain_th_ = th; }

// Assumption: the sets are already sorted
// Greedy solution
void SetCoverSolver::solve() {
  int num_selected_elements = 0;
  int idx = -1;
  double gain = numeric_limits<double>::max();

  cout << "[Set Cover] Start solve set cover problem" << endl;

  while (num_selected_elements < problem_->num_elements_ && gain > gain_th_) {
    // find next set with smallest effectiveness (alpha: average cost per newly covered node)
    nextSet(idx);
    if (idx == -1) break;
    // cout << "The index of seleted set is: " << idx << endl;

    problem_->masks_[idx] = true;  // selected

    gain = 0.0;
    for (int i : problem_->sets_->at(idx).second) {
      if (grid_visb_[i] > 0) {
        gain += 1.0;
        grid_visb_[i]--;
      }
      if (grid_visb_[i] == 0) {
        // covered_grids_.push_back(i);
        num_selected_elements++;
        // for (auto it_set = problem_->sets_.begin(); it_set != problem_->sets_.end(); ++it_set) {
        //   if (it_set - problem_->sets_.begin() == idx) continue;
        //   for (auto it_ele = it_set->second.begin(); it_ele != it_set->second.end();) {
        //     if (*it_ele == i) {
        //       it_ele = it_set->second.erase(it_ele);
        //     } else {
        //       it_ele++;
        //     }
        //   }
        // }
      }
    }
    // gain = num_selected_elements - num_selected_elements_last;
    // cout << "The gain of this iteration is: " << gain << endl;

    // num_selected_elements += problem_->sets_->at(idx].size();  // only new elements left in the set
    // num_selected_elements = covered_grids_.size();
    solution_->num_selected_sets_++;
    solution_->cost_ += problem_->costs_->at(idx);
  }

  solution_->masks_.resize(problem_->masks_.size(), false);
  for (int i = 0; i < problem_->num_sets_; i++) {
    if (problem_->masks_[i]) {
      solution_->set_indices_.push_back(problem_->sets_->at(i).first);
      solution_->masks_[problem_->sets_->at(i).first] = true;
    }
  }

  std::sort(solution_->set_indices_.begin(), solution_->set_indices_.end());
  // outputSolution();
}

void SetCoverSolver::nextSet(int &idx) {
  double min_alpha = std::numeric_limits<double>::max();  // record min effectiveness
  int last_idx = idx;

  for (int i = 0; i < problem_->num_sets_; i++) {
    if (problem_->masks_[i] || problem_->sets_->at(i).second.size() == 0) continue;

    int size = problem_->sets_->at(i).second.size();
    for (int j = 0; j < problem_->sets_->at(i).second.size(); j++) {
      if (grid_visb_[problem_->sets_->at(i).second[j]] <= 0) {
        size--;
      }
    }

    // cost / num of new elements
    if (size > 0) {
      double alpha = (double)(problem_->costs_->at(i)) / size;

      if (alpha <= min_alpha) {
        min_alpha = alpha;
        idx = i;
      }
    }
  }

  if (idx == last_idx) idx = -1;
}

void SetCoverSolver::getSolution(SetCoverSolution &solution) { solution = *solution_; }

void SetCoverSolver::outputSolution() {
  cout << "Solution sets number: " << solution_->num_selected_sets_ << endl;
  cout << "Solution total cost: " << solution_->cost_ << endl;

  cout << "Solution selected idx: ";
  for (auto idx : solution_->set_indices_) cout << idx << " ";
  cout << endl;
}

}  // namespace fast_planner