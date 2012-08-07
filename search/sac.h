#ifndef SAC_H
#define SAC_H

#include "globals.h"
#include <map>
#include <vector>

bool use_sac = true;

using namespace std;

class Operator;
class State;

// The overall architecture of the SAC extension.
// Everything that is related to the generic structure of the planning graph
// and the propagation logic on the graph should go to StructuralGraph.
// This SAC should just have pick_operator that does the lightweight operator
// picking work once the propagation is done.

class SAC {
 public:
  SAC();
     
  void pick_operators(
      vector <const Operator*>& ops,
      vector <const Operator*>& return_ops);

  void emit_operator(
      const State& current_state,
      vector<const Operator*> &ops,
      vector<const Operator*> &final_ops);
      
 private:
  // Sparse version for holding ready-propagation tuples
  vector<pair<int, int> > to_propagate;
  map<pair<int, int>, vector<pair<int, int> > > g_tuple_map;
  map < pair< int, int> , int > g_tuple_address;
  
  vector<int> goal_position;
  int propagate(const State& current_state);
};

#endif // SAC_H
