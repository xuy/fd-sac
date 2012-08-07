// This file is part of the FastDownward + SAC package.  For more details of
// the SAC algorithm, refer to: TODO.  We use a simply and efficient algorithm
// to calculate action closures.  we define a <var, value> pair as a tuple in
// the goal DTG, <var, value> is our landmark we can pre-built the permuted
// index between <var, value> and <var, value>, those four tuples are not that
// large in the computing, we start from any goal tuple, iterate through our
// pre-built permuted index and stop at the current_state tuple (which is the
// dead-end) by doing this, we can finally propagate to a set of tuples, those
// tuples are the var-values that really matter to a goal then at the current,
// only collect those actions that have intersection with the tuple set.  For
// memory allocation and other things, must be called before the search.

// TODO(xuy):

// Clearly split the functions between StructuralGraph and SCC.
// Ideally, StructuralGraph does not know anything about the planning problem
// except doing some graph base operations.
// SAC knows a lot about planning and will do commucatitive pruning.
// Overall design:
//   Everything related the graphic structure of the planning problem should go to
//      the structural_graph class.
//   SAC should use that class to propagate, but not to do anything special.

// - Porting all the landmark logic in DTG.cc to this file.???
// It is necessary give our naive implementation?
#include "sac.h"

#include "domain_transition_graph.h"
#include "operator.h"
#include "state.h"
#include "structural_graph.h"

#include <map>
#include <vector>

using namespace std;

// Maps each tuple to a linear address.
// These two things are for pick opeartor
// pre_mark = (int*) malloc(sizeof(int) * g_variable_domain.size());
  // post_mark = (int*) malloc(sizeof(int) * g_variable_domain.size());


enum PropagateState {
    WHITE,
    GRAY,
    BLACK,
    NOGOAL
};

/*
const int WHITE = 0;
const int GRAY  = 1;
const int BLACK = 2;
const int NOGOAL = -2;
*/

int action_removed_count = 0;



// Those two are used for checking the commutativity of an operator
// with a selected "set" (instead of an op)
int *pre_mark;
int *post_mark;

// Data structure moved from global.

// Goal_position is a vector that holds the position of goals for EACH DTG.
// DTGs that does not have goal will have a NOGOAL entry.
SAC::SAC() {
  for (int var = 0; var < g_transition_graphs.size(); var++ ) {
    goal_position[var] = NOGOAL;
  }
  for (int i = 0; i < g_goal.size(); i++) {
    // TODO(xuy): change the logic here. The following logic was for DTG.
    // Figure out the usage here and remove this.
    goal_position[g_goal[i].first]->set_goal(g_goal[i].second);
  }
}



void clear_mark () {
    memset( pre_mark, 0, sizeof(int) * g_variable_domain.size());
    memset( post_mark, 0, sizeof(int) * g_variable_domain.size());
}

void mark(State& current_state, const Operator* op) {
    const vector< Prevail > & prevail = op->get_prevail();
    const vector< PrePost > & prepost = op->get_pre_post();

    // Mark as precondition 
    for (int i=0; i< prevail.size(); i++) {
        pre_mark[prevail[i].var] = 1;
    }

    for (int j = 0; j < prepost.size(); j++) {
        if (prepost[j].pre != -1 )  
            pre_mark[prepost[j].var] = 1;
    }
    
    // Mark delete effects 
    for (int j = 0; j < prepost.size(); j++) {
        //the value will change  
        if ( prepost[j].post != current_state[prepost[j].var])
            post_mark[prepost[j].var] = 1;
    }
}

bool is_commutative(State& current_state, const Operator* op) {
  // check if op will delete someone's preconditon 
  const vector< PrePost > & prepost = op->get_pre_post();
  for (int j = 0; j < prepost.size(); j++) {
        int var = prepost[j].var;
        if (prepost[j].post != current_state[var] && pre_mark[var] ) 
          return false;
  }

  const vector< Prevail > & prevail = op->get_prevail();
  // check if op requires some precondtion  
  for (int i=0; i< prevail.size(); i++) {
      if (post_mark[prevail[i].prev]) return false;
  }

  for (int j = 0; j < prepost.size(); j++) {
      if (prepost[j].pre != -1  && pre_mark[prepost[j].var]) return false;
  }
  return true;
}

bool is_operator_related(const Operator* op) {
    const vector<PrePost>& pre_post = op->get_pre_post();
    for (int i = 0; i < pre_post.size(); i++) {
        pair <int, int> affected = 
            make_pair(pre_post[i].var, pre_post[i].post);
        if (structural_graph[actions_sgindex[affected]]) return true;
    }
    return false;
}


// The idea is very simple: we start from a DTG that has an unachieved goal,
// and mark that goal as the starting point of the propagation. We iteratively


// A classic DFS algorithm that will terminate at current state tuple.
// This function belongs to StructuralGraph.
bool SAC::propagate(const State& current_state) {
  bool should_continue = false;
  pair <int, int> p;
  vector< pair < int, int > >  temp;
  for (int index = 0; index <to_propagate.size(); index++)   {
    int my_address = actions_sgindex[to_propagate[index]] ;
    if (structural_graph[my_address] == BLACK)  continue;
    structural_graph[ my_address ] = BLACK;
    // the tuple we are currently one is the current state, terminate this one.
    if (current_state[to_propagate[index].first] == to_propagate[index].second) 
    {
      structural_graph[my_address] = BLACK;
      continue;
    }
    for (int k = 0; k < g_tuple_map[to_propagate[index]].size(); k ++) {
      // Find the tuple that we are going to propagate to.
      int address = actions_sgindex[g_tuple_map[to_propagate[index]][k]];
      if (structural_graph[address] == WHITE) {
        structural_graph[address] =  GRAY;
        temp.push_back(g_tuple_map[to_propagate[index]][k]); 
        should_continue = true;
      }
    }
  }
  to_propagate = temp;
  return should_continue;
}

// A very naive way of picking goal. We select a goal variable and use the 
// goal of that variable as the over all goal to start propagation.
// Returns the index of the DTG that contains the goal we picked.
// TODO(xuy): in future, use heuristics to pick the best DTG to start with.
int SAC::mark_first_unachieved_goal() {
  int current_goal;
  int var;
  for (var = 0; var < g_transition_graphs.size(); var++) {
    current_goal = goal_position[var];
    if (current_goal != NOGOAL && current_state[var] != current_goal) {
      pair< int , int> var_goal_pair = make_pair(var, current_goal); 
      structural_graph[ actions_sgindex[var_goal_pair] ] = GRAY;
      to_propagate.clear();
      to_propagate.push_back(var_goal_pair);
      break;
    }
  }
  return var;
}

// TODO: figure out why this function is here.
// are we using sac as a success generator? or something else.
void sac_generate_successor(State& current_state,
							vector<const Operator*> &ops) {
	vector<const Operator*> temp_ops;
	if (use_sac) {
		g_successor_generator->generate_applicable_ops(
				current_state, temp_ops);
		emit_operator(current_state, temp_ops, ops);
	}
	else {
		g_successor_generator->generate_applicable_ops(
				current_state, temp_ops);
	}
}

// Two public functions offered by SAC.
void SAC::emit_operator(
    State& current_state,
    vector<const Operator*> &ops,
    vector<const Operator*> &final_ops) {
  prepare_for_propagation();
  while (propagate(current_state)) {}
  pick_operators(current_state, ops, final_ops);
  action_removed_count += (ops.size() - final_ops.size());  
}

// Take in the current state and a set of operators, emit a vector of operators
// calculated using the SAC algorithm.
void SAC::pick_operators(
    const State& current_state,
    vector < const Operator* > & ops,
    vector< const Operator* > & return_ops) {
    // Step 1: collect all dotted operators
    // vector <const Operator *> selected;
    vector <const Operator *> not_selected;
    vector <const Operator *> temp;
    clear_mark();
    for (int i = 0; i < ops.size(); i++) {
        if (is_operator_related(ops[i])) {  
          return_ops.push_back(ops[i]);
          mark(current_state, ops[i]);
        }
        else {
          not_selected.push_back(ops[i]);
        }
    }

    /*/
    printf("removed begin \n") ;
    for (int j = 0; j < not_selected.size(); j++)
           not_selected[j]->dump(); 
    printf("removed end \n" );
    action_removed_count += not_selected.size();
    // **/
    
    // Step 2: return the non-communitative operators
    /* The non-commutative pairs can only be Pa Db or Pb Da (since a, b are both applicable)
     * To avoid checking them pairwise with complexity O(N^2), we
     * mark their precondition and post effects set on pre/post_mark .*/ 
    // Worse case: O(N)
    
    bool flag = true;
    while (flag) {
      flag  = false;
      temp.clear();
      for (int i = 0;  i < not_selected.size(); i++) {
        if ( ! is_commutative(current_state, not_selected[i]) ) {
          flag = true;
          return_ops.push_back(not_selected[i]);
          mark(current_state, not_selected[i]);
        }
        else {
          temp.push_back(not_selected[i]);
        }
      }
      not_selected = temp;
    }
    /*/
    for (int i =0; i < selected.size(); i++)
        for (int j = 0; j < not_selected.size(); j++) {
            int result = selected[i]->is_comm(not_selected[j]); 
            if ( result!= 1) {
              printf ("===========not comm===begin======\n" );
              selected[i]->dump();
              not_selected[j]->dump();
              printf ("type %d\n", result); 
              printf ("===========not comm===end========\n" );
            }
        }
    // **/
}

