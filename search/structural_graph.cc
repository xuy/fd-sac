#include "structural_graph.h"

// Read the tuple graph stored in the preproecssed input file.
StructuralGraph::StructuralGraph(istream &in){
  // Linear encoding for the fluents.
  int count = 0;
  for (int i = 0; i < g_variable_domain.size(); i++ ) {
    for (int j = 0; j < g_transition_graphs[i]->range(); j++) {
      fluent_encoding.insert(make_pair( make_pair(i, j), count ) ) ;
      count ++;
    }
  }
  total_fluent_count = count;
  // efluent_graph.resize(count);
  read_fluent_dependency(in);    
}

// The actural code to read tuple mapping, will be added to read_everything function
void StructuralGraph::read_fluent_dependency(istream& in) {
  int map_counter; 
  int var, value, target_var, target_value, fluent_count;
  in >> map_counter;
  for (int index = 0; index < map_counter; ++index) {
    in >> var >> value >> fluent_count;
    Fluent source_fluent = make_pair(var, value);
    int source_index = fluent_encoding[source_fluent];
    efluent_graph[source_index] = vector<int>();
    
    for(int index2 = 0; index2 < fluent_count; ++index2) {
        in >> target_var >> target_value;
        int target_index = fluent_encoding[make_pair(target_var, target_value)];
        efluent_graph[source_index].push_back(target_index);
    }
  }  
}

/*
static void prepare_for_propagation() {
    // Prepare for data structure.
    memset (structural_graph, 0,  sizeof(int) * structural_graph_size);
    to_propagate.clear();
    mark_first_unachieved_goal();
    // Unarchived Fluent.
}*/
/* This function is called by read_everything in global.cc
 * Despite the name, the real read_tuple_map thing happends
 * in the read_tuple_map_helper() function */




