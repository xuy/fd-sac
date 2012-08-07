#ifndef STRUCTURAL_GRAPH_H
#define STRUCTURAL_GRAPH_H

#include "domain_transition_graph.h"
#include "globals.h"

#include <iostream>
#include <vector>
#include <map>

using namespace std;

typedef pair<int, int> Fluent;

// A Structural Graph for a SAS+ planning problem is a graph that describes
// the dependencies between domain transition variables and their values.
// For instance, for a SAS+ planning problem with two variables x and y, if
// action a has precondition (x=1, y=1) and effect (x=2, y=2), then we have
// dependencies (x=2 -> x=1), (y=2 -> x=1), (y=2 -> y=1), and (x=2, y=1).
// It is easy to see that the vertices in a Structural Graph are (variable,
// value) pairs, and the edges are dependecies.

// The implementation encodes the (variable, value) pair into a integer
// so that it is much easier to represent the whole graph using a linear
// data structure, and to conduct DFS on it really quick. We call the
// (variable, value) pair "fluent" in this implementation. All our internal
// data structure used in this class are based on encoded fluent.

// We should only allow one instance of the SG, but multiple vectors in
// run time used for propagation.
// initiate_propagation(start_fluent, vector* to_propagate);
// propagate(to_propagate) // Invariant here is that after propage, everything
// in is black or white, no gray.

class StructuralGraph{
 public:
  // Replaces read_tuple_map.
  StructuralGraph(istream &in);
  
 private:
  void read_fluent_dependency(istream &in);
  
  int total_fluent_count;
  map<int, vector<int> > efluent_graph;
  map<Fluent, int> fluent_encoding;
};

#endif  // STRUCTURAL_GRAPH_H