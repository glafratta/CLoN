#ifndef GENERAL_H
#define GENERAL_H
#include <set>
//#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp> //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <map>
#include <boost/property_map/property_map.hpp> //property map
//#include <boost/variant/get.hpp> //get function
#include <boost/graph/copy.hpp>
#include "disturbance.h"


float angle_subtract(float a1, float a2); //subtracts angles accounting for the fact that they're represented in a range of +/-pi


//elements of cognitive map
struct Edge{
	Direction direction=DEFAULT;
	int step=0; //to be removed
	Edge()=default;

	Edge(Direction d):direction(d){}

};


struct State{ //this represents an instruction for a Task to be executed
	Disturbance Di; //initial Disturbance
	Disturbance Dn; //new Disturbance
	b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)); 
	simResult::resultType outcome; //whether it crashes or not
	State* ID=this;
	bool filled=false;

	
	State()=default;

	State(const b2Transform &_start): start(_start){}

	State(const b2Transform &_start, const Disturbance& di): start(_start), Di(di){}

	b2Transform start_from_disturbance()const; //from Dn

	b2Transform end_from_disturbance()const; //from Dn

	float distance();

};

namespace math {
	void applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose);

	void applyAffineTrans(const b2Transform&, State& );
};


//custom operator for b2Transform
typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);
void operator-=(Transform &, Transform const&);
void operator+=(Transform &, Transform const&);
Transform operator+( Transform const &, Transform const &);
Transform operator-( Transform const &, Transform const &);
Transform operator-(Transform const &);


struct ExecutionError{ //structure to record task execution error

	ExecutionError(){}

	ExecutionError(float fr, float ft){
		_r=fr;
		_theta=ft;
	}

	float r(){ //distance error
		return _r;
	}

	float theta(){ //angle error
		return _theta;
	}

	void setTheta(float f){
		_theta=f;
	}

	void setR(float f){
		_r=f;
	}
	private:
	float _r=0;
	float _theta=0;
};


template <class I>
auto check_vector_for(std::vector <I>& vector, const I& item){ //check that item is in the vector
	for (int i=0; i<vector.size(); i++){
		if (vector[i]==item){
			return vector.begin()+i;
		}
	}
	return vector.end();
}

//SETUP OF TRANSITION SYSTEM: made up of states and edges

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, State, Edge> TransitionSystem;
typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

namespace gt{ //tools to modify transition system

	void fill(simResult, State* s=NULL, Edge* e=NULL);

}
#endif