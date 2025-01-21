#ifndef DEBUG_H
#define DEBUG_H

#include "worldbuilder.h"
#include <fstream>

namespace debug{

b2Vec2 GetWorldPoints(b2Body*, b2Vec2 );

void print_pose(const b2Transform& p);

void print_matrix(cv::Mat);

std::vector<b2Vec2> GetBodies( b2World*);

}



#endif