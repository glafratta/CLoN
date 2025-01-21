 #include "debug.h"


b2Vec2 GetWorldPoints(b2Body* b, b2Vec2 v){
	b2Vec2 wp=b->GetWorldPoint(v);
	printf("x=%f, y=%f\t", wp.x, wp.y);
}

void debug::print_pose(const b2Transform& p){
	printf("x=%f, y=%f, theta=%f\n", p.p.x, p.p.y, p.q.GetAngle());
}

void debug::print_matrix(cv::Mat m){
	std::cout << "M = " << std::endl << " "  << m << std::endl << std::endl;
}

std::vector<b2Vec2> debug::GetBodies(b2World* w){
	std::vector<b2Vec2> result;
	for (b2Body * b=w->GetBodyList(); b; b=b->GetNext()){
		result.push_back(b->GetPosition());
	}
	return result;
}
