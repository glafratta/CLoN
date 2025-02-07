#ifndef ROBOT_H
#define ROBOT_H
#include "user_settings.h"
#include<vector>
class Robot {
private: 
	b2FixtureDef fixtureDef;
public:
	b2Vec2 velocity = {0,0};
	b2Body* body;
	b2BodyDef bodyDef;

	Robot(b2World * world) {
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 0.0f);
		body = world->CreateBody(&bodyDef);
		//body->GetUserData().pointer = reinterpret_cast<uintptr_t>(this);
		body->GetUserData().pointer=reinterpret_cast<uintptr_t>(ROBOT_FLAG);
		b2Vec2 center(ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
		b2PolygonShape box;
		box.SetAsBox(ROBOT_HALFWIDTH, ROBOT_HALFLENGTH, center, ROBOT_BOX_OFFSET_ANGLE);
		fixtureDef.shape = &box;
		fixtureDef.friction =0;
		body->CreateFixture(&fixtureDef);
		
	}

	static std::vector<b2Vec2> zero_vertices(){
		b2Vec2 bl(-ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, -ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y);
		b2Vec2 br(ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, -ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y);
		b2Vec2 tl(-ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y);
		b2Vec2 tr(ROBOT_HALFWIDTH+ROBOT_BOX_OFFSET_X, ROBOT_HALFLENGTH+ROBOT_BOX_OFFSET_Y);
		std::vector <b2Vec2> result={bl, br, tl, tr};
		return result;
	}


};



#endif

