#include "graphTools.h"


b2Transform State::start_from_disturbance()const{
	return Dn.pose()-start; //START
}

b2Transform State::end_from_disturbance()const{
	return Dn.pose()-endPose; //START
}

float State::distance(){
	return (start-endPose).p.Length();
}


float angle_subtract(float a1, float a2){
	float result = 0;
	if (fabs(a1)> 3*M_PI_4 || fabs(a2)> 3*M_PI_4){
		if (a1<0 & a2>0){ 
			a2-=2*M_PI;
		}
		else if(a2<0 & a1>0){
			a2+=2*M_PI;
		}
	}
	result=a1-a2;
}

void math::applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose){
	pose.q.Set(pose.q.GetAngle()-deltaPose.q.GetAngle());
	float og_x= pose.p.x, og_y=pose.p.y;
	pose.p.x= og_x* cos(deltaPose.q.GetAngle())+ og_y*sin(deltaPose.q.GetAngle());
	pose.p.y= og_y* cos(deltaPose.q.GetAngle())- og_x*sin(deltaPose.q.GetAngle());
	pose.p.x-=deltaPose.p.x;
	pose.p.y-=deltaPose.p.y;
}

void math::applyAffineTrans(const b2Transform& deltaPose, State& state){
	applyAffineTrans(deltaPose, state.endPose);
	applyAffineTrans(deltaPose, state.start);
	if (state.Dn.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Dn.bf.pose);
	}
	if (state.Di.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Di.bf.pose);
	}

}


void gt::fill(simResult sr, State* s, Edge* e){
	if (NULL!=s){
		s->Dn = sr.collision;
		s->endPose = sr.endPose;
		s->outcome = sr.resultCode;
		s->filled=true;
	}
}



bool operator!=(Transform const &t1, Transform const& t2){
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

bool operator==(Transform const &t1, Transform const& t2){
	return (t1.p.x == t2.p.x) && (t1.p.y == t2.p.y) && (t1.q.GetAngle() == t2.q.GetAngle());
}

void operator-=(Transform & t1, Transform const&t2){
	t1.p.x-=t2.p.x;
	t1.p.y-=t2.p.y;
	t1.q.Set(t1.q.GetAngle()-t2.q.GetAngle());
}

void operator+=(Transform & t1, Transform const&t2){
	t1.p.x+=t2.p.x;
	t1.p.y+=t2.p.y;
	t1.q.Set(t1.q.GetAngle()+t2.q.GetAngle());
}

Transform operator+(Transform const & t1, Transform const&t2){
	b2Transform result;
	result.p.x=t1.p.x+t2.p.x;
	result.p.y=t1.p.y+t2.p.y;
	result.q.Set(t1.q.GetAngle()+t2.q.GetAngle());
	return result;
}

Transform operator-(Transform const & t1, Transform const&t2){
	b2Transform result;
	result.p.x=t1.p.x-t2.p.x;
	result.p.y=t1.p.y-t2.p.y;
	result.q.Set(t1.q.GetAngle()-t2.q.GetAngle());
	return result;

}

Transform operator-(Transform const & t){
	b2Transform result;
	result.p.x=-(t.p.x);
	result.p.y=-(t.p.y);
	result.q.Set(-t.q.GetAngle());
	return result;

}

