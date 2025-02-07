#include "configurator.h"
#include <chrono>



void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}


bool Configurator::Spawner(){ 
	//PREPARE VECTORS TO RECEIVE DATA
	if (data2fp.empty()){
		return 1;
	}
	iteration++; //iteration set in getVelocity
	worldBuilder.iteration++;
	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	bool explored=0;
	if (transitionSystem.m_vertices.size()==1 && iteration<=1){
		movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingEdge].direction=DEFAULT;
		currentTask.action.init(transitionSystem[movingEdge].direction);
	}
	if (currentTask.action.getOmega()!=0 && currentTask.motorStep<(transitionSystem[movingEdge].step)){
		return 1;
	}
	float _simulationStep=simulationStep;
	adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
	worldBuilder.buildWorld(world, data2fp, transitionSystem[movingVertex].start, currentTask.direction); //was g[v].endPose
	simResult result = simulate(currentTask, world, _simulationStep); //transitionSystem[currentVertex],transitionSystem[currentVertex],
	gt::fill(result, transitionSystem[currentVertex].ID, &transitionSystem[currentEdge]);
	currentTask.change = transitionSystem[currentVertex].outcome!=simResult::successful;
	worldBuilder.resetBodies();
	//printf("end explor\n");
	return 1;
}



simResult Configurator::simulate(Task  t, b2World & w, float _simulationStep){ //State& state, State src, 
		//EVALUATE NODE()
	simResult result;
	float distance=BOX2DRANGE;
	if (controlGoal.disturbance.isValid()){
		distance= controlGoal.disturbance.getPosition().Length();
	}
	float remaining=distance/controlGoal.action.getLinearSpeed();
	Robot robot(&w);
	robot.body->SetTransform(t.start.p, t.start.q.GetAngle());
	b2AABB sensor_aabb=worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
	result =t.willCollide(w, iteration, robot.body, debugOn, remaining, _simulationStep); //default start from 0
	//approximate angle to avoid stupid rounding errors
	float approximated_angle=approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode);
	result.endPose.q.Set(approximated_angle);
	return result;
	}

void Configurator::start(){
	if (ci == NULL){
		throw std::invalid_argument("no data interface found");
		return;
	}
	running =1;
	if (t!=NULL){ //already running
		return;
	}
	t= new std::thread(Configurator::run, this);

}

void Configurator::stop(){
	running =0;
	if (t!=NULL){
		t->join();
		delete t;
		t=NULL;
	}
}

void Configurator::registerInterface(ConfiguratorInterface * _ci){
	ci = _ci;
}

void Configurator::run(Configurator * c){
	while (c->running){
		if (c->ci->stop){
			c->ci=NULL;
		}
		if (c->ci == NULL){
			printf("null pointer to interface\n");
			c->running=0;
			return;
		}
		if (c == NULL){
			printf("null pointer to configurator\n");
			c->running=0;
			return;
		}
		if (c->ci->isReady()){
			c->ci->ready=0;
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
		}
	}

}

void Configurator::adjustStepDistance(vertexDescriptor v, TransitionSystem &g, Task * t, float& step, std::pair<bool,vertexDescriptor> tgt){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(theta_obs);
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		if (t->direction==getOppositeDirection(currentTask.direction).second){
			remainingAngle=M_PI-remainingAngle;
		}
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered
	//printf("adjusted\n");
}

ExecutionError Configurator::trackTaskExecution(Task & t){
	ExecutionError error;
	//could keep track of of DI in the State representing th einstruction

	//here sample point cloud available (perhaps using the robot sensor)
	//get distance and/or angle
	//update t with new info

	if(t.motorStep==0){ //this should be changed to t.checkEnded (here is a good point to inject disturbance from vertex to task)
		t.change=1;
	}

	return error; //garbage, return void but can be used if you want to implment some learning
}


int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (simulationStep)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }

std::vector <vertexDescriptor> Configurator::changeTask(bool b, int &ogStep, std::vector <vertexDescriptor> pv){
	if (!b){
		return pv;
	}
	if (transitionSystem[currentVertex].Dn.isValid()){
		currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
	}
	else if(currentTask.direction!=DEFAULT){
			currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
	}
	else{
		currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
	}
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	currentTask.motorStep = motorStep(currentTask.getAction());
	transitionSystem[movingEdge].step=currentTask.motorStep;
	printf("changed to %f\n", currentTask.action.getOmega());
	ogStep = currentTask.motorStep;
	return pv;
}

void Configurator::trackDisturbance(b2Transform & pose, Task::Action a, float error){
	float angleTurned =MOTOR_CALLBACK*a.getOmega();
	pose.q.Set(pose.q.GetAngle()-angleTurned);	
	float distanceTraversed = 0;
	float initialL = pose.p.Length();
	if(fabs(error)<TRACKING_ERROR_TOLERANCE){
		distanceTraversed= MOTOR_CALLBACK*a.getLinearSpeed();
	}
	else{
		distanceTraversed=error;
	}
	pose.p.x=cos(pose.q.GetAngle())*initialL-cos(angleTurned)*distanceTraversed;
	pose.p.y = sin(pose.q.GetAngle())*initialL-sin(angleTurned)*distanceTraversed;
}




void Configurator::applyAffineTrans(const b2Transform& deltaPose, Task& task){
	math::applyAffineTrans(deltaPose, task.start);
	applyAffineTrans(deltaPose, task.disturbance);
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, TransitionSystem& g){
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
	if (*vIt!=movingVertex){
		math::applyAffineTrans(deltaPose, g[*vIt]);
	}
}
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, Disturbance& d){
	if (d.getAffIndex()!=NONE){
		math::applyAffineTrans(deltaPose, d.bf.pose);
	}
}


float Configurator::approximate_angle(const float & angle, const Direction & d, const simResult::resultType & outcome){
	float result=angle;
	if ((d==LEFT || d==RIGHT)&& outcome!=simResult::crashed){
		float ratio= angle/ANGLE_RESOLUTION;
		float decimal, integer;
		decimal=std::modf(ratio, &integer);
		if (decimal>=0.5){
			integer+=1;
		}		
		result=integer*ANGLE_RESOLUTION;
	}
	return result;
}

