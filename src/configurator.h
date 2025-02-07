#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
//#include "worldbuilder.h"
#include <algorithm>
#include <sys/stat.h>
#include "debug.h"

const std::map<Direction, char*> dirmap={{DEFAULT, "DEFAULT"}, {LEFT, "LEFT"}, {RIGHT, "RIGHT"}, {STOP, "STOP"}, {UNDEFINED, "UNDEFINED"}};

class ConfiguratorInterface{
public:
	bool debugOn=0;
	int iteration=0;
	CoordinateContainer data2fp;
	bool ready=0;
	bool newData=0;
	bool stop=0;
	void setReady(bool b);
	bool isReady();

};

class Configurator{
protected:
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	Task currentTask;
	bool benchmark=0;
public:
	ConfiguratorInterface * ci=NULL;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float simulationStep=2*std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
	b2Transform ogGoal;
	Task controlGoal;
	CoordinateContainer data2fp;
	bool planning =1;
	ImgProc imgProc;
	std::vector <vertexDescriptor> planVertices;
	TransitionSystem transitionSystem;
	WorldBuilder worldBuilder;
	vertexDescriptor movingVertex;
	vertexDescriptor currentVertex;
	edgeDescriptor movingEdge, currentEdge;
	std::unordered_map <State*, ExecutionError> errorMap;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug){
	worldBuilder.debug=debug;
	ogGoal=controlGoal.disturbance.pose();
	movingVertex=boost::add_vertex(transitionSystem);
	transitionSystem[movingVertex].Di=controlGoal.disturbance;
	currentVertex=movingVertex;
	currentTask.action.setVelocities(0,0);
	gt::fill(simResult(), &transitionSystem[movingVertex]);
}


bool Spawner(); 

int getIteration(){
	return iteration;
}

void addIteration(int i=1){
	iteration+=i;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}


//inputs: g, src vertex, b2d world, direction of the task to be created
Disturbance getDisturbance(TransitionSystem&, const vertexDescriptor&, b2World &, const Direction &);

Task task_to_execute(const TransitionSystem &, const edgeDescriptor&);

simResult simulate(Task, b2World &, float _simulationStep=BOX2DRANGE);

void trackDisturbance(b2Transform &, Task::Action, float); //open loop

void adjustStepDistance(vertexDescriptor, TransitionSystem &, Task*, float&, std::pair<bool,vertexDescriptor> tgt=std::pair(false,TransitionSystem::null_vertex())); //adjusts the task to be simulated based on how far along execution it is in the real world

std::pair <bool, Direction> getOppositeDirection(Direction);

void applyAffineTrans(const b2Transform& , Task& );

void applyAffineTrans(const b2Transform&, TransitionSystem&);

void applyAffineTrans(const b2Transform&, Disturbance&);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

ExecutionError trackTaskExecution(Task &);


std::vector <vertexDescriptor> changeTask(bool, int&, std::vector <vertexDescriptor>);

int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
	//worldBuilder.simulationStep=f;
}
float approximate_angle(const float &, const Direction &, const simResult::resultType &);

};




 #endif