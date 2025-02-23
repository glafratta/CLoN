#ifndef TASK_H
#define TASK_H
#include "measurement.h"
 
b2Fixture * GetSensor(b2Body * body); //gets a sensor from a robot body

b2Body * GetDisturbance(b2World *); //gets disturbance from world

bool overlaps(b2Body *, b2Body *); //does the robot sensor overlap with disturbance? using disturbance body

bool overlaps(b2Body *, Disturbance *);//does the robot sensor overlap with disturbance? using disturbance features



class Task{

public:
    friend class Configurator;
    b2Transform start;
    bool change =0;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    int motorStep=0;
    AffordanceIndex affordance=NONE;

struct Action{
private:
    float linearSpeed=WHEEL_SPEED_DEFAULT*2; //used to calculate instantaneous velocity using omega
    float recordedSpeed=linearSpeed;
    float omega=0; //initial angular velocity is 0
    float recordedOmega = omega;
    bool valid=0;
public:
    float R=WHEEL_SPEED_DEFAULT;
    float L=WHEEL_SPEED_DEFAULT;

    Action()=default;

    void init(Direction& direction){
        switch (direction){
        case Direction::DEFAULT:
        L=WHEEL_SPEED_DEFAULT;
        R=WHEEL_SPEED_DEFAULT;
        break;
        case Direction::LEFT:
        L=-WHEEL_SPEED_TURN;
        R=WHEEL_SPEED_TURN;
        break;
        case Direction::RIGHT:
        L=WHEEL_SPEED_TURN;//0.2537;
        R=-WHEEL_SPEED_TURN;
        break;
        default:
        L=0;
        R=0;
        break;
    }
    setVelocities(L, R);
    }

void setVelocities(float l, float r){
    omega = (MAX_SPEED*(r-l)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
    recordedOmega = omega;
    linearSpeed = MAX_SPEED*(l+r)/2;
    recordedSpeed=linearSpeed;
    valid=1;
}

    b2Vec2 getLinearVelocity(){
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega);
        velocity.y = linearSpeed *sin(omega);
        return velocity;
    }

    b2Transform getTransform(){
    return b2Transform(getLinearVelocity(), b2Rot(omega));
}

    float getRWheelSpeed(){
        return R;
    }

    float getLWheelSpeed(){
    return L;
    }


    bool isValid(){
        return valid;
    }

    float getLinearSpeed(){
        return linearSpeed;
    }

    float getOmega(){
    return omega;
    }

    float getOmega(float l, float r){
        float result = (MAX_SPEED*(r-l)/BETWEEN_WHEELS)*TURN_FRICTION;
        return result;
    }

    void setOmega(float o){
        omega =o;
    }

    void setLinearSpeed(float s){
        linearSpeed =s;
    }

    void setRecSpeed(float s){
        recordedSpeed =s;
    }

    void setRecOmega(float w){
        recordedOmega=w;
    }

    float getRecSpeed(){
        return recordedSpeed;
    }

    float getRecOmega(){
        return recordedOmega;
    }
    //friend class Configurator;
    void setRec(float _speed, float _omega){
        recordedSpeed=_speed;
        recordedOmega=_omega;
    }

};

Task(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
    printf("default constructro\n");
}

Task(Direction d){
    direction=d;
    action.init(direction);
}

Task(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), bool topDown=0){
    start = _start;
    disturbance = ob;
    affordance=disturbance.getAffIndex();
    direction = H(disturbance, d, topDown);  
    action.init(direction);
    setEndCriteria();

}


simResult willCollide(b2World &, int, b2Body *,bool debug =0, float remaining = SIM_DURATION, float simulationStep=BOX2DRANGE); //gets simulation data: new disturbances? details of motor behaviour...

EndCriteria getEndCriteria(const Disturbance&);


class Listener : public b2ContactListener { //call-back based, returns a collision when it occurs in the simulation
 // int iteration=1;
    Disturbance * d_ptr;
    public:
    Listener(){}
    Listener(Disturbance * _d): d_ptr(_d){}
    std::vector <b2Body*> collisions;
        void BeginContact(b2Contact * contact) {
        b2Fixture * fixtureA= contact->GetFixtureA();
        b2Fixture * fixtureB= contact->GetFixtureB();
        b2BodyUserData bodyData = fixtureA->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) { //if fixtureA belongs to robot
            b2Body * other = contact->GetFixtureB()->GetBody();
            if (fixtureA->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureB->IsSensor()){
                return;
            }
            collisions.push_back(other);
        }
        bodyData = fixtureB->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) {//WAS IF BODYDATA.POINTER if fixtureB belongs to robot
            b2Body * other = contact->GetFixtureA()->GetBody();
            if (fixtureB->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureA->IsSensor()){
                return;
            }
            collisions.push_back(other);
            }       
        }

        
	};
	
    class Query : public b2QueryCallback {
		Disturbance * d_ptr=NULL;
        public:
            Query(Disturbance * _d_ptr):d_ptr(_d_ptr){}
            std::vector<b2Body*> d;
            
            bool ReportFixture(b2Fixture* fixture) {
                if (fixture->GetBody()->GetUserData().pointer==DISTURBANCE_FLAG){
                    if (d_ptr==NULL){
                        return false;
                    }
                    d_ptr->invalidate();
                    return true;//keep going to find all fixtures in the query area
                }
                return false;
            }
    };


struct Correct{ //not used yet, outline of trajectory control
    
    Correct(){}

    void operator()( Action&, int);

    float errorCalc(Action, double);

    float getError(){
        return p();
    }

    float Ki(){
        return ki;
    }

    float Kp(){
        return kp;
    }
    float Kd(){
        return kd;
    }

    float get_i(){
        return i;
    }

    float get_d(){
        return d;
    }

    float update(float);

    void reset(){
        p_buffer=std::vector <float>(bufferSize,0);
        i=0;
        d=0;
        mf.buffer=std::vector<float>(mf.kernelSize,0);
    }

    float kp=0.075;    
    float kd=0, ki=0;
    private:


    float p(){
        float sum=0;
        for (int j=0;j<p_buffer.size(); j++){
            sum+=p_buffer[j];
        }
        return sum;
    }
    int correction_rate=2; //Hz
    int bufferSize= correction_rate*(FPS/MOTOR_CALLBACK);
    std::vector <float>p_buffer=std::vector <float>(bufferSize,0);
    float i=0, d=0;
    float tolerance_upper=0.01, tolerance_lower=-0.01;

    struct MedianFilter{
        int kernelSize=3;
        std::vector<float>buffer=std::vector<float>(kernelSize,0);

        float get_median(){
            std::vector <float> tmp=buffer;
            std::sort(tmp.begin(), tmp.end());
            return tmp[int(kernelSize/2)];
        }
    }mf;
    

}correct;

public:
friend Task::Correct;    
Action action;

Disturbance disturbance;

Task::Action getAction(){
    return action;
}

AffordanceIndex getAffIndex(){
    return affordance;
}

//generate motor output (Agent transfer function)
Direction H(Disturbance, Direction, bool topDown=0); //topDown enables Configurator topdown control on reactive behaviour


void setEndCriteria(Angle angle=SAFE_ANGLE, Distance distance=BOX2DRANGE);

void setEndCriteria(const Distance& distance);

//check if task has ended from the robot's position 
EndedResult checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), Direction dir=UNDEFINED, bool relax=0, b2Body* robot=NULL, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))));

//check if task has ended from a state's end position 
EndedResult checkEnded(State, Direction dir=UNDEFINED, bool relax=false, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)))); //usually used to check against control goal

};

#endif