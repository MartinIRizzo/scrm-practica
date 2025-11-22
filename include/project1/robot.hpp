#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

namespace MUSIRobot{

enum Algorithm {
    SimpleAvoidance = 0,
    PotentialFields = 1
};

enum Role {
    Leader = 0,
    Follower = 1
};

enum RobotState {
    Normal = 0,
    RotatingToAvoidObstacle = 1,
    AvoidingObstacle = 2,
    ExecutingPotentialFieldsDecision = 3
};

struct Odom {
    double currentX;
    double currentY;
    double currentOrientation;
};

struct Params {
    double criticalDistance;
    double distanceToObjective;
    double permittedOrientationError;
    double kMinimumRotation, kMaximumRotation, vMaximumRotation;
    double vMaximumDisplacement;
    double timeToAvoidObstacle;

    double w1;
    double w2;
    int timeToWait;

    double distLeader;
};

class Robot {
private:
    int id;
    Params params;
    RobotState currentState;
    Role role;
    Odom currentOdom;
    Algorithm selectedAlgorithm;
    double targetX, targetY;
    double lastAvoidTimestamp;
    tf::TransformListener *listener;

    geometry_msgs::Twist lastPotentialFieldsDecision;
    double lastPotentialFieldsDecisionTimestamp;

    // We know that the laser takes measurements from -180 to 180 degrees
    // using 0.1 angles between measurements. Thus, we can specify a parameter
    // to specify how wide our "no obstacles path" is. This will be used
    // in computing the simple avoidance algorithm
    static const int ANGLE_STEPS_OFFSET = 30;

    bool isObjectAhead(const sensor_msgs::LaserScan& laserData);
    double computeAngleDifference();
    double computeRotation(double angleDifference);
    bool reachedObjective();
    tf::Vector3 generateObjectiveVector();
    tf::Vector3 generateObstacleVectors(const sensor_msgs::LaserScan& laserData);
    tf::Vector3 generateObstacleVector(size_t i, double angleIncrement, double angleMin, double reading);
    tf::Vector3 transformToOdomFrame(tf::Vector3 robotVector);
    geometry_msgs::Twist runSimpleAvoidance(const sensor_msgs::LaserScan& laserData);
    geometry_msgs::Twist runPotentialFields(const sensor_msgs::LaserScan& laserData);
public:
    Robot();
    void init(int id, Algorithm algorithm, Role role, Params params, tf::TransformListener *listener);
    void setOdom(Odom odom);
    void setTarget(double targetX, double targetY);
    geometry_msgs::Twist run(const sensor_msgs::LaserScan& laserData);
};
}
