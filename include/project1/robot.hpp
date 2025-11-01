#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

namespace MUSIRobot{

enum Algorithm {
    SimpleAvoidance = 0,
    PotentialFields = 1
};

enum RobotState {
    Normal = 0,
    RotatingToAvoidObstacle = 1,
    AvoidingObstacle = 2
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
};

class Robot {
private:
    int id;
    Params params;
    RobotState currentState;
    Odom currentOdom;
    Algorithm selectedAlgorithm;
    double targetX, targetY;
    double lastAvoidTimestamp;

    // We know that the laser takes measurements from -180 to 180 degrees
    // using 0.1 angles between measurements. Thus, we can specify a parameter
    // to specify how wide our "no obstacles path" is. This will be used
    // in computing the simple avoidance algorithm
    static const int ANGLE_STEPS_OFFSET = 10;

    bool isObjectAhead(const sensor_msgs::LaserScan& laserData);
    double computeAngleDifference();
    double computeRotation(double angleDifference);
    geometry_msgs::Twist runSimpleAvoidance(const sensor_msgs::LaserScan& laserData);
    geometry_msgs::Twist runPotentialFields(const sensor_msgs::LaserScan& laserData);
public:
    Robot();
    void init(int id, Algorithm algorithm, Params params);
    void setOdom(Odom odom);
    void setTarget(double targetX, double targetY);
    geometry_msgs::Twist run(const sensor_msgs::LaserScan& laserData);
};
}
