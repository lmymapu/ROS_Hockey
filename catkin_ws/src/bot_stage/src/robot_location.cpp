#include<ros/ros.h>
#include<tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_location");
    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Rate rate(2.0);
    listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));       //The translation origin points from odom origin to base_footprint origin
    tf::Vector3 pt_odom, pt_base;
    pt_base.setValue(1,2,0);
    while(ros::ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);          //target: odom, source: base_footprint.

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            tf::Quaternion q = transform.getRotation();
            //pt_base = transform.inverse() * pt_odom;
            pt_odom = transform * pt_base;
            cout << "Current position & translation: ("<<x<<" , "<<y<<")"<<endl;
            cout << "Current rotation axis: ("<<q.getAxis().x()<<" , "<<q.getAxis().y()<<" , "<<q.getAxis().z()<<" , theta: "<<q.getAngle()<<endl;
            cout << "trafo from base to odom: "<< pt_odom.getX() <<" , " << pt_odom.getY()<<" , "<< pt_odom.getZ()<<endl<<endl;
        }catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
        }
        rate.sleep();
    }
    return 0;
}
