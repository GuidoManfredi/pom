#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "../xp/Gripper.h"
#include "../xp/Listener.h"

using namespace std;

void relative_error(tf::Quaternion qtrue, tf::Vector3 ttrue,
        			  tf::Quaternion qest,  tf::Vector3 test,
        			  double & rot_err, double & transl_err);

void save(string object_path, double rotation_error, double translation_error);

// 1) Mettre bras dans une position
// 2) Regarder object avec camera (pas forcement object au centre de l'image)
// 3) Lancer detection avec nom d'objet
// 4) Verifier que objet bien vu
// 5) Lancer ce programme avec le meme nom d'objet
// 6) Recommencer

// rosrun pom experiment narrow_stereo_l_stereo_camera_optical_frame lait 
// rosrun image_view image_view image:=/narrow_stereo/left/image_rect
int main (int argc, char** argv) {
    assert(argc == 4 && "Usage: experiment camera object_name offset (meters)");
    ros::init(argc, argv, "pom_experiment");

    Listener listener;
    //Gripper gripper;
    //string gripper_frame = "r_gripper_tool_frame";
    string gripper_frame = "r_gripper_led_frame";
    //string gripper_frame = "r_gripper_motor_slider_link";

    //gripper.open(); // be sure that the gripper is open
    //gripper.grab(); // wait for human to give object
    ROS_INFO("Waiting for object...");
    
    ros::Duration d(1.0);
    d.sleep(); // sleep for tf synching
    double x_offset = atof(argv[3]);
    //ROS_INFO("Enter offset along x, in meters:");
    //cin >> x_offset; // wait for user to press "enter" before computing error
    
    ROS_INFO("Listening for transforms...");
    tf::StampedTransform estimate = listener.listen(argv[1], argv[2]);
    tf::StampedTransform ground_truth = listener.listen(argv[1], gripper_frame);
    // offset to measured object center
    tf::Transform to_object;
    to_object.setOrigin( tf::Vector3(x_offset, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    to_object.setRotation(q);
    ground_truth.mult(ground_truth, to_object);
    
    ROS_INFO("Computing errors...");
    tf::Quaternion qtrue = ground_truth.getRotation();
    tf::Vector3 ttrue = ground_truth.getOrigin();
    tf::Quaternion qest = estimate.getRotation();
    tf::Vector3 test = estimate.getOrigin();
    double rot_error, trans_error;
    relative_error(qtrue, ttrue, qest, test, rot_error, trans_error);
    ROS_INFO("Errors: %lf %lf", rot_error, trans_error);
 
    ROS_INFO("Saving errors");
    string save_path = string(argv[2]) + ".txt";
    save(save_path, rot_error, trans_error);
    
    //gripper.give(); // give back object
    ROS_INFO("Waiting for human to take object...");
    
    ROS_INFO("Broadcasting ground truth");
    tf::TransformBroadcaster br;
    ros::Rate loop_rate(10);
    while(ros::ok())
        br.sendTransform(tf::StampedTransform(ground_truth, ros::Time::now(), argv[1], "ground_truth"));
        loop_rate.sleep();
    return 0;
}

void save(string object_path, double rotation_error, double translation_error) {
    ofstream file;
    file.open(object_path.c_str(), ios::app);
    file << rotation_error << ";" << translation_error << endl;
    file.close();
}

void relative_error(tf::Quaternion qtrue, tf::Vector3 ttrue,
        			  tf::Quaternion qest,  tf::Vector3 test,
        			  double & rot_err, double & transl_err) {
    // x_true, x_est...
    double xt = qtrue.x(),    yt = qtrue.y(),    zt = qtrue.z(),    wt = qtrue.w();
    double xe = qest.x(),    ye = qest.y(),    ze = qest.z(),    we = qest.w();
    double rot_err1 = sqrt((xt - xe) * (xt - xe)
		             + (yt - ye) * (yt - ye)
		             + (zt - ze) * (zt - ze)
		             + (wt - we) * (wt - we) )
                     / sqrt(xt * xt + yt * yt + zt * zt + wt * wt);

    double rot_err2 = sqrt((xt + xe) * (xt + xe)
		             + (yt + ye) * (yt + ye)
		             + (zt + ze) * (zt + ze)
		             + (wt + we) * (wt + we) )
                     / sqrt(xt * xt + yt * yt + zt * zt + wt * wt);

    rot_err = min(rot_err1, rot_err2);

    xt = ttrue.x(),    yt = ttrue.y(),    zt = ttrue.z();
    xe = test.x(),    ye = test.y(),    ze = test.z();
    transl_err = sqrt((xt - xe) * (xt - xe)
                 + (yt - ye) * (yt - ye)
                 + (zt - ze) * (zt - ze))
                 / sqrt(xt * xt + yt * yt + zt * zt);
}

