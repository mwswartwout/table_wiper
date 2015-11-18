#include <coplanar_finder/updated_pcl_utils.h>
#include <limits.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coplanar_finder");  // node name
    ros::NodeHandle nh;
    UpdatedPclUtils cwru_pcl_utils(&nh);
    while (!cwru_pcl_utils.got_kinect_cloud())
    {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    // set up a publisher to display clouds in rviz:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_cloud_display", 1);

    pcl::PointCloud<pcl::PointXYZ> display_cloud;  // instantiate a pointcloud object, used for display in rviz
    sensor_msgs::PointCloud2 pcl2_display_cloud;  // corresponding data type for ROS message

    tf::StampedTransform tf_sensor_frame_to_torso_frame;  // use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener;  // start a transform listener

    // let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr)
    {
        tferr = false;
        try
        {
            // The direction of the transform returned will be from the target_frame to the source_frame.
            // Which if applied to data, will transform data in the source_frame into the target_frame.
            // See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();  // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");  // tf-listener found a complete chain from sensor to world; ready to roll
    // convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    // transform the kinect data to the torso frame;
    // we don't need to have it returned; cwru_pcl_utils can own it as a member var
    cwru_pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);

    while (ros::ok())
    {
        if (cwru_pcl_utils.got_selected_points())
        {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);
            ROS_INFO("Finding coplanar points...");
            cwru_pcl_utils.findCoplanarPoints();
            ROS_INFO("Found coplanar points, now copying to output cloud for display...");
            cwru_pcl_utils.get_gen_purpose_cloud(display_cloud);
            cwru_pcl_utils.reset_got_selected_points();
        }

        pcl::toROSMsg(display_cloud, pcl2_display_cloud);  // convert to compatible ROS message type for publication
        pcl2_display_cloud.header.stamp = ros::Time::now();  // update the time stamp, so rviz does not complain
        pcl2_display_cloud.header.frame_id = "torso";
        pubCloud.publish(pcl2_display_cloud);  // publish a point cloud that can be viewed in rviz

        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    ROS_INFO("my work here is done!");
}
