#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <igvc_bot/LanesConfig.h>

#include <utility>
#include <vector>


// Objects that the callback needs. Initialized in main().
// TODO: add a mutex
struct Helpers {
    Helpers(ros::Publisher pc_publisher, image_transport::Publisher image_masked_publisher) {
        pub_pc = pc_publisher;
        pub_masked = image_masked_publisher;
    }

    ros::Publisher pub_pc;
    image_transport::Publisher pub_masked;

    cv::Scalar white_lower{0, 0, 0}, white_upper{180, 40, 255}; // HSV range for color white.

    double rect_frac{0.6}; // Vertical fraction of image masked

    uint8_t erosion_size{2}, erosion_iter{1};
    cv::Mat erosion_element{cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(2 * 3 + 1, 2 * 3 + 1))};

    uint8_t blur_size{7};

    bool publish_masked{false};

    // For projecting the image onto the ground.
    image_geometry::PinholeCameraModel cameraModel;
    tf::TransformListener listener;
};


const char *topic_image = "image_rect_color";
const char *topic_camera_info = "camera_info";
const char *topic_masked = "image_masked";
const char *topic_pointcloud2 = "points2";

// In rviz map/odom seems to be a true horizontal plane located at ground level.
const char *ground_frame = "odom";


void callback(const sensor_msgs::ImageConstPtr &msg_left,
              Helpers &helper) {

    try {

        tf::StampedTransform transform;
        helper.listener.lookupTransform(ground_frame, helper.cameraModel.tfFrame(), ros::Time(0), transform);

        cv::Mat hsv, blur, raw_mask, eroded_mask, final_mask, final_mask2, final_mask1, masked, bgr, grey, eroded_mask1;

        cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(msg_left, "bgr8");


        cv::cvtColor(cv_img->image, hsv, cv::COLOR_BGR2HSV);
        cv::GaussianBlur(hsv, blur, cv::Size(helper.blur_size, helper.blur_size), 0, 0);

        // Get white pixels
        cv::inRange(blur, helper.white_lower, helper.white_upper, raw_mask);


        // Flood Fill from the top of the mask to remove the sky in gazebo.
        cv::floodFill(raw_mask, cv::Point(raw_mask.cols / 2, 2), cv::Scalar(0));

        // Errors in projection increase as we approach the halfway point of the image:
        // Apply a mask to remove top 60%
        raw_mask(cv::Rect(0, 0, raw_mask.cols, (int) (raw_mask.rows * helper.rect_frac))) = 0;

        cv::erode(raw_mask, eroded_mask, helper.erosion_element, cv::Point(-1, -1), helper.erosion_iter);

        cv::erode(eroded_mask, eroded_mask1, helper.erosion_element, cv::Point(-1, -1), helper.erosion_iter);

        cv::medianBlur(eroded_mask1, final_mask1, 3);
        cv::medianBlur(final_mask1, final_mask, 5);

        cv_img->image.copyTo(masked, final_mask);


        if (helper.publish_masked) {
            helper.pub_masked.publish(cv_bridge::CvImage(cv_img->header, cv_img->encoding, masked).toImageMsg());
        }

        std::vector<cv::Point> points; // All the points which is detected as part of the lane.
        cv::findNonZero(final_mask, points);


        // Initialize the point cloud:
        sensor_msgs::PointCloud2Ptr point_cloud = boost::make_shared<sensor_msgs::PointCloud2>();
        point_cloud->header.frame_id = ground_frame;
        point_cloud->header.stamp = ros::Time::now();
        point_cloud->height = 1;
        point_cloud->width = points.size();
        point_cloud->is_bigendian = false;
        point_cloud->is_dense = false;

        sensor_msgs::PointCloud2Modifier pc_mod(*point_cloud);
        pc_mod.setPointCloud2FieldsByString(1, "xyz");


        // Change the transform to a more useful form.
        const tf::Quaternion trans_rot = transform.getRotation();
        const cv::Vec3d trans_vec{trans_rot.x(), trans_rot.y(), trans_rot.z()};
        const double trans_sca = trans_rot.w();

        sensor_msgs::PointCloud2Iterator<float> x(*point_cloud, "x"), y(*point_cloud, "y"), z(*point_cloud, "z");

        for (const auto &point : points) {
            // ___________ Ray is a vector that points from the camera to the pixel: __________
            // Its calculation is pretty simple but is easier to use the image_geometry package.
            /* Basically:
             * cv::Point3d ray_cameraModel_frame;
             * ray_cameraModel_frame.x = (uv_rect.x - cx() - Tx()) / fx();
             * ray_cameraModel_frame.y = (uv_rect.y - cy() - Ty()) / fy();
             * ray_cameraModel_frame.z = 1.0;
             * f is focal length
             */
            cv::Point3d ray_cameraModel_frame = helper.cameraModel.projectPixelTo3dRay(point);

            /* Note: the ray_cameraModel_frame is not in the same frame as the tf_frame: camera_left
             * Ray frame:
             * x points to the right of the image, y points down, z points inward
             *
             * camera_left frame:
             * x points in the image and y points up
             *
             * If you look at the camera from above:
             *
             * ray_cameraModel_frame: x <---(x) y  camera_left: z (.)---> y
             *             |                     |
             *             |                     |
             *             z                     x
             *
             * What we do is basically scale the ray_cameraModel_frame such that the end touches the ground (we know the lane points are actually on the ground.
             * Then we add those co-ords to the point cloud.
             * ___________ We are basically checking the coords where the ray_cameraModel_frame intersects the ground ________
             * This can later be extended for non planar surfaces by checking where the ray intersects a lidar generated 3d mesh
             */
            cv::Vec3d ray{ray_cameraModel_frame.z, -ray_cameraModel_frame.x, -ray_cameraModel_frame.y};

            // Rotate ray by using the transform.
            // Kinda black mathematics on v_p = q * v * q'
            // https://gamedev.stackexchange.com/a/50545/90578
            cv::Vec3d ray_p = 2.0 * trans_vec.dot(ray) * trans_vec
                              + (trans_sca * trans_sca - trans_vec.dot(trans_vec)) * ray
                              + 2.0f * trans_sca * trans_vec.cross(ray);

            if (ray_p[2] == 0) // Should never happen
                continue;

            // Scale factor for ray so it touches the ground
            const double scale = transform.getOrigin().z() / ray_p[2];

            // Add ray to camera_left's origin
            *x = transform.getOrigin().x() - ray_p[0] * scale;
            *y = transform.getOrigin().y() - ray_p[1] * scale;
            *z = 0; // transform.getOrigin().z() - ray_p[2] * scale;

            // Pointcloud iterators
            ++x;
            ++y;
            ++z;
        }
        helper.pub_pc.publish(point_cloud);

    } catch (std::exception &e) { // Not a good idea....
        ROS_ERROR("Callback failed: %s", e.what());
    }
}

// This allows us to change params of the node while it is running: uses cfg/lanes.cfg.
// Try running `rosrun rqt_reconfigure rqt_reconfigure` while node is running.
// This also auto loads any params initially set in the param server.
void dynamic_reconfigure_callback(const igvc_bot::LanesConfig &config, const uint32_t &level, Helpers &helpers) {
    ROS_INFO("Reconfiguring the params.");

    if (level & 1u << 0u) {
        helpers.white_lower = cv::Scalar(config.h_lower, config.s_lower, config.v_lower);
    }

    if (level & 1u << 1u) {
        helpers.white_upper = cv::Scalar(config.h_upper, config.s_upper, config.v_upper);
    }

    if (level & 1u << 2u) {
        helpers.erosion_size = config.erosion_size;
        helpers.erosion_iter = config.erosion_iter;
        helpers.erosion_element = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE,
                                                            cv::Size(2 * helpers.erosion_size + 1,
                                                                     2 * helpers.erosion_size + 1));
    }

    if (level & 1u << 3u) {
        helpers.publish_masked = config.publish_masked;
    }

    if (level & 1u << 4u) {
        helpers.blur_size = 2 * config.blur_size + 1;
    }

    if (level & 1u << 5u) {
        helpers.rect_frac = config.upper_mask_percent / 100.0;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lane");
    ros::NodeHandle nh;

    ROS_INFO("Started");

    // For receiving and publishing the images in an easy way.
    image_transport::ImageTransport imageTransport(nh);

    Helpers helper{
            nh.advertise<sensor_msgs::PointCloud2>(topic_pointcloud2, 5),
            imageTransport.advertise(topic_masked, 1),
    };


    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            topic_camera_info);
    helper.cameraModel.fromCameraInfo(camera_info);


    dynamic_reconfigure::Server<igvc_bot::LanesConfig> server;
    dynamic_reconfigure::Server<igvc_bot::LanesConfig>::CallbackType dynamic_reconfigure_callback_function = boost::bind(
            &dynamic_reconfigure_callback, _1, _2, boost::ref(helper));
    server.setCallback(dynamic_reconfigure_callback_function);


    image_transport::Subscriber sub = imageTransport.subscribe(topic_image, 2,
                                                               boost::bind(&callback, _1, boost::ref(helper)));


    ROS_INFO("Spinning...");
    ros::spin();
}
