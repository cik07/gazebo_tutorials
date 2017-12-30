/**
 * Makes all the non-planar things green, publishes the final result on the /obstacles topic.
 */
#include <iostream>
#include <ros/ros.h>

#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


ros::Publisher pub;
ros::Publisher type_pub;
int function(float, float, float, float);
float treshold(float);

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (0.1);
    seg.setDistanceThreshold (1.0);
    seg.setInputCloud (cloud_filtered);

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    pcl::IndicesPtr remaining (new std::vector<int>);
    remaining->resize (nr_points);

    for (size_t i = 0; i < remaining->size (); ++i) {
        (*remaining)[i] = static_cast<int>(i);
    }


    float duzlem[10][4]; // Segmente edilen yuzeylerin duzlem denklemi katsayilarini tutan dizi; birinci indis duzlemi, ikinci indis o duzlemin katsayilarini tutar
    int count=0;
    std_msgs::Int8 tip; // Engel tipi

    // While 30% of the original cloud is still there
    while (remaining->size () > 0.1 * nr_points) {

        // Segment the largest planar component from the remaining cloud
        seg.setIndices (remaining);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) break;

        //std::cout << i << ". yüzeyin sahip oldugu nokta sayisi: " << inliers->indices.at(0) << " " << inliers->indices.at(1) << " " << inliers->indices.at(2) << std::endl;

        // Duzlemlerin verileri duzlem matrisine alinir.
        duzlem[count][0]=coefficients->values[0];
        duzlem[count][1]=coefficients->values[1];
        duzlem[count][2]=coefficients->values[2];
        duzlem[count][3]=coefficients->values[3];


        tip.data = function(duzlem[count][0], duzlem[count][1], duzlem[count][2], duzlem[count][3]);

        if (tip.data==4) {
            if ( treshold( (duzlem[count][0]*cloud_filtered->points[inliers->indices[0]].x + duzlem[count][1]*cloud_filtered->points[inliers->indices[0]].y + duzlem[count][2]*cloud_filtered->points[inliers->indices[0]].z + duzlem[count][3]) ) != 0  )
                tip.data=5;
        }

        switch (tip.data)  {
            case 0:
                std::cout << "Zemin" << std::endl;
                break;
            case 1:
                std::cout << "Duvar" << std::endl;
                break;
            case 2:
                std::cout << "Cukur" << std::endl;
                break;
            case 3:
                std::cout << "Kaldirim" << std::endl;
                break;
            case 4:
                std::cout << "Rampa" << std::endl;
                break;
            case 5:
                std::cout << "Merdiven" << std::endl;
                break;
            case 6:
            default:
                std::cout << "Tanınmayan ortam" << std::endl;
        }

        count++;

        std::cout << coefficients->values[0]<< std::endl;
        std::cout << coefficients->values[1]<< std::endl;
        std::cout << coefficients->values[2]<< std::endl;
        std::cout << coefficients->values[3]<< std::endl;

        std::cout << cloud_filtered->points[inliers->indices[0]].x << " ----------------------------- point x değeri" << std::endl;
        std::cout << inliers->indices.data()[0] <<" ----------------------------- indices.data değeri" << std::endl;


        /*
        for( int a = 0; a < 12; a = a + 1 ) {
            std::cout << coefficients->values[a]<< std::endl;
        }

        std::cout << coefficients->values.at(0)<< std::endl;
        std::cout << coefficients->values[0]<< std::endl;

        */


        // Extract the inliers
        std::vector<int>::iterator it = remaining->begin();
        for (size_t i = 0; i < inliers->indices.size (); ++i) {
        int curr = inliers->indices[i];

        // Remove it from further consideration.
            while (it != remaining->end() && *it < curr) {
                ++it;
            }
            if (it == remaining->end()) break;
            if (*it == curr) it = remaining->erase(it);
        }
        i++;
    }

    std::cout << "Found " << i << " planes." << std::endl;

    // Engel tipi publish edilir.
    type_pub.publish(tip);


    // Color all the non-planar things.
    for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it) {
        uint8_t r = 0, g = 255, b = 0;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
    }

    // Publish the planes we found.
    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
    pub.publish (outcloud);

}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "obstacles");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);

    // Tip publisherı oluşturulur.
    type_pub = nh.advertise<std_msgs::Int8>("set_type",1);

    // Spin
    ros::spin ();
}



/*
 * Return değerleri:
 *
 * 0 - Zemin
 * 1 - Duvar
 * 2 - Çukur
 * 3 - Kaldırım
 * 4 - Rampa veya Merdiven
 *
 */

int function (float a, float b, float c, float d) {

    d -= 0.3;
    a = treshold(a);
    b = treshold(b);
    c = treshold(c);
    d = treshold(d);

    if (a == 0 && c == 0 && b != 0) {
        if (d == 0) return 0;
        if (d > 0) return 2;
        if (d < 0) return 3;
    }
    else if (b == 0) {
        if (a != 0 || c != 0) {
            return 1;
        }
    }
    return 4;
}

float treshold (float x) {
    if ( fabs(x) < 0.05 ) {
        return 0;
    }
    else return x;
}
