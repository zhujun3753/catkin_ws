// isiolate one support.
//              -- created by dongy. Modified: 20210116


#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;

using namespace std;

pcl::PointCloud<PointT> g_keyPoints;
bool g_filterBeginFlag = false;

double x_min = 999, x_max = -999;
double y_min = 999, y_max = -999;
double z_min = 999, z_max = -999;

struct callback_args{
    // structure used to pass arguments to the callback function
    pcl::PointCloud<PointT>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void mouseCallback(const pcl::visualization::PointPickingEvent& event, void *args){
    // cout << "mouse callback" << endl;

    struct callback_args * data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;

    PointT pt;
    event.getPoint(pt.x, pt.y, pt.z);
    data->clicked_points_3d->points.push_back(pt);
    
    //Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << "Points: [" << pt.x << " " << pt.y << " " << pt.z << "]" << std::endl;

    g_keyPoints.push_back(pt);          // save keypoints. 
}

void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void *args){
    if(event.keyDown()){
        if(event.getKeyCode()=='a'){
            // cout << "Show selected points: " << endl;
            for(int i=0; i<g_keyPoints.size(); ++i){
                PointT p = g_keyPoints[i];
                x_min = p.x < x_min ? p.x : x_min;
                y_min = p.y < y_min ? p.y : y_min;
                z_min = p.z < z_min ? p.z : z_min;
                x_max = p.x > x_max ? p.x : x_max;
                y_max = p.y > y_max ? p.y : y_max;
                z_max = p.z > z_max ? p.z : z_max;
                // cout << p.x << ", " << p.y << ", " << p.z << endl;
            }
            cout << "------------- Range ------------- " << endl;
            cout << "x: [" << x_min << ", " << x_max << "]" << endl;
            cout << "y: [" << y_min << ", " << y_max << "]" << endl;
            cout << "z: [" << z_min << ", " << z_max << "]" << endl;
        }
        if(event.getKeyCode()=='b'){
            if(x_min>x_max)
                cout << "[Error]. Must select points before isolate." << endl;
            cout << "Isolate support" << endl;
            g_filterBeginFlag = true;
        }
    }
}


// isolate suppoprt by passthrough filter. 
bool isolateSupport(pcl::PointCloud<PointT>::Ptr cloud, bool done){
    if(done)
        return true;
    cout << "Save isoalted pointcloud." << endl;
    pcl::PassThrough<pcl::PointXYZ> pass;
    double margin = 0.1;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min - margin, x_max + margin);
    pass.filter(*cloud);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min - margin, y_max + margin);
    pass.filter(*cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min - margin, z_max + margin);
    pass.filter(*cloud);
    pcl::io::savePCDFileASCII("isolated_support.pcd", *cloud);
    cout << "File saved. Size: " << cloud->size() << endl;
    return true;
}


int main(int argc, char **argv){

    cout << "Isolate one support." << endl;
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());

    if(pcl::io::loadPCDFile<PointT>("ver.pcd", *cloud_in) == -1){
        cout << "Cannot open 'ver.pcd'. " << endl;
    }
    
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "full_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "full_cloud");

    // interaction parts.
    viewer->addCoordinateSystem(2.0, "axis"); // show axes;

    // mouse interaction
    struct callback_args cb_args;
    pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);        // an empty pointCloud. Not used.
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(mouseCallback, (void*)&cb_args);
    
    // keyboard callback
    viewer->registerKeyboardCallback(keyboardCallback, NULL);

    bool isIsolated = false;        // flag, only isolate once.
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        if(g_filterBeginFlag)
            isIsolated = isolateSupport(cloud_in, isIsolated);
    }
    return (0);
}

