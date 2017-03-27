#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>  //openni数据流获取头文件
#include <pcl/common/time.h>

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)  
  //回调函数，作为在获取数据时对数据进行处理的回调函数的封装，在本例没做处理，只是实时打印信息
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z 
                << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
      // 注册回调函数，什么意思？？？
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
