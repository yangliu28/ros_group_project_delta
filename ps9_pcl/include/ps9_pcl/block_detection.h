#include <cwru_pcl_utils/cwru_pcl_utils.h>

using namespace std;

///let's use RGB first.
Eigen::Vector3f std_red;
Eigen::Vector3f std_yellow;
Eigen::Vector3f std_blue;
Eigen::Vector3f std_green;
Eigen::Vector3f std_black;

///determin the stand color.
std_red = (255, 255, 255);
std_yellow = (255, 255, 255);
std_blue = (255, 255, 255);
std_green = (255, 255, 255);
std_black = (255, 255, 255);

int find_color();
int block_detection();
