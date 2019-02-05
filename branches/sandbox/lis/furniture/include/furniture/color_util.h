#ifndef COLOR_UTIL_H_
#define COLOR_UTIL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace furniture {

  template <class PointT>
  void convertPoint(PointT &point) {
    //Reference: Mr.Mark Ruzon
    /* Color.c */
    // Convert between RGB and CIE-Lab color spaces
    // Uses ITU-R recommendation BT.709 with D65 as reference white.
    // Yossi Rubner
    // Last modified 2/24/98
    //=================================================================== LAB_GEL Function
    
    // From OpenCV
    
    uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
    int r = (int) (rgb >> 16) & 0x0000ff;
    int g = (int) (rgb >> 8)  & 0x0000ff;
    int b = (int) (rgb)       & 0x0000ff;
    
    int l, a, bB;
    
    float x, y, z, fX, fY, fZ;
    
    x = 0.412453 * r + 0.357580 * g + 0.180423 * b;
    y = 0.212671 * r + 0.715160 * g + 0.072169 * b;
    z = 0.019334 * r + 0.119193 * g + 0.950227 * b;
    
    x /= (255 * 0.950456);
    y /=  255;
    z /= (255 * 1.088754);
    
    if (y > 0.008856) {
      fY = pow(y, 1.0/3.0);
      l = (int)(116.0*fY - 16.0 + 0.5);
    }
    else {
      fY = 7.787*y + 16.0/116.0;
      l = (int)(903.3*y + 0.5);
    }
    
    if (x > 0.008856)
      fX = pow(x, 1.0/3.0);
    else
      fX = 7.787*x + 16.0/116.0;
    
    if (z > 0.008856)
      fZ = pow(z, 1.0/3.0);
    else
      fZ = 7.787*z + 16.0/116.0;
    
    a = (int)(500.0*(fX - fY) + 0.5);
    bB = (int)(200.0*(fY - fZ) + 0.5);
    
    // Offset by 110 (original scale is [-110, 110]) and scale to 0-255 to be consistent with RGB
    a += 110;
    bB += 110;

    l = (int) (l * 255.0 / 100.0);
    if (l > 255 || l < 0) {
      ROS_ERROR("L out of range, L=%d", l);
    }
    if (a > 255 || a < 0) {
      ROS_ERROR("A out of range, A=%d", a);
    }
    if (bB > 255 || bB < 0) {
      ROS_ERROR("B out of range, B=%d", bB);
    }
    a = (int) (a * 255.0 / 220.0);
    bB = (int) (bB * 255.0 / 220.0);
    
    uint32_t lab = ((uint32_t)l << 16 | (uint32_t)a << 8 | (uint32_t)bB);
    point.rgb = *reinterpret_cast<float*>(&lab);
  }

  template <class PointT>
  void convert_rgb_to_lab(pcl::PointCloud<PointT> &cloud) {
    for (int i = 0; i < (int) cloud.points.size(); ++i) {
      convertPoint(cloud.points[i]);
    }
  }
  template <class PointT>
  void convert_rgb_to_lab(const pcl::PointCloud<PointT> &input, pcl::PointCloud<PointT> &output) {
    output = input;
    for (int i = 0; i < (int) output.points.size(); ++i) {
      convertPoint(output.points[i]);
    }
  }

  template <class PointT>
  void get_color(PointT p, int &c1, int &c2, int &c3) {
    uint32_t c123 = *reinterpret_cast<int*>(&p.rgb);
    c1 = (int) (c123 >> 16) & 0x0000ff;
    c2 = (int) (c123 >> 8)  & 0x0000ff;
    c3 = (int) (c123)       & 0x0000ff;
  }
  template <class PointT>
  void set_color(PointT &p, uint8_t c1, uint8_t c2, uint8_t c3) {
    uint32_t c123 = ((uint32_t) c1 << 16 | (uint32_t) c2 << 8 | (uint32_t) c3);
    p.rgb = *reinterpret_cast<float*>(&c123);
  }
}

#endif
