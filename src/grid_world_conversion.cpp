#include "grid_world_conversion/grid_world_conversion.h"

#include <angles/angles.h>
#include <iostream>
//#include <sbpl/utils/utils.h>
#include <Eigen/Core>

GridConverter::GridConverter(int numThetaDirs):
    numThetaDirs_(numThetaDirs),
    thetaBinSize_(2.0 * M_PI / numThetaDirs_),
    mapOffsetX_(0),
    mapOffsetY_(0),
    cellSize_(1)
{
    
}

int GridConverter::numThetaDirs() const
{
    return numThetaDirs_;
}

double GridConverter::mapOffsetX() const
{
    return mapOffsetX_;
}

double GridConverter::mapOffsetY() const
{
    return mapOffsetY_;
}

void GridConverter::mapOffset(double& x, double& y) const
{
    x = mapOffsetX_;
    y = mapOffsetY_;
}

double GridConverter::cellSize() const
{
    return cellSize_;
}

void GridConverter::setNumThetaDirs(const int n)
{
    numThetaDirs_ = n;
    thetaBinSize_ = 2.0 * M_PI / numThetaDirs_;
}

void GridConverter::setMapOffset(const double xOffset, const double yOffset)
{
    mapOffsetX_ = xOffset;
    mapOffsetY_ = yOffset;
}

void GridConverter::setCellSize(const double cellSize)
{
    cellSize_ = cellSize;
}

int GridConverter::contXYToDisc(const double x) const
{
    if(x >= 0){
        return x/cellSize_;
    }else{
        return x/cellSize_-1;
    }
}

double GridConverter::discXYToCont(const int x) const
{
    return x*cellSize_ + cellSize_/2.;
}

double GridConverter::discThetaToCont(const int nTheta) const
{
    return nTheta * thetaBinSize_;
}

int GridConverter::contThetaToDisc(const double fTheta) const
{
    return (int)(angles::normalize_angle_positive(fTheta + thetaBinSize_ / 2.0) / (2.0 * M_PI) * (numThetaDirs_));
}


/*    sbpl_xy_theta_pt_t GridConverter::gridToWorld(const int x_d, const int y_d, const int theta_d) const
      {
      sbpl_xy_theta_pt_t pose;
      pose.x = discXYToCont(x_d)+mapOffsetX;
      pose.y = discXYToCont(y_d)+mapOffsetY_;
      pose.theta = discThetaToCont(theta_d);
      return pose;
      }*/

void GridConverter::gridToWorld(const int x_d, const int y_d, const int theta_d, double& x_c, double& y_c, double& theta_c) const
{
    x_c = discXYToCont(x_d)+mapOffsetX_;
    y_c = discXYToCont(y_d)+mapOffsetY_;
    theta_c = discThetaToCont(theta_d);
}

void GridConverter::grid2dToWorld(const int x_d, const int y_d, double& x_c, double& y_c) const
{
    x_c = discXYToCont(x_d)+mapOffsetX_;
    y_c = discXYToCont(y_d)+mapOffsetY_;
}

void GridConverter::grid2dToWorld(const Eigen::Vector2i gridCoords, Eigen::Vector2d& worldCoords) const
{
    worldCoords.x() = discXYToCont(gridCoords.x())+mapOffsetX_;
    worldCoords.y() = discXYToCont(gridCoords.y())+mapOffsetY_;
}

/*    void GridConverter::worldToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const
      {
      x_d = contXYToDisc(pose.x - mapOffsetX_);
      y_d = contXYToDisc(pose.y - mapOffsetY_);
      theta_d = contThetaToDisc(pose.theta);
      theta_d = NORMALIZEDISCTHETA(theta_d, numThetaDirs_);
      }*/

void GridConverter::worldToGrid(const double x_c, const double y_c, const double theta_c, int& x_d, int& y_d, int& theta_d) const
{
    x_d = contXYToDisc(x_c - mapOffsetX_);
    y_d = contXYToDisc(y_c - mapOffsetY_);
    theta_d = contThetaToDisc(theta_c);
    //theta_d = NORMALIZEDISCTHETA(theta_d);
    if(theta_d < 0 || theta_d >= numThetaDirs_){
        std::cerr << "ERROR: got discrete theta smaller than 0 or larger than number of valid theta values!" << std::endl;
    }
}

void GridConverter::world2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const
{
    x_d = contXYToDisc(x_c - mapOffsetX_);
    y_d = contXYToDisc(y_c - mapOffsetY_);
}

Eigen::Vector2i GridConverter::world2dToGrid(const Eigen::Vector2d& xy_c) const
{
    Eigen::Vector2i xy_d;
    xy_d.x() = contXYToDisc(xy_c.x() - mapOffsetX_);
    xy_d.y() = contXYToDisc(xy_c.y() - mapOffsetY_);
    return xy_d;
}

void GridConverter::cont2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const
{
    x_d = contXYToDisc(x_c);
    y_d = contXYToDisc(y_c);
}

void GridConverter::cont2dToGrid(const Eigen::Vector2d xy_c, Eigen::Vector2i& xy_d) const
{
    xy_d.x() = contXYToDisc(xy_c.x());
    xy_d.y() = contXYToDisc(xy_c.y());
}

/*void GridConverter::contToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const
  {
  x_d = contXYToDisc(pose.x);
  y_d = contXYToDisc(pose.y);
  theta_d = contThetaToDisc(pose.theta);
  }*/

void GridConverter::grid2dToCont(const int x_d, const int y_d, double& x_c, double& y_c) const
{
    x_c = discXYToCont(x_d);
    y_c = discXYToCont(y_d);
}

void GridConverter::grid2dToCont(const Eigen::Vector2i xy_d, Eigen::Vector2d& xy_c) const
{
    xy_c.x() = discXYToCont(xy_d.x());
    xy_c.y() = discXYToCont(xy_d.y());
}

/*    void GridConverter::gridToCont(const int x_d, const int y_d, const int theta_d, sbpl_xy_theta_pt_t& pose) const
      {
      pose.x = discXYToCont(x_d);
      pose.y = discXYToCont(y_d);
      pose.theta = discThetaToCont(theta_d);
      }
*/
