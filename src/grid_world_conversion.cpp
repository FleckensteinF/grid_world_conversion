#include "bonirob_traversability_checker/grid_converter.h"

//#include <sbpl/utils/utils.h>
#include <Eigen/Core>

namespace Flourish{
  namespace BoniRob{
    GridConverter::GridConverter(int numThetaDirs):
      numThetaDirs(numThetaDirs),
      mapOffsetX(0),
      mapOffsetY(0),
      cellSize(1)
    {
    
    }

/*    sbpl_xy_theta_pt_t GridConverter::gridToWorld(const int x_d, const int y_d, const int theta_d) const
    {
      sbpl_xy_theta_pt_t pose;
      pose.x = DISCXY2CONT(x_d, cellSize)+mapOffsetX;
      pose.y = DISCXY2CONT(y_d, cellSize)+mapOffsetY;
      pose.theta = DiscTheta2Cont(theta_d, numThetaDirs);
      return pose;
      }*/

    void GridConverter::gridToWorld(const int x_d, const int y_d, const int theta_d, double& x_c, double& y_c, double& theta_c) const
    {
      x_c = DISCXY2CONT(x_d, cellSize)+mapOffsetX;
      y_c = DISCXY2CONT(y_d, cellSize)+mapOffsetY;
      theta_c = DiscTheta2Cont(theta_d, numThetaDirs);
    }

    void GridConverter::grid2dToWorld(const int x_d, const int y_d, double& x_c, double& y_c) const
    {
      x_c = DISCXY2CONT(x_d, cellSize)+mapOffsetX;
      y_c = DISCXY2CONT(y_d, cellSize)+mapOffsetY;
    }

    void GridConverter::grid2dToWorld(const Eigen::Vector2i gridCoords, Eigen::Vector2d& worldCoords) const
    {
      worldCoords.x() = DISCXY2CONT(gridCoords.x(), cellSize)+mapOffsetX;
      worldCoords.y() = DISCXY2CONT(gridCoords.y(), cellSize)+mapOffsetY;
    }

/*    void GridConverter::worldToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const
    {
      x_d = CONTXY2DISC(pose.x - mapOffsetX, cellSize);
      y_d = CONTXY2DISC(pose.y - mapOffsetY, cellSize);
      theta_d = ContTheta2Disc(pose.theta, numThetaDirs);
      theta_d = NORMALIZEDISCTHETA(theta_d, numThetaDirs);
      }*/

    void GridConverter::worldToGrid(const double x_c, const double y_c, const double theta_c, int& x_d, int& y_d, int& theta_d) const
    {
      x_d = CONTXY2DISC(x_c - mapOffsetX, cellSize);
      y_d = CONTXY2DISC(y_c - mapOffsetY, cellSize);
      theta_d = ContTheta2Disc(theta_c, numThetaDirs);
      theta_d = NORMALIZEDISCTHETA(theta_d, numThetaDirs);
    }

    void GridConverter::world2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const
    {
      x_d = CONTXY2DISC(x_c - mapOffsetX, cellSize);
      y_d = CONTXY2DISC(y_c - mapOffsetY, cellSize);
    }

    Eigen::Vector2i GridConverter::world2dToGrid(const Eigen::Vector2d& xy_c) const
    {
      Eigen::Vector2i xy_d;
      xy_d.x() = CONTXY2DISC(xy_c.x() - mapOffsetX, cellSize);
      xy_d.y() = CONTXY2DISC(xy_c.y() - mapOffsetY, cellSize);
      return xy_d;
    }

    void GridConverter::cont2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const
    {
      x_d = CONTXY2DISC(x_c, cellSize);
      y_d = CONTXY2DISC(y_c, cellSize);
    }

    void GridConverter::cont2dToGrid(const Eigen::Vector2d xy_c, Eigen::Vector2i& xy_d) const
    {
      xy_d.x() = CONTXY2DISC(xy_c.x(), cellSize);
      xy_d.y() = CONTXY2DISC(xy_c.y(), cellSize);
    }

  /*void GridConverter::contToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const
    {
      x_d = CONTXY2DISC(pose.x, cellSize);
      y_d = CONTXY2DISC(pose.y, cellSize);
      theta_d = ContTheta2Disc(pose.theta, numThetaDirs);
      }*/

    void GridConverter::grid2dToCont(const int x_d, const int y_d, double& x_c, double& y_c) const
    {
      x_c = DISCXY2CONT(x_d, cellSize);
      y_c = DISCXY2CONT(y_d, cellSize);
    }

    void GridConverter::grid2dToCont(const Eigen::Vector2i xy_d, Eigen::Vector2d& xy_c) const
    {
      xy_c.x() = DISCXY2CONT(xy_d.x(), cellSize);
      xy_c.y() = DISCXY2CONT(xy_d.y(), cellSize);
    }

/*    void GridConverter::gridToCont(const int x_d, const int y_d, const int theta_d, sbpl_xy_theta_pt_t& pose) const
    {
      pose.x = DISCXY2CONT(x_d, cellSize);
      pose.y = DISCXY2CONT(y_d, cellSize);
      pose.theta = DiscTheta2Cont(theta_d, numThetaDirs);
    }
*/
  };
};
