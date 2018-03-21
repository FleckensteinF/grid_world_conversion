#ifndef _GRID_WORLD_CONVERTER_H_
#define _GRID_WORLD_CONVERTER_H_

#include <sbpl/utils/utils.h>
#include <Eigen/Core>

class GridConverter{
public:
    GridConverter(int numThetaDirs);
    
    // functions to convert from grid indices to world coordinates (cell center)
//      sbpl_xy_theta_pt_t gridToWorld(const int x, const int y, const int theta) const;
    void gridToWorld(const int x_d, const int y_d, const int theta_d, double& x_c, double& y_c, double& theta_c) const;
    void grid2dToWorld(const int x_d, const int y_d, double& x_c, double& y_c) const;
    void grid2dToWorld(const Eigen::Vector2i gridCoords, Eigen::Vector2d& worldCoords) const;
    // functions to convert from world coordinates to grid indices
    //void worldToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const;
    void worldToGrid(const double x_c, const double y_c, const double theta_c, int& x_d, int& y_d, int& theta_d) const;
    void world2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const;
    Eigen::Vector2i world2dToGrid(const Eigen::Vector2d& xy_c) const;
    // convert from continuous to gridmap coordinates, do not apply offset
    void cont2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const;
    void cont2dToGrid(const Eigen::Vector2d xy_c, Eigen::Vector2i& xy_d) const;
    //void contToGrid(const sbpl_xy_theta_pt_t& pose, int& x_d, int& y_d, int& theta_d) const;
    // convert from gridmap to continuous coordinates, do not apply offset
    void grid2dToCont(const int x_d, const int y_d, double& x_c, double& y_c) const;
    void grid2dToCont(const Eigen::Vector2i xy_d, Eigen::Vector2d& xy_c) const;
    //void gridToCont(const int x_d, const int y_d, const int theta_d, sbpl_xy_theta_pt_t& pose) const;

    int numThetaDirs; // the number of discrete theta directions
    double mapOffsetX, mapOffsetY; // the grid offset in world coordinates
    double cellSize; // the size of one cell in world coordinates
};

#endif // _GRID_WORLD_CONVERTER_H_
