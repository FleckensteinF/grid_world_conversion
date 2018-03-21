#ifndef _GRID_WORLD_CONVERTER_H_
#define _GRID_WORLD_CONVERTER_H_

//#include <sbpl/utils/utils.h>
#include <Eigen/Core>

class GridConverter{
public:
    GridConverter(int numThetaDirs);

    int numThetaDirs() const;
    double mapOffsetX() const;
    double mapOffsetY() const;
    void mapOffset(double& x, double& y) const;
    double cellSize() const;

    void setNumThetaDirs(const int n);
    void setMapOffset(const double xOffset, const double yOffset);
    void setCellSize(const double cellSize);

    // converts discretized version of angle into continuous (radians)
    // maps 0->0, 1->delta, 2->2*delta, ...
    double discThetaToCont(int nTheta) const;

    // converts continuous (radians) version of angle into discrete
    //maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
    int contThetaToDisc(double fTheta) const;

    // functions to convert from grid indices to world coordinates (cell center)
    void gridToWorld(const int x_d, const int y_d, const int theta_d, double& x_c, double& y_c, double& theta_c) const;
    void grid2dToWorld(const int x_d, const int y_d, double& x_c, double& y_c) const;
    void grid2dToWorld(const Eigen::Vector2i gridCoords, Eigen::Vector2d& worldCoords) const;

    // functions to convert from world coordinates to grid indices
    void worldToGrid(const double x_c, const double y_c, const double theta_c, int& x_d, int& y_d, int& theta_d) const;
    void world2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const;
    Eigen::Vector2i world2dToGrid(const Eigen::Vector2d& xy_c) const;

    // convert from continuous to gridmap coordinates, do not apply offset
    void cont2dToGrid(const double x_c, const double y_c, int& x_d, int& y_d) const;
    void cont2dToGrid(const Eigen::Vector2d xy_c, Eigen::Vector2i& xy_d) const;

    // convert from gridmap to continuous coordinates, do not apply offset
    void grid2dToCont(const int x_d, const int y_d, double& x_c, double& y_c) const;
    void grid2dToCont(const Eigen::Vector2i xy_d, Eigen::Vector2d& xy_c) const;

protected:
    int contXYToDisc(const double x) const;
    double discXYToCont(const int x) const;

    int numThetaDirs_; // the number of discrete theta directions
    double thetaBinSize_;
    double mapOffsetX_, mapOffsetY_; // the grid offset in world coordinates
    double cellSize_; // the size of one cell in world coordinates
};

#endif // _GRID_WORLD_CONVERTER_H_
