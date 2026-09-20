#include "maps.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

namespace
{
template <typename T>
void loadParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

void
Maps::randomMapGenerate()
{

  std::default_random_engine eng(info.seed);

  double _resolution = 1 / info.scale;

  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  double _h_l = 0;
  double _h_h = info.sizeZ / info.scale;

  double _w_l, _w_h;
  int    _ObsNum;

  loadParameter(info.node, "width_min", _w_l, 0.6);
  loadParameter(info.node, "width_max", _w_h, 1.5);
  loadParameter(info.node, "height_min", _h_l, _h_l);
  loadParameter(info.node, "height_max", _h_h, _h_h);
  loadParameter(info.node, "obstacle_number", _ObsNum, 10);
  double surface_resolution;
  loadParameter(info.node, "surface_resolution", surface_resolution, _resolution * 0.5);

  _h_l = std::max(0.0, _h_l);
  _h_h = std::min(info.sizeZ / info.scale, std::max(_h_l, _h_h));
  surface_resolution = std::max(1e-3, surface_resolution);

  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;

  rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

  for (int i = 0; i < _ObsNum; i++)
  {
    double x, y;
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    const double half_w = w * 0.5;
    const double x_min = x - half_w;
    const double x_max = x + half_w;
    const double y_min = y - half_w;
    const double y_max = y + half_w;
    const int xy_steps = std::max(1, static_cast<int>(std::ceil(w / surface_resolution)));
    const int z_steps = std::max(1, static_cast<int>(std::ceil(h / surface_resolution)));

    auto pushPoint = [&](double px, double py, double pz) {
      pt_random.x = px;
      pt_random.y = py;
      pt_random.z = pz;
      info.cloud->points.push_back(pt_random);
    };

    for (int iz = 0; iz <= z_steps; ++iz)
    {
      const double z = h * iz / z_steps;
      for (int ix = 0; ix <= xy_steps; ++ix)
      {
        const double px = x_min + w * ix / xy_steps;
        pushPoint(px, y_min, z);
        pushPoint(px, y_max, z);
      }
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double py = y_min + w * iy / xy_steps;
        pushPoint(x_min, py, z);
        pushPoint(x_max, py, z);
      }
    }

    for (int ix = 0; ix <= xy_steps; ++ix)
      for (int iy = 0; iy <= xy_steps; ++iy)
      {
        const double px = x_min + w * ix / xy_steps;
        const double py = y_min + w * iy / xy_steps;
        pushPoint(px, py, 0.0);
        pushPoint(px, py, h);
      }
  }

  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;

  pcl2ros();
}

void
Maps::pcl2ros()
{
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
  RCLCPP_INFO(info.node->get_logger(), "finish: infill %lf%%",
              info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

void
Maps::perlin3D()
{
  double complexity;
  double fill;
  int    fractal;
  double attenuation;

  loadParameter(info.node, "complexity", complexity, 0.142857);
  loadParameter(info.node, "fill", fill, 0.38);
  loadParameter(info.node, "fractal", fractal, 1);
  loadParameter(info.node, "attenuation", attenuation, 0.5);

  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  PerlinNoise noise(info.seed);

  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  std::sort(v->begin(), v->end());
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  RCLCPP_INFO(info.node->get_logger(), "threshold: %lf", tmp);

  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  info.cloud->width = pos;
  RCLCPP_INFO(info.node->get_logger(), "points before optimization: %u", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

void
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  RCLCPP_INFO(info.node->get_logger(),
    "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    RCLCPP_INFO(info.node->get_logger(), "entered 5*5 mode");
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides)
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  else if (xl < xh - 2 && yl < yh - 2)
  {
    // bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
           // add a wall at the sides)
           // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    RCLCPP_INFO(info.node->get_logger(), "entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    RCLCPP_INFO(info.node->get_logger(), "entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {
    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    return;
  }
}

void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  RCLCPP_DEBUG(info.node->get_logger(), "%d %d %d %d", sx, sy, px, py);

  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  RCLCPP_DEBUG(info.node->get_logger(), "%d %d %d %d", x1, x2, y1, y2);

  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  loadParameter(info.node, "road_width", width, 1.0);
  loadParameter(info.node, "add_wall_x", addWallX, 0);
  loadParameter(info.node, "add_wall_y", addWallY, 0);
  loadParameter(info.node, "maze_type", type, 1);

  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

Maps::BasicInfo
Maps::getInfo() const
{
  return info;
}

void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3d maze
      std::srand(info.seed);
      Maze3DGen();
      break;
    case 5:
      indoorScene();
      break;
  }
}

pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

void
Maps::Maze3DGen()
{
  // getting required info parameters from the given node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  loadParameter(info.node, "numNodes", numNodes, 10);
  loadParameter(info.node, "connectivity", connectivity, 0.5);
  loadParameter(info.node, "nodeRad", nodeRad, 3);
  loadParameter(info.node, "roadRad", roadRad, 2);
  RCLCPP_INFO(info.node->get_logger(), "received parameters : numNodes: %d connectivity: "
           "%f nodeRad: %d roadRad: %d",
           numNodes,
           connectivity,
           nodeRad,
           roadRad);
  // generating random points
  std::vector<pcl::PointXYZ> base;

  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    RCLCPP_DEBUG(info.node->get_logger(), "point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate points
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // marking the corresponding point location

        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // setting super large starting values
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  RCLCPP_INFO(info.node->get_logger(), "points before optimization: %u", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

// ---------------------------------------------------------------------------
// Helper: fill surface points of an axis-aligned box into a point cloud
// ---------------------------------------------------------------------------
static void
addBoxSurface(pcl::PointCloud<pcl::PointXYZ> &cloud,
              double x_min, double x_max,
              double y_min, double y_max,
              double z_min, double z_max,
              double res)
{
  res = std::max(res, 1e-3);
  const int nx = std::max(1, static_cast<int>(std::ceil((x_max - x_min) / res)));
  const int ny = std::max(1, static_cast<int>(std::ceil((y_max - y_min) / res)));
  const int nz = std::max(1, static_cast<int>(std::ceil((z_max - z_min) / res)));
  const double w = x_max - x_min;
  const double d = y_max - y_min;
  const double h = z_max - z_min;

  pcl::PointXYZ pt;
  auto push = [&](double px, double py, double pz) {
    pt.x = px; pt.y = py; pt.z = pz;
    cloud.points.push_back(pt);
  };

  // XY faces (top z_max and bottom z_min)
  for (int ix = 0; ix <= nx; ++ix)
    for (int iy = 0; iy <= ny; ++iy)
    {
      double px = x_min + w * ix / nx;
      double py = y_min + d * iy / ny;
      push(px, py, z_min);
      push(px, py, z_max);
    }
  // XZ faces (front y_min and back y_max)
  for (int ix = 0; ix <= nx; ++ix)
    for (int iz = 0; iz <= nz; ++iz)
    {
      double px = x_min + w * ix / nx;
      double pz = z_min + h * iz / nz;
      push(px, y_min, pz);
      push(px, y_max, pz);
    }
  // YZ faces (left x_min and right x_max)
  for (int iy = 0; iy <= ny; ++iy)
    for (int iz = 0; iz <= nz; ++iz)
    {
      double py = y_min + d * iy / ny;
      double pz = z_min + h * iz / nz;
      push(x_min, py, pz);
      push(x_max, py, pz);
    }
}

// ---------------------------------------------------------------------------
// Type 5: Indoor scene – room with walls, partition, tables and cabinets
// ---------------------------------------------------------------------------
void
Maps::indoorScene()
{
  const double res = 1.0 / info.scale;  // point cloud resolution
  double surface_res;
  loadParameter(info.node, "surface_resolution", surface_res, res * 0.5);
  surface_res = std::max(surface_res, 1e-3);

  // Room parameters (configurable via ROS params)
  double room_w, room_l, wall_t, wall_h;
  loadParameter(info.node, "indoor.room_width",     room_w, 20.0);
  loadParameter(info.node, "indoor.room_length",    room_l, 20.0);
  loadParameter(info.node, "indoor.wall_thickness", wall_t,  0.2);
  loadParameter(info.node, "indoor.wall_height",    wall_h,  2.5);

  // Door opening in the partition wall, optionally topped by a low lintel beam
  // that forces the robot to squat and crawl through (HAVEN height adaptation).
  double door_w, beam_clearance, beam_depth;
  bool   beam_enable = true;
  loadParameter(info.node, "indoor.door_width",      door_w,         2.0);
  loadParameter(info.node, "indoor.beam_enable",     beam_enable,    true);
  loadParameter(info.node, "indoor.beam_clearance",  beam_clearance, 0.62);
  loadParameter(info.node, "indoor.beam_depth",      beam_depth,     0.6);

  // Robot envelope, keep in sync with the planner's grid_map.* parameters:
  // robot top = base_z + body_top_offset, a ceiling is passable only when
  // ceiling_height >= base_z + body_top_offset + clearance_margin.
  double body_top_offset, base_z_min, base_z_nominal, clear_margin, planner_res;
  loadParameter(info.node, "indoor.body_top_offset",    body_top_offset, 0.328);
  loadParameter(info.node, "indoor.base_z_min",         base_z_min,      0.17);
  loadParameter(info.node, "indoor.base_z_nominal",     base_z_nominal,  0.28);
  loadParameter(info.node, "indoor.clearance_margin",   clear_margin,    0.03);
  loadParameter(info.node, "indoor.planner_resolution", planner_res,     0.05);

  const double hw = room_w * 0.5;   // half-width  (X)
  const double hl = room_l * 0.5;   // half-length (Y)

  auto box = [&](double x0, double x1, double y0, double y1, double z0, double z1) {
    addBoxSurface(*info.cloud, x0, x1, y0, y1, z0, z1, surface_res);
  };

  // ------------------------------------------------------------------
  // 1) Perimeter walls (4 thin tall boxes)
  // ------------------------------------------------------------------
  // South wall  (y = -hl)
  box(-hw, hw,  -hl, -hl + wall_t,  0.0, wall_h);
  // North wall  (y = +hl)
  box(-hw, hw,   hl - wall_t, hl,   0.0, wall_h);
  // West wall   (x = -hw)
  box(-hw, -hw + wall_t,  -hl, hl,  0.0, wall_h);
  // East wall   (x = +hw)
  box( hw - wall_t, hw,   -hl, hl,  0.0, wall_h);

  // ------------------------------------------------------------------
  // 2) Internal partition wall at y=0 with a door opening in centre
  // ------------------------------------------------------------------
  door_w = std::clamp(door_w, 0.0, room_w - 2.0 * wall_t);
  // left segment: x in [-hw+wall_t, -door_w/2]
  box(-hw + wall_t, -door_w * 0.5,  -wall_t * 0.5, wall_t * 0.5,  0.0, wall_h);
  // right segment: x in [door_w/2, hw-wall_t]
  box( door_w * 0.5, hw - wall_t,   -wall_t * 0.5, wall_t * 0.5,  0.0, wall_h);

  // ------------------------------------------------------------------
  // 2b) Low lintel beam above the door opening
  //
  // Same box primitive as the walls, but starting at beam_clearance instead
  // of the floor, so the space underneath stays free: the only path between
  // the two halves of the room requires squatting down and crawling through.
  // ------------------------------------------------------------------
  if (beam_enable && door_w > 0.0)
  {
    const double bd = std::max(beam_depth, wall_t);
    const double z_top = std::max(beam_clearance + surface_res, wall_h);
    box(-door_w * 0.5, door_w * 0.5,  -bd * 0.5, bd * 0.5,  beam_clearance, z_top);

    // The planner sees the beam quantized to its own voxel grid: the reported
    // ceiling height is the bottom of the occupied voxel, i.e. rounded down.
    const double eff_clearance =
        std::floor(beam_clearance / std::max(planner_res, 1e-6)) * planner_res;
    const double base_z_needed = eff_clearance - body_top_offset - clear_margin;

    RCLCPP_INFO(info.node->get_logger(),
                "indoorScene: door lintel bottom %.3f m (planner sees %.3f m after "
                "%.3f m voxelization) -> requires base_z <= %.3f m (leg height %.3f m)",
                beam_clearance, eff_clearance, planner_res, base_z_needed,
                base_z_needed - 0.0322);

    if (base_z_needed < base_z_min)
      RCLCPP_WARN(info.node->get_logger(),
                  "indoorScene: lintel too low, the door is impassable even fully "
                  "squatted (needs base_z <= %.3f m, robot minimum is %.3f m). "
                  "Raise indoor.beam_clearance to at least %.3f m.",
                  base_z_needed, base_z_min,
                  base_z_min + body_top_offset + clear_margin + planner_res);
    else if (base_z_needed >= base_z_nominal)
      RCLCPP_WARN(info.node->get_logger(),
                  "indoorScene: lintel too high, the robot walks through at its "
                  "nominal height %.3f m without squatting. Lower "
                  "indoor.beam_clearance below %.3f m to force squatting.",
                  base_z_nominal, base_z_nominal + body_top_offset + clear_margin);
  }

  // ------------------------------------------------------------------
  // 3) Tables  (1.2m x 0.6m x 0.75m, solid box approximation)
  // ------------------------------------------------------------------
  const double tw = 1.2, td = 0.6, th = 0.75;

  // North area – two tables
  box(-6.0,       -6.0 + tw,   4.0, 4.0 + td,   0.0, th);
  box( 3.0,        3.0 + tw,   5.0, 5.0 + td,   0.0, th);

  // South area – two tables
  box(-2.0,       -2.0 + tw,  -4.0, -4.0 + td,  0.0, th);
  box( 5.0,        5.0 + tw,  -6.0, -6.0 + td,  0.0, th);

  // ------------------------------------------------------------------
  // 4) Cabinets  (0.6m x 0.4m x 1.5m, against walls)
  // ------------------------------------------------------------------
  const double cw = 0.6, cd = 0.4, ch = 1.5;

  // North wall – against inner face
  box(-3.0,       -3.0 + cw,   hl - wall_t - cd, hl - wall_t,  0.0, ch);
  // South area – against partition
  box(-7.0,       -7.0 + cw,  -wall_t * 0.5 - cd, -wall_t * 0.5,  0.0, ch);
  // East wall – south side
  box( hw - wall_t - cd, hw - wall_t,  -7.0, -7.0 + cw,  0.0, ch);

  // ------------------------------------------------------------------
  // Finalize
  // ------------------------------------------------------------------
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  RCLCPP_INFO(info.node->get_logger(),
              "indoorScene: %u points generated", info.cloud->width);
  pcl2ros();
}
