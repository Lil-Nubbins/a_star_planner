#include <a_star_planner/cell.h>

namespace a_star_planner 
{
  Cell::Cell()
  : x_(0.0), y_(0.0), cost_(-1.0){}

  Cell::Cell(double x, double y)
  {
    x_ = x;
    y_ = y;
    cost_ = -1.0;
  }

  Cell::Cell(double x, double y, double cost)
  {
    x_ = x;
    y_ = y;
    cost_ = cost;
  }

  int Cell::computeHeuristic(Cell goal)
  {
    int dx = std::abs(x_ - goal.getX());
    int dy = std::abs(y_ - goal.getY());
    int D = 1;
    int D2 = 1;
    return D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);
  }

  Cell::~Cell(){}
};