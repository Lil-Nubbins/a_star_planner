#ifndef CELL_H_
#define CELL_H_

#include <cmath>
#include <algorithm>

namespace a_star_planner {
  class Cell
  {
    public:
      //constructors and destructor
      Cell();
      Cell(int x, int y);
      Cell(int x, int y, double cost);
      ~Cell();

      //getters
      inline int getX() const    {return x_;}
      inline int getY() const    {return y_;}
      inline double getCost() const {return cost_;}

      //setters
      inline void setX(int new_x)      {x_ = new_x;}
      inline void setY(int new_y)      {y_ = new_y;}
      inline void setCost(int new_cost){cost_ = new_cost;}

      //Comparison operator overloads
      inline bool operator<(const Cell& rhs)  const { return this->getCost() < rhs.getCost();}
      inline bool operator>(const Cell& rhs)  const { return rhs < *this;}
      inline bool operator<=(const Cell& rhs) const { return !(*this > rhs);}
      inline bool operator>=(const Cell& rhs) const { return !(*this < rhs);}

      //Equality operator overloads
      inline bool operator==(const Cell& rhs) const { return ((this->getX() == rhs.getX()) && (this->getY() == rhs.getY()));}
      inline bool operator!=(const Cell& rhs) const { return !(*this == rhs); }

      //compute the heuristic cost of this cell from the goal
      int computeHeuristic(Cell goal);

    private:
      //Cells have a position in 2D space and a cost
      int x_;
      int y_;
      double cost_;
  };
};
#endif