#pragma once
#include <Eigen/Core>

namespace motion2d
{
/** @brief Compact no-pivot LU for the ordered MINCO band system (chapter 15).
 * @details Stores only 2*bandwidth+1 diagonals. Fixed bandwidth yields O(n)
 * factorization/solve. This is not a general pivoting linear solver: an unsafe
 * pivot causes an explicit numerical failure. Assemble, factor once, then solve.
 */
class BandedLu
{
public:
  BandedLu(int size,int bandwidth);
  /** @brief Assembly access; caller supplies an index inside the stored band. */
  double & at(int row,int column);
  /** @brief Factor in place; throws on a nonfinite or near-zero pivot. */
  void factor();
  /** @brief Solve A*X=B or A^T*X=B using the same factors; B has size rows.
   * @pre factor() has succeeded and right_hand_side is finite.
   */
  Eigen::MatrixXd solve(Eigen::MatrixXd right_hand_side,bool transpose=false) const;
private:
  int size_,bandwidth_;
  Eigen::MatrixXd bands_;
  double get(int row,int column) const {return bands_(row,column-row+bandwidth_);}
};
}  // namespace motion2d
