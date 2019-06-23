// Synopsis: Class representing the affine hull of a point set.
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>

#ifndef SEB_SUBSPAN_H
#define SEB_SUBSPAN_H

#include <vector>
#include "Seb_configure.h"

namespace SEB_NAMESPACE {
  
  template<typename Float>
  inline Float sqr(const Float x)
  {
    return x * x;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  class Subspan
  // An instance of this class represents the affine hull of a
  // non-empty set M of affinely independent points.  The set M is not
  // represented explicity; when an instance of this class is
  // constructed, you pass a list S of points to it (which the
  // instance will never change and which is assumed to stay fixed for
  // the lifetime of this instance): The set M is then a subset of S,
  // and its members are identified by their (zero-based) indices in
  // S.  The following routines are provided to query and change the
  // set M:
  //
  // - int size() returns the size of the instance's set M, a number
  //   between 0 and dim+1.
  //   Complexity: O(1).
  //
  // - bool is_member(int global_index) returns true iff S[global_index]
  //   is a member of M.
  //   Complexity: O(1)
  //
  // - global_index(int local_index) returns the index (into S) of the
  //   local_index-th point in M.  The points in M are internally
  //   ordered (in an arbitrary way) and this order only changes when
  //   add() or remove() (see below) is called.
  //   Complexity: O(1)
  //
  // - void add_point(int global_index) adds the global_index-th point
  //   of S to the instance's set M.
  //   Precondition: !is_member(global_index)
  //   Complexity: O(dim^2).
  //
  // - void remove_point(int local_index) removes the local_index-th
  //   point in M.
  //   Precondition: 0<=local_index<=size() and size()>1
  //   Complexity: O(dim^2)
  //
  // - int any_member() returns the global index (into S) of an
  //   arbitrary element of M.
  //   Precondition: size()>0
  //   Postcondition: is_member(any_member())
  //
  // The following routines are provided to query the affine hull of M:
  //
  // - void shortest_vector_to_span(p,w): Computes the vector w
  //   directed from point p to v, where v is the point in aff(M) that
  //   lies nearest to p.  Returned is the squared length of w.
  //   Precondition: size()>0
  //   Complexity: O(dim^2)
  //
  // - void find_affine_coefficients(c,coeffs):
  //   Preconditions: c lies in the affine hull aff(M) and size() > 0.
  //   Calculates the size()-many coefficients in the representation
  //   of c as an affine combination of the points M.  The i-th computed
  //   coefficient coeffs[i] corresponds to the i-th point in M, or,
  //   in other words, to the point in S with index global_index(i).
  //   Complexity: O(dim^2)
  {
  public: // construction and deletion:
    
    Subspan(unsigned int dim, const PointAccessor& S, int i);
    // Constructs an instance representing the affine hull aff(M) of M={p},
    // where p is the point S[i] from S.
    //
    // Notice that S must not changed as long as this instance of
    // Subspan<Float> is in use.
    
    ~Subspan();
    
  public: // modification:
    
    void add_point(int global_index);
    void remove_point(unsigned int local_index);
    
  public: // access:
    
    unsigned int size() const
    {
      return r+1;
    }
    
    bool is_member(unsigned int i) const
    {
      SEB_ASSERT(i < S.size());
      return membership[i];
    }
    
    unsigned int global_index(unsigned int i) const
    {
      SEB_ASSERT(i < size());
      return members[i];
    }
    
    unsigned int any_member() const {
      SEB_ASSERT(size()>0);
      return members[r];
    }
    
    template<typename RandomAccessIterator1,
    typename RandomAccessIterator2>
    Float shortest_vector_to_span(RandomAccessIterator1 p,
                                  RandomAccessIterator2 w);
    
    template<typename RandomAccessIterator1,
    typename RandomAccessIterator2>
    void find_affine_coefficients(RandomAccessIterator1 c,
                                  RandomAccessIterator2 coeffs);
    
  public: // debugging routines:
    
    Float representation_error();
    // Computes the coefficient representations of all points in the
    // (internally used) system (Q) and returns the maximal deviation
    // from the theoretical values.
    // Warning: This routine has running time O(dim^3).
    
  private: // private helper routines:
    
    void append_column();
    // Appends the new column u (which is a member of this instance) to
    // the right of "A = QR", updating Q and R.  It assumes r to still
    // be the old value, i.e., the index of the column used now for
    // insertion; r is not altered by this routine and should be changed
    // by the caller afterwards.
    // Precondition: r<dim
    
    void hessenberg_clear(unsigned int start);
    // Given R in lower Hessenberg form with subdiagonal entries 0 to
    // pos-1 already all zero, clears the remaining subdiagonal entries
    // via Givens rotations.
    
    void special_rank_1_update();
    // Update current QR-decomposition "A = QR" to
    // A + u [1,...,1] = Q' R'.
    
  private: // member fields:
    const PointAccessor &S;            // a const-reference to the set S
    std::vector<bool> membership;      // S[i] in M iff membership[i]
    const unsigned int dim;            // ambient dimension (not to be
    // confused with the rank r,
    // see below)
    
    // Entry i of members contains the index into S of the i-th point
    // in M.  The point members[r] is called the "origin."
    std::vector<unsigned int> members;
    
  private: // member fields for maintaining the QR-decomposition:
    Float **Q, **R;                    // (dim x dim)-matrices Q
    // (orthogonal) and R (upper
    // triangular); notice that
    // e.g.  Q[j][i] is the element
    // in row i and column j
    Float *u,*w;                       // needed for rank-1 update
    unsigned int r;                    // the rank of R (i.e. #points - 1)
  };
  
} // namespace SEB_NAMESPACE

#include "Subspan-inl.h"

#endif // SEB_SUBSPAN_H
