/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <cassert>

#include <pdal/KDIndex.hpp>
#include <filters/NormalFilter.hpp>

#include "GreedyProjection.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.greedyprojection",
    "Greedy Triangulation filter",
    "http://pdal.org/stages/filters.greedyprojection.html"
};

CREATE_STATIC_STAGE(GreedyProjection, s_info)

std::string GreedyProjection::getName() const
{
    return s_info.name;
}


void GreedyProjection::addArgs(ProgramArgs& args)
{
    args.add("multiplier", "Nearest neighbor distance multiplier",
        mu_).setPositional();
    args.add("radius", "Search radius for neighbors",
        search_radius_).setPositional();
    args.add("num_neighbors", "Number of nearest neighbors to consider",
        nnn_, 100);
    args.add("min_angle", "Minimum angle for created triangles",
        minimum_angle_, M_PI / 18);  // 10 degrees default
    args.add("max_angle", "Maximum angle for created triangles",
        maximum_angle_, 2 * M_PI / 3);  // 120 degrees default
    args.add("eps_angle", "Max normal difference angle for triangulation "
        "consideration", eps_angle_, M_PI / 4);
}


void GreedyProjection::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims( { Dimension::Id::NormalX, Dimension::Id::NormalY,
        Dimension::Id::NormalZ } );
}


void GreedyProjection::initialize()
{
    if (search_radius_ <= 0)
        throwError("Invalid search radius of '" +
            std::to_string(search_radius_) + "'.  Must be greater than 0.");
    if (mu_ <= 0)
        throwError("Invalid distance multiplier of '" +
            std::to_string(mu_) + "'.  Must be greater than 0.");
}

Eigen::Vector3d GreedyProjection::getCoord(PointId id)
{
    assert(view_);

    return Eigen::Vector3d(
        view_->getFieldAs<double>(Dimension::Id::X, id),
        view_->getFieldAs<double>(Dimension::Id::Y, id),
        view_->getFieldAs<double>(Dimension::Id::Z, id)
    );
}


Eigen::Vector3d GreedyProjection::getNormalCoord(PointId id)
{
    assert(view_);

    return Eigen::Vector3d(
        view_->getFieldAs<double>(Dimension::Id::NormalX, id),
        view_->getFieldAs<double>(Dimension::Id::NormalY, id),
        view_->getFieldAs<double>(Dimension::Id::NormalZ, id)
    );
}


void GreedyProjection::addTriangle(PointId a, PointId b, PointId c)
{
    mesh_->add(a, b, c);
}


void GreedyProjection::filter(PointView& view)
{
    NormalFilter nf;
    nf.setLog(log());
    nf.doFilter(view);

    KD3Index& tree = view.build3dIndex();

    view_ = &view;
    mesh_ = view_->createMesh(getName());
    const double sqr_mu = mu_ * mu_;
    const double sqr_max_edge = search_radius_*search_radius_;

    nnn_ = (int)(std::min)((point_count_t)nnn_, view.size());

    // Variables to hold the results of nearest neighbor searches
    PointIdList nnIdx(nnn_);
    std::vector<double> sqrDists(nnn_);

    // current number of connected components
    int part_index = 0;

    // 2D coordinates of points
    const Eigen::Vector2d uvn_nn_qp_zero = Eigen::Vector2d::Zero();
    Eigen::Vector2d uvn_current;
    Eigen::Vector2d uvn_prev;
    Eigen::Vector2d uvn_next;

    // initializing fields
    already_connected_ = false; // see declaration for comments :P

    // initializing states and fringe neighbors
    part_.clear();
    state_.clear();
    source_.clear();
    ffn_.clear();
    sfn_.clear();
    part_.resize(view.size()); // indices of point's part
    state_.resize(view.size(), GP3Type::FREE);
    source_.resize(view.size());
    ffn_.resize(view.size());
    sfn_.resize(view.size());
    fringe_queue_.clear();
    int fqIdx = 0; // current fringe's index in the queue to be processed

  // Avoiding NaN coordinates if needed
  /**
  if (!input_->is_dense)
  {
    // Skip invalid points from the indices list
    for (std::vector<int>::const_iterator it = indices_->begin ();o
        it != indices_->end (); ++it)
      if (!pcl_isfinite (input_->points[*it].x) ||
          !pcl_isfinite (input_->points[*it].y) ||
          !pcl_isfinite (input_->points[*it].z))
        state_[*it] = GP3Type::NONE;
  }
  **/

  // Saving coordinates and point to index mapping
  /**
  coords_.clear ();
  coords_.reserve (indices_->size ());
  std::vector<int> point2index (input_->points.size (), -1);
  for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
  {
    coords_.push_back(input_->points[(*indices_)[cp]].getVector3dMap());
    point2index[(*indices_)[cp]] = cp;
  }
  **/

  // Initializing
  PointId isFree = 0;
  bool done = false;
  int nr_parts=0, increase_nnn4fn=0, increase_nnn4s=0, increase_dist=0;
  bool is_fringe;
  angles_.resize(nnn_);
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > uvn_nn (nnn_);
  Eigen::Vector2d uvn_s;

  // iterating through fringe points and finishing them until everything is done
  while (!done)
  {
    R_ = isFree;
    if (state_[R_] == GP3Type::FREE)
    {
      state_[R_] = GP3Type::NONE;
//ABELL - What is part_index?
      part_[R_] = part_index++;

      // creating starting triangle
      tree.knnSearch(R_, nnn_, &nnIdx, &sqrDists);

      double sqr_dist_threshold =
          (std::min)(sqr_max_edge, sqr_mu * sqrDists[1]);

/**
      // Search tree returns indices into the original cloud, but we are working with indices. TODO: make that optional!
      for (int i = 1; i < nnn_; i++)
      {
        //if (point2index[nnIdx[i]] == -1)
        //  std::cerr << R_ << " [" << indices_->at (R_) << "] " << i << ": " << nnIdx[i] << " / " << point2index[nnIdx[i]] << std::endl;
        nnIdx[i] = point2index[nnIdx[i]];
      }
**/

      // Get the normal estimate at the current point
      const Eigen::Vector3d nc(getNormalCoord(R_));

      // Get a coordinate system that lies on a plane defined by its normal
      v_ = nc.unitOrthogonal ();
      u_ = nc.cross (v_);

      // Projecting point onto the surface
      Eigen::Vector3d coord(getCoord(R_));
      proj_qp_ = coord - nc.dot(coord) * nc;

      // Converting coords, calculating angles and saving the
      // projected near boundary edges
      size_t nr_edge = 0;
      std::vector<doubleEdge> doubleEdges;
      // nearest neighbor with index 0 is the query point R_ itself
      for (int i = 1; i < nnn_; i++)
      {
        // Transforming coordinates
        tmp_ = getCoord(nnIdx[i]) - proj_qp_;
        uvn_nn[i][0] = tmp_.dot(u_);
        uvn_nn[i][1] = tmp_.dot(v_);
        // Computing the angle between each neighboring point and
        // the query point itself
        angles_[i].angle = atan2(uvn_nn[i][1], uvn_nn[i][0]);
        // initializing angle descriptors
        angles_[i].index = nnIdx[i];
        if ((state_[nnIdx[i]] == GP3Type::COMPLETED) ||
            (state_[nnIdx[i]] == GP3Type::BOUNDARY) ||
            (state_[nnIdx[i]] == GP3Type::NONE) ||
            (sqrDists[i] > sqr_dist_threshold))
          angles_[i].visible = false;
        else
          angles_[i].visible = true;
        // Saving the edges between nearby boundary points
        if ((state_[nnIdx[i]] == GP3Type::FRINGE) ||
            (state_[nnIdx[i]] == GP3Type::BOUNDARY))
        {
          doubleEdge e;
          e.index = i;
          nr_edge++;
          tmp_ = getCoord(ffn_[nnIdx[i]]) - proj_qp_;
          e.first[0] = tmp_.dot(u_);
          e.first[1] = tmp_.dot(v_);
          tmp_ = getCoord(sfn_[nnIdx[i]]) - proj_qp_;
          e.second[0] = tmp_.dot(u_);
          e.second[1] = tmp_.dot(v_);
          doubleEdges.push_back(e);
        }
      }
      angles_[0].visible = false;

      // Verify the visibility of each potential new vertex
      // nearest neighbor with index 0 is the query point R_ itself
      for (int i = 1; i < nnn_; i++)
        if ((angles_[i].visible) && (ffn_[R_] != nnIdx[i]) &&
            (sfn_[R_] != nnIdx[i]))
        {
          bool visibility = true;
          for (size_t j = 0; j < nr_edge; j++)
          {
            if (ffn_[nnIdx[doubleEdges[j].index]] != nnIdx[i])
              visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index],
                doubleEdges[j].first, Eigen::Vector2d::Zero());
            if (!visibility)
              break;
            if (sfn_[nnIdx[doubleEdges[j].index]] != nnIdx[i])
              visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index],
                  doubleEdges[j].second, Eigen::Vector2d::Zero());
            if (!visibility == false)
              break;
          }
          angles_[i].visible = visibility;
        }

      // Selecting first two visible free neighbors
      bool not_found = true;
      PointId left = 1;
      do
      {
        while ((left < (PointId)nnn_) &&
            ((!angles_[left].visible) || stateSet(nnIdx[left])))
            left++;
        if (left >= (PointId)nnn_)
          break;
        else
        {
          PointId right = left+1;
          do
          {
            while ((right < (PointId)nnn_) &&
                ((!angles_[right].visible) || stateSet(nnIdx[right])))
                    right++;
            if (right >= (PointId)nnn_)
              break;
            else if ((getCoord(nnIdx[left]) -
                getCoord(nnIdx[right])).squaredNorm () > sqr_max_edge)
              right++;
            else
            {
              addFringePoint (nnIdx[right], R_);
              addFringePoint (nnIdx[left], nnIdx[right]);
              addFringePoint (R_, nnIdx[left]);
              state_[R_] = state_[nnIdx[left]] =
                  state_[nnIdx[right]] = GP3Type::FRINGE;
              ffn_[R_] = nnIdx[left];
              sfn_[R_] = nnIdx[right];
              ffn_[nnIdx[left]] = nnIdx[right];
              sfn_[nnIdx[left]] = R_;
              ffn_[nnIdx[right]] = R_;
              sfn_[nnIdx[right]] = nnIdx[left];
              addTriangle (R_, nnIdx[left], nnIdx[right]);
              nr_parts++;
              not_found = false;
              break;
            }
          }
          while (true);
          left++;
        }
      }
      while (not_found);
    }

    // Set the index of the first free point in isFree.
    done = true;
    auto it = std::find(state_.begin(), state_.end(), GP3Type::FREE);
    if (it == state_.end())
        done = true;
    else
    {
        done = false;
        isFree = std::distance(state_.begin(), it);
    }

    is_fringe = true;
    while (is_fringe)
    {
      is_fringe = false;

      int fqSize = static_cast<int> (fringe_queue_.size ());
      while ((fqIdx < fqSize) &&
          (state_[fringe_queue_[fqIdx]] != GP3Type::FRINGE))
        fqIdx++;

      // an unfinished fringe point is found
      if (fqIdx >= fqSize)
      {
        continue;
      }

      R_ = fringe_queue_[fqIdx];
      is_fringe = true;

      if (ffn_[R_] == sfn_[R_])
      {
        state_[R_] = GP3Type::COMPLETED;
        continue;
      }
      tree.knnSearch(R_, nnn_, &nnIdx, &sqrDists);

/**
      // Search tree returns indices into the original cloud, but we are working with indices TODO: make that optional!
      for (int i = 1; i < nnn_; i++)
      {
        //if (point2index[nnIdx[i]] == -1)
        //  std::cerr << R_ << " [" << indices_->at (R_) << "] " << i << ": " << nnIdx[i] << " / " << point2index[nnIdx[i]] << std::endl;
        nnIdx[i] = point2index[nnIdx[i]];
      }
**/

      Eigen::Vector3d coord(getCoord(R_));
      // Locating FFN and SFN to adapt distance threshold
      double sqr_source_dist = (coord - getCoord(source_[R_])).squaredNorm ();
      double sqr_ffn_dist = (coord - getCoord(ffn_[R_])).squaredNorm ();
      double sqr_sfn_dist = (coord - getCoord(sfn_[R_])).squaredNorm ();
      double max_sqr_fn_dist = (std::max)(sqr_ffn_dist, sqr_sfn_dist);
      double sqr_dist_threshold = (std::min)(sqr_max_edge, sqr_mu * sqrDists[1]);
      if (max_sqr_fn_dist > sqrDists[nnn_-1])
      {
        if (0 == increase_nnn4fn)
          log()->get(LogLevel::Warning) << "Not enough neighbors are "
              "considered: ffn or sfn out of range! Consider increasing "
              "nnn_... Setting R = " << R_ << " to be BOUNDARY!\n";
        increase_nnn4fn++;
        state_[R_] = GP3Type::BOUNDARY;
        continue;
      }
      double max_sqr_fns_dist = (std::max)(sqr_source_dist, max_sqr_fn_dist);
      if (max_sqr_fns_dist > sqrDists[nnn_-1])
      {
        if (0 == increase_nnn4s)
          log()->get(LogLevel::Warning) << "Not enough neighbors are "
              "considered: source of R " << R_ << " is out of range! "
              "Consider increasing nnn_...\n";
        increase_nnn4s++;
      }

      // Get the normal estimate at the current point
      const Eigen::Vector3d nc(getNormalCoord(R_));

      // Get a coordinate system that lies on a plane defined by its normal
      v_ = nc.unitOrthogonal ();
      u_ = nc.cross (v_);

      const Eigen::Vector3d c(getCoord(R_));
      // Projecting point onto the surface
      proj_qp_ = c - nc.dot(c)* nc;

      // Converting coords, calculating angles and saving the projected
      // near boundary edges
      size_t nr_edge = 0;
      std::vector<doubleEdge> doubleEdges;
      // nearest neighbor with index 0 is the query point R_ itself
      for (int i = 1; i < nnn_; i++)
      {
        tmp_ = getCoord(nnIdx[i]) - proj_qp_;
        uvn_nn[i][0] = tmp_.dot(u_);
        uvn_nn[i][1] = tmp_.dot(v_);

        // Computing the angle between each neighboring point and the
        // query point itself
        angles_[i].angle = atan2(uvn_nn[i][1], uvn_nn[i][0]);
        // initializing angle descriptors
        angles_[i].index = nnIdx[i];
        angles_[i].nnIndex = i;
        if ((state_[nnIdx[i]] == GP3Type::COMPLETED) ||
            (state_[nnIdx[i]] == GP3Type::BOUNDARY) ||
            (state_[nnIdx[i]] == GP3Type::NONE) ||
            (sqrDists[i] > sqr_dist_threshold))
          angles_[i].visible = false;
        else
          angles_[i].visible = true;
        if ((ffn_[R_] == nnIdx[i]) || (sfn_[R_] == nnIdx[i]))
          angles_[i].visible = true;
        bool same_side = true;
        const Eigen::Vector3d neighbor_normal = getNormalCoord(nnIdx[i]);
        double cosine = nc.dot (neighbor_normal);
        if (cosine > 1) cosine = 1;
        if (cosine < -1) cosine = -1;
        double angle = acos (cosine);
        if ((!consistent_) && (angle > M_PI/2))
          angle = M_PI - angle;
        if (angle > eps_angle_)
        {
          angles_[i].visible = false;
          same_side = false;
        }
        // Saving the edges between nearby boundary points
        if ((i!=0) && (same_side) &&
            ((state_[nnIdx[i]] == GP3Type::FRINGE) ||
             (state_[nnIdx[i]] == GP3Type::BOUNDARY)))
        {
          doubleEdge e;
          e.index = i;
          nr_edge++;
          tmp_ = getCoord(ffn_[nnIdx[i]]) - proj_qp_;
          e.first[0] = tmp_.dot(u_);
          e.first[1] = tmp_.dot(v_);
          tmp_ = getCoord(sfn_[nnIdx[i]]) - proj_qp_;
          e.second[0] = tmp_.dot(u_);
          e.second[1] = tmp_.dot(v_);
          doubleEdges.push_back(e);
          // Pruning by visibility criterion
          if ((state_[nnIdx[i]] == GP3Type::FRINGE) &&
              (ffn_[R_] != nnIdx[i]) && (sfn_[R_] != nnIdx[i]))
          {
            double angle1 = atan2(e.first[1] - uvn_nn[i][1],
                e.first[0] - uvn_nn[i][0]);
            double angle2 = atan2(e.second[1] - uvn_nn[i][1],
                e.second[0] - uvn_nn[i][0]);
            double angleMin, angleMax;
            if (angle1 < angle2)
            {
              angleMin = angle1;
              angleMax = angle2;
            }
            else
            {
              angleMin = angle2;
              angleMax = angle1;
            }
            double angleR = angles_[i].angle + M_PI;
            if (angleR >= 2*M_PI)
              angleR -= 2*M_PI;
            if ((source_[nnIdx[i]] == ffn_[nnIdx[i]]) || (source_[nnIdx[i]] == sfn_[nnIdx[i]]))
            {
              if ((angleMax - angleMin) < M_PI)
              {
                if ((angleMin < angleR) && (angleR < angleMax))
                  angles_[i].visible = false;
              }
              else
              {
                if ((angleR < angleMin) || (angleMax < angleR))
                  angles_[i].visible = false;
              }
            }
            else
            {
              tmp_ = getCoord(source_[nnIdx[i]]) - proj_qp_;
              uvn_s[0] = tmp_.dot(u_);
              uvn_s[1] = tmp_.dot(v_);
              double angleS = atan2(uvn_s[1] - uvn_nn[i][1],
                  uvn_s[0] - uvn_nn[i][0]);
              if ((angleMin < angleS) && (angleS < angleMax))
              {
                if ((angleMin < angleR) && (angleR < angleMax))
                  angles_[i].visible = false;
              }
              else
              {
                if ((angleR < angleMin) || (angleMax < angleR))
                  angles_[i].visible = false;
              }
            }
          }
        }
      }
      angles_[0].visible = false;

      // Verify the visibility of each potential new vertex
      // nearest neighbor with index 0 is the query point R_ itself
      for (int i = 1; i < nnn_; i++)
        if ((angles_[i].visible) &&
            (ffn_[R_] != nnIdx[i]) && (sfn_[R_] != nnIdx[i]))
        {
          bool visibility = true;
          for (size_t j = 0; j < nr_edge; j++)
          {
            //ABELL - This seems weird.  i is just a count of the nearest
            // neighbors, not a point index.  Are some indicies the index
            // of the nearest neighbors?  If so, confusing.
            if (doubleEdges[j].index != (PointId)i)
            {
              PointId f = ffn_[nnIdx[doubleEdges[j].index]];
              if ((f != nnIdx[i]) && (f != R_))
                visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index],
                    doubleEdges[j].first, Eigen::Vector2d::Zero());
              if (visibility == false)
                break;

              PointId s = sfn_[nnIdx[doubleEdges[j].index]];
              if ((s != nnIdx[i]) && (s != R_))
                visibility = isVisible(uvn_nn[i], uvn_nn[doubleEdges[j].index],
                    doubleEdges[j].second, Eigen::Vector2d::Zero());
              if (visibility == false)
                break;
            }
          }
          angles_[i].visible = visibility;
        }

      // Sorting angles
      std::sort(angles_.begin (), angles_.end (),
          [](const nnAngle& a1, const nnAngle& a2)
      {
          return (a1.visible == a2.visible ?
            a1.angle < a2.angle :
            a1.visible);
      });

      // Triangulating
      if (angles_[2].visible == false)
      {
        if ( !( (angles_[0].index == ffn_[R_] && angles_[1].index == sfn_[R_])
            || (angles_[0].index == sfn_[R_] && angles_[1].index == ffn_[R_]) ))
        {
          state_[R_] = GP3Type::BOUNDARY;
        }
        else
        {
          if ((source_[R_] == angles_[0].index) ||
              (source_[R_] == angles_[1].index))
            state_[R_] = GP3Type::BOUNDARY;
          else
          {
            if (sqr_max_edge <
                (getCoord(ffn_[R_]) - getCoord(sfn_[R_])).squaredNorm ())
            {
              state_[R_] = GP3Type::BOUNDARY;
            }
            else
            {
              tmp_ = getCoord(source_[R_]) - proj_qp_;
              uvn_s[0] = tmp_.dot(u_);
              uvn_s[1] = tmp_.dot(v_);
              double angleS = atan2(uvn_s[1], uvn_s[0]);
              double dif = angles_[1].angle - angles_[0].angle;
              if ((angles_[0].angle < angleS) && (angleS < angles_[1].angle))
              {
                if (dif < 2*M_PI - maximum_angle_)
                  state_[R_] = GP3Type::BOUNDARY;
                else
                  closeTriangle();
              }
              else
              {
                if (dif >= maximum_angle_)
                  state_[R_] = GP3Type::BOUNDARY;
                else
                  closeTriangle();
              }
            }
          }
        }
        continue;
      }

      // Finding the FFN and SFN
      int start = -1, end = -1;
      for (int i=0; i<nnn_; i++)
      {
        if (ffn_[R_] == angles_[i].index)
        {
          start = i;
          if (sfn_[R_] == angles_[i+1].index)
            end = i+1;
          else
            if (i==0)
            {
              for (i = i+2; i < nnn_; i++)
                if (sfn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
            else
            {
              for (i = i+2; i < nnn_; i++)
                if (sfn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
          break;
        }
        if (sfn_[R_] == angles_[i].index)
        {
          start = i;
          if (ffn_[R_] == angles_[i+1].index)
            end = i+1;
          else
            if (i==0)
            {
              for (i = i+2; i < nnn_; i++)
                if (ffn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
            else
            {
              for (i = i+2; i < nnn_; i++)
                if (ffn_[R_] == angles_[i].index)
                  break;
              end = i;
            }
          break;
        }
      }

      // start and end are always set, as we checked if ffn or sfn are out
      // of range before, but let's check anyways if < 0
      if ((start < 0) || (end < 0) || (end == nnn_) ||
          (!angles_[start].visible) || (!angles_[end].visible))
      {
        state_[R_] = GP3Type::BOUNDARY;
        continue;
      }

      // Finding last visible nn
      int last_visible = end;
      while ((last_visible+1<nnn_) && (angles_[last_visible+1].visible))
          last_visible++;

      // Finding visibility region of R
      bool need_invert = false;
      int sourceIdx = nnn_;
      if ((source_[R_] == ffn_[R_]) || (source_[R_] == sfn_[R_]))
      {
        if ((angles_[end].angle - angles_[start].angle) < M_PI)
          need_invert = true;
      }
      else
      {
        for (sourceIdx=0; sourceIdx<nnn_; sourceIdx++)
          if (angles_[sourceIdx].index == source_[R_])
            break;
        if (sourceIdx == nnn_)
        {
          // any free visible and nearest completed or boundary neighbor of R
          int vis_free(-1);
          int nnCB(-1);

          // nearest neighbor with index 0 is the query point R_ itself
          for (int i = 1; i < nnn_; i++)
          {
            // NOTE: nnCB is an index in nnIdx
            if ((state_[nnIdx[i]] == GP3Type::COMPLETED) ||
                (state_[nnIdx[i]] == GP3Type::BOUNDARY))
            {
              if (nnCB == -1)
              {
                nnCB = i;
                if (vis_free != -1)
                  break;
              }
            }
            // NOTE: vis_free is an index in angles
            if (!stateSet(angles_[i].index))
            {
              if (i <= last_visible)
              {
                vis_free = i;
                if (nnCB != -1)
                  break;
              }
            }
          }
          // NOTE: nCB is an index in angles
          int nCB = 0;
          if (nnCB != -1)
            while (angles_[nCB].index != nnIdx[nnCB]) nCB++;
          else
            nCB = -1;

          if (vis_free != -1)
          {
            if ((vis_free < start) || (vis_free > end))
              need_invert = true;
          }
          else
          {
            if (nCB != -1)
            {
              if ((nCB == start) || (nCB == end))
              {
                bool inside_CB = false;
                bool outside_CB = false;
                for (int i=0; i<nnn_; i++)
                {
                  if (
                      ((state_[angles_[i].index] == GP3Type::COMPLETED) ||
                       (state_[angles_[i].index] == GP3Type::BOUNDARY)) &&
                       (i != start) && (i != end)
                     )
                    {
                      if ((angles_[start].angle <= angles_[i].angle) &&
                          (angles_[i].angle <= angles_[end].angle))
                      {
                        inside_CB = true;
                        if (outside_CB)
                          break;
                      }
                      else
                      {
                        outside_CB = true;
                        if (inside_CB)
                          break;
                      }
                    }
                }
                if (inside_CB && !outside_CB)
                  need_invert = true;
                else if (!(!inside_CB && outside_CB))
                {
                  if ((angles_[end].angle - angles_[start].angle) < M_PI)
                    need_invert = true;
                }
              }
              else
              {
                if ((angles_[nCB].angle > angles_[start].angle) &&
                    (angles_[nCB].angle < angles_[end].angle))
                  need_invert = true;
              }
            }
            else
            {
              if (start == end-1)
                need_invert = true;
            }
          }
        }
        else if ((angles_[start].angle < angles_[sourceIdx].angle) &&
            (angles_[sourceIdx].angle < angles_[end].angle))
          need_invert = true;
      }

      // switching start and end if necessary
      if (need_invert)
        std::swap(start, end);

      // Arranging visible nnAngles in the order they need to be connected and
      // compute the maximal angle difference between two consecutive visible
      // angles
      bool is_boundary = false, is_skinny = false;
      std::vector<bool> gaps (nnn_, false);
      std::vector<bool> skinny (nnn_, false);
      std::vector<double> dif (nnn_);
      std::vector<int> angleIdx; angleIdx.reserve (nnn_);
      if (start > end)
      {
        for (int j=start; j<last_visible; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) &&
              (sqr_max_edge <
                  (getCoord(angles_[j+1].index) - getCoord(angles_[j].index)).
                  squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }

        dif[last_visible] = (2 * M_PI + angles_[0].angle -
            angles_[last_visible].angle);
        if (dif[last_visible] < minimum_angle_)
        {
          skinny[last_visible] = is_skinny = true;
        }
        else if (maximum_angle_ <= dif[last_visible])
        {
          gaps[last_visible] = is_boundary = true;
        }
        if ((!gaps[last_visible]) &&
            (sqr_max_edge < (getCoord(angles_[0].index) -
                getCoord(angles_[last_visible].index)).squaredNorm ()))
        {
          gaps[last_visible] = is_boundary = true;
        }
        angleIdx.push_back(last_visible);

        for (int j=0; j<end; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) &&
              (sqr_max_edge < (getCoord(angles_[j+1].index) -
                  getCoord(angles_[j].index)).squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }
        angleIdx.push_back(end);
      }
      // start < end
      else
      {
        for (int j=start; j<end; j++)
        {
          dif[j] = (angles_[j+1].angle - angles_[j].angle);
          if (dif[j] < minimum_angle_)
          {
            skinny[j] = is_skinny = true;
          }
          else if (maximum_angle_ <= dif[j])
          {
            gaps[j] = is_boundary = true;
          }
          if ((!gaps[j]) &&
              (sqr_max_edge < (getCoord(angles_[j+1].index) -
                  getCoord(angles_[j].index)).squaredNorm ()))
          {
            gaps[j] = is_boundary = true;
          }
          angleIdx.push_back(j);
        }
        angleIdx.push_back(end);
      }

      // Set the state of the point
      state_[R_] = is_boundary ? GP3Type::BOUNDARY : GP3Type::COMPLETED;

      std::vector<int>::iterator first_gap_after = angleIdx.end ();
      std::vector<int>::iterator last_gap_before = angleIdx.begin ();
      int nr_gaps = 0;
      for (auto it = angleIdx.begin (); it != angleIdx.end () - 1; it++)
      {
        if (gaps[*it])
        {
          nr_gaps++;
          if (first_gap_after == angleIdx.end())
            first_gap_after = it;
          last_gap_before = it + 1;
        }
      }
      if (nr_gaps > 1)
      {
        angleIdx.erase(first_gap_after+1, last_gap_before);
      }

      // Neglecting points that would form skinny triangles (if possible)
      if (is_skinny)
      {
        double angle_so_far = 0, angle_would_be;
        double max_combined_angle =
            (std::min)(maximum_angle_, M_PI-2*minimum_angle_);
        Eigen::Vector2d X;
        Eigen::Vector2d S1;
        Eigen::Vector2d S2;
        std::vector<int> to_erase;
        for (auto it = angleIdx.begin()+1; it != angleIdx.end()-1; it++)
        {
          if (gaps[*(it-1)])
            angle_so_far = 0;
          else
            angle_so_far += dif[*(it-1)];
          if (gaps[*it])
            angle_would_be = angle_so_far;
          else
            angle_would_be = angle_so_far + dif[*it];
          if (
              (skinny[*it] || skinny[*(it-1)]) &&
              (!stateSet(angles_[*it].index) ||
               !stateSet(angles_[*(it-1)].index)) &&
              ((!gaps[*it]) ||
                  (angles_[*it].nnIndex > angles_[*(it-1)].nnIndex)) &&
              ((!gaps[*(it-1)]) ||
                  (angles_[*it].nnIndex > angles_[*(it+1)].nnIndex)) &&
              (angle_would_be < max_combined_angle)
             )
          {
            if (gaps[*(it-1)])
            {
              gaps[*it] = true;
              to_erase.push_back(*it);
            }
            else if (gaps[*it])
            {
              gaps[*(it-1)] = true;
              to_erase.push_back(*it);
            }
            else
            {
              int erased_idx = static_cast<int> (to_erase.size ()) -1;
              auto prev_it = it - 1;
              for (; (erased_idx != -1) && (it != angleIdx.begin()); it--)
                if (*it == to_erase[erased_idx])
                  erased_idx--;
                else
                  break;
              bool can_delete = true;
              for (auto curr_it = prev_it+1; curr_it != it+1; curr_it++)
              {
                tmp_ = getCoord(angles_[*curr_it].index) - proj_qp_;
                X[0] = tmp_.dot(u_);
                X[1] = tmp_.dot(v_);
                tmp_ = getCoord(angles_[*prev_it].index) - proj_qp_;
                S1[0] = tmp_.dot(u_);
                S1[1] = tmp_.dot(v_);
                tmp_ = getCoord(angles_[*(it+1)].index) - proj_qp_;
                S2[0] = tmp_.dot(u_);
                S2[1] = tmp_.dot(v_);
                // check for inclusions
                if (isVisible(X,S1,S2))
                {
                  can_delete = false;
                  angle_so_far = 0;
                  break;
                }
              }
              if (can_delete)
              {
                to_erase.push_back(*it);
              }
            }
          }
          else
            angle_so_far = 0;
        }
        for (auto it = to_erase.begin(); it != to_erase.end(); it++)
        {
          for (auto iter = angleIdx.begin(); iter != angleIdx.end(); iter++)
            if (*it == *iter)
            {
              angleIdx.erase(iter);
              break;
            }
        }
      }

      // Writing edges and updating edge-front
      changed_1st_fn_ = false;
      changed_2nd_fn_ = false;
//ABELL - This seems bad - sentinel for index that's positive.
      new2boundary_ = -1;
      for (auto it = angleIdx.begin() + 1; it != angleIdx.end() - 1; it++)
      {
        current_index_ = angles_[*it].index;

        is_current_free_ = false;
        if (!stateSet(current_index_))
        {
          state_[current_index_] = GP3Type::FRINGE;
          is_current_free_ = true;
        }
        else if (!already_connected_)
        {
          prev_is_ffn_ = (ffn_[current_index_] ==
            angles_[*(it-1)].index) && (!gaps[*(it-1)]);
          prev_is_sfn_ = (sfn_[current_index_] ==
            angles_[*(it-1)].index) && (!gaps[*(it-1)]);
          next_is_ffn_ = (ffn_[current_index_] ==
              angles_[*(it+1)].index) && (!gaps[*it]);
          next_is_sfn_ = (sfn_[current_index_] ==
              angles_[*(it+1)].index) && (!gaps[*it]);
//           if (!prev_is_ffn_ && !next_is_sfn_ && !prev_is_sfn_ && !next_is_ffn_)
//           {
//             nr_touched++;
//           }
        }

        if (gaps[*it])
          if (gaps[*(it-1)])
          {
            if (is_current_free_)
              state_[current_index_] = GP3Type::NONE; /// TODO: document!
          }

          else // (gaps[*it]) && ^(gaps[*(it-1)])
          {
            addTriangle (current_index_, angles_[*(it-1)].index, R_);
            addFringePoint (current_index_, R_);
            new2boundary_ = current_index_;
            if (!already_connected_)
              connectPoint (angles_[*(it-1)].index, R_, angles_[*(it+1)].index,
                  uvn_nn[angles_[*it].nnIndex],
                  uvn_nn[angles_[*(it-1)].nnIndex], uvn_nn_qp_zero);
            else
                already_connected_ = false;
            if (ffn_[R_] == angles_[*(angleIdx.begin())].index)
            {
              ffn_[R_] = new2boundary_;
            }
            else if (sfn_[R_] == angles_[*(angleIdx.begin())].index)
            {
              sfn_[R_] = new2boundary_;
            }
          }

        else // ^(gaps[*it])
          if (gaps[*(it-1)])
          {
            addFringePoint (current_index_, R_);
            new2boundary_ = current_index_;
            if (!already_connected_)
                connectPoint(R_, angles_[*(it+1)].index,
                    (it+2) == angleIdx.end() ? -1 : angles_[*(it+2)].index,
                    uvn_nn[angles_[*it].nnIndex], uvn_nn_qp_zero,
                    uvn_nn[angles_[*(it+1)].nnIndex]);
            else
                already_connected_ = false;
            if (ffn_[R_] == angles_[*(angleIdx.end()-1)].index)
            {
              ffn_[R_] = new2boundary_;
            }
            else if (sfn_[R_] == angles_[*(angleIdx.end()-1)].index)
            {
              sfn_[R_] = new2boundary_;
            }
          }

          else // ^(gaps[*it]) && ^(gaps[*(it-1)])
          {
            addTriangle (current_index_, angles_[*(it-1)].index, R_);
            addFringePoint (current_index_, R_);
            if (!already_connected_)
                connectPoint (angles_[*(it-1)].index, angles_[*(it+1)].index,
                    (it+2) == angleIdx.end() ? -1 :
                        gaps[*(it+1)] ? R_ :
                        angles_[*(it+2)].index,
                    uvn_nn[angles_[*it].nnIndex],
                    uvn_nn[angles_[*(it-1)].nnIndex],
                    uvn_nn[angles_[*(it+1)].nnIndex]);
            else
                already_connected_ = false;
          }
      }

      // Finishing up R_
      if (ffn_[R_] == sfn_[R_])
      {
        state_[R_] = GP3Type::COMPLETED;
      }
      if (!gaps[*(angleIdx.end()-2)])
      {
        addTriangle (angles_[*(angleIdx.end()-2)].index,
            angles_[*(angleIdx.end()-1)].index, R_);
        addFringePoint (angles_[*(angleIdx.end()-2)].index, R_);
        if (R_ == ffn_[angles_[*(angleIdx.end()-1)].index])
        {
          if (angles_[*(angleIdx.end()-2)].index ==
              sfn_[angles_[*(angleIdx.end()-1)].index])
          {
            state_[angles_[*(angleIdx.end()-1)].index] = GP3Type::COMPLETED;
          }
          else
          {
            ffn_[angles_[*(angleIdx.end()-1)].index] =
                angles_[*(angleIdx.end()-2)].index;
          }
        }
        else if (R_ == sfn_[angles_[*(angleIdx.end()-1)].index])
        {
          if (angles_[*(angleIdx.end()-2)].index ==
              ffn_[angles_[*(angleIdx.end()-1)].index])
          {
            state_[angles_[*(angleIdx.end()-1)].index] = GP3Type::COMPLETED;
          }
          else
          {
            sfn_[angles_[*(angleIdx.end()-1)].index] =
                angles_[*(angleIdx.end()-2)].index;
          }
        }
      }
      if (!gaps[*(angleIdx.begin())])
      {
        if (R_ == ffn_[angles_[*(angleIdx.begin())].index])
        {
          if (angles_[*(angleIdx.begin()+1)].index ==
              sfn_[angles_[*(angleIdx.begin())].index])
          {
            state_[angles_[*(angleIdx.begin())].index] = GP3Type::COMPLETED;
          }
          else
          {
            ffn_[angles_[*(angleIdx.begin())].index] =
                angles_[*(angleIdx.begin()+1)].index;
          }
        }
        else if (R_ == sfn_[angles_[*(angleIdx.begin())].index])
        {
          if (angles_[*(angleIdx.begin()+1)].index ==
              ffn_[angles_[*(angleIdx.begin())].index])
          {
            state_[angles_[*(angleIdx.begin())].index] = GP3Type::COMPLETED;
          }
          else
          {
            sfn_[angles_[*(angleIdx.begin())].index] =
                angles_[*(angleIdx.begin()+1)].index;
          }
        }
      }
    }
  }
  log()->get(LogLevel::Debug) << "Number of triangles: " <<
      mesh_->size() << ".\n";
  log()->get(LogLevel::Debug) << "Number of unconnected parts: " << nr_parts <<
      ".\n";
  if (increase_nnn4fn > 0)
      log()->get(LogLevel::Warning) << "Number of neighborhood size "
          "increase requests for fringe neighbors: " << increase_nnn4fn <<
          ".\n";
  if (increase_nnn4s > 0)
      log()->get(LogLevel::Warning) << "Number of neighborhood size "
          "increase requests for source: " << increase_nnn4s << ".\n";
  if (increase_dist > 0)
      log()->get(LogLevel::Warning) << "Number of automatic maximum "
          "distance increases: " << increase_dist << ".\n";

  // sorting and removing doubles from fringe queue
  std::sort (fringe_queue_.begin(), fringe_queue_.end ());
  fringe_queue_.erase (std::unique(fringe_queue_.begin(), fringe_queue_.end()),
      fringe_queue_.end ());
  log()->get(LogLevel::Debug) << "Number of processed points: " <<
      fringe_queue_.size() << " / " << view.size() << "!\n";
  view_ = nullptr;
}

void GreedyProjection::closeTriangle ()
{
  state_[R_] = GP3Type::COMPLETED;
  addTriangle (angles_[0].index, angles_[1].index, R_);
  for (int aIdx=0; aIdx<2; aIdx++)
  {
    if (ffn_[angles_[aIdx].index] == R_)
    {
      if (sfn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index)
      {
        state_[angles_[aIdx].index] = GP3Type::COMPLETED;
      }
      else
      {
        ffn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
      }
    }
    else if (sfn_[angles_[aIdx].index] == R_)
    {
      if (ffn_[angles_[aIdx].index] == angles_[(aIdx+1)%2].index)
      {
        state_[angles_[aIdx].index] = GP3Type::COMPLETED;
      }
      else
      {
        sfn_[angles_[aIdx].index] = angles_[(aIdx+1)%2].index;
      }
    }
  }
}

void GreedyProjection::connectPoint (
    PointId prev_index, PointId next_index, PointId next_next_index,
    const Eigen::Vector2d &uvn_current,
    const Eigen::Vector2d &uvn_prev,
    const Eigen::Vector2d &uvn_next)
{
  if (is_current_free_)
  {
    ffn_[current_index_] = prev_index;
    sfn_[current_index_] = next_index;
  }
  else
  {
    if ((prev_is_ffn_ && next_is_sfn_) || (prev_is_sfn_ && next_is_ffn_))
      state_[current_index_] = GP3Type::COMPLETED;
    else if (prev_is_ffn_ && !next_is_sfn_)
      ffn_[current_index_] = next_index;
    else if (next_is_ffn_ && !prev_is_sfn_)
      ffn_[current_index_] = prev_index;
    else if (prev_is_sfn_ && !next_is_ffn_)
      sfn_[current_index_] = next_index;
    else if (next_is_sfn_ && !prev_is_ffn_)
      sfn_[current_index_] = prev_index;
    else
    {
      bool found_triangle = false;
      if ((prev_index != R_) &&
          ((ffn_[current_index_] == ffn_[prev_index]) ||
           (ffn_[current_index_] == sfn_[prev_index])))
      {
        found_triangle = true;
        addTriangle (current_index_, ffn_[current_index_], prev_index);
        state_[prev_index] = GP3Type::COMPLETED;
        state_[ffn_[current_index_]] = GP3Type::COMPLETED;
        ffn_[current_index_] = next_index;
      }
      else if ((prev_index != R_) &&
          ((sfn_[current_index_] == ffn_[prev_index]) ||
           (sfn_[current_index_] == sfn_[prev_index])))
      {
        found_triangle = true;
        addTriangle (current_index_, sfn_[current_index_], prev_index);
        state_[prev_index] = GP3Type::COMPLETED;
        state_[sfn_[current_index_]] = GP3Type::COMPLETED;
        sfn_[current_index_] = next_index;
      }
      else if (stateSet(next_index))
      {
        if ((ffn_[current_index_] == ffn_[next_index]) ||
            (ffn_[current_index_] == sfn_[next_index]))
        {
          found_triangle = true;
          addTriangle (current_index_, ffn_[current_index_], next_index);

          if (ffn_[current_index_] == ffn_[next_index])
          {
            ffn_[next_index] = current_index_;
          }
          else
          {
            sfn_[next_index] = current_index_;
          }
          state_[ffn_[current_index_]] = GP3Type::COMPLETED;
          ffn_[current_index_] = prev_index;
        }
        else if ((sfn_[current_index_] == ffn_[next_index]) ||
            (sfn_[current_index_] == sfn_[next_index]))
        {
          found_triangle = true;
          addTriangle (current_index_, sfn_[current_index_], next_index);

          if (sfn_[current_index_] == ffn_[next_index])
          {
            ffn_[next_index] = current_index_;
          }
          else
          {
            sfn_[next_index] = current_index_;
          }
          state_[sfn_[current_index_]] = GP3Type::COMPLETED;
          sfn_[current_index_] = prev_index;
        }
      }

      if (found_triangle)
      {
      }
      else
      {
        tmp_ = getCoord(ffn_[current_index_]) - proj_qp_;
        uvn_ffn_[0] = tmp_.dot(u_);
        uvn_ffn_[1] = tmp_.dot(v_);
        tmp_ = getCoord(sfn_[current_index_]) - proj_qp_;
        uvn_sfn_[0] = tmp_.dot(u_);
        uvn_sfn_[1] = tmp_.dot(v_);
        bool prev_ffn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_ffn_) &&
            isVisible(uvn_prev, uvn_sfn_, uvn_current, uvn_ffn_);
        bool prev_sfn = isVisible(uvn_prev, uvn_next, uvn_current, uvn_sfn_) &&
            isVisible(uvn_prev, uvn_ffn_, uvn_current, uvn_sfn_);
        bool next_ffn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_ffn_) &&
            isVisible(uvn_next, uvn_sfn_, uvn_current, uvn_ffn_);
        bool next_sfn = isVisible(uvn_next, uvn_prev, uvn_current, uvn_sfn_) &&
            isVisible(uvn_next, uvn_ffn_, uvn_current, uvn_sfn_);
        int min_dist = -1;
        if (prev_ffn && next_sfn && prev_sfn && next_ffn)
        {
          /* should be never the case */
          double prev2f = (getCoord(ffn_[current_index_]) -
              getCoord(prev_index)).squaredNorm ();
          double next2s = (getCoord(sfn_[current_index_]) -
              getCoord(next_index)).squaredNorm ();
          double prev2s = (getCoord(sfn_[current_index_]) -
              getCoord(prev_index)).squaredNorm ();
          double next2f = (getCoord(ffn_[current_index_]) -
              getCoord(next_index)).squaredNorm ();
          if (prev2f < prev2s)
          {
            if (prev2f < next2f)
            {
              if (prev2f < next2s)
                min_dist = 0;
              else
                min_dist = 3;
            }
            else
            {
              if (next2f < next2s)
                min_dist = 2;
              else
                min_dist = 3;
            }
          }
          else
          {
            if (prev2s < next2f)
            {
              if (prev2s < next2s)
                min_dist = 1;
              else
                min_dist = 3;
            }
            else
            {
              if (next2f < next2s)
                min_dist = 2;
              else
                min_dist = 3;
            }
          }
        }
        else if (prev_ffn && next_sfn)
        {
          /* a clear case */
          double prev2f = (getCoord(ffn_[current_index_]) -
              getCoord(prev_index)).squaredNorm();
          double next2s = (getCoord(sfn_[current_index_]) -
              getCoord(next_index)).squaredNorm ();
          if (prev2f < next2s)
            min_dist = 0;
          else
            min_dist = 3;
        }
        else if (prev_sfn && next_ffn)
        {
          /* a clear case */
          double prev2s = (getCoord(sfn_[current_index_]) -
              getCoord(prev_index)).squaredNorm ();
          double next2f = (getCoord(ffn_[current_index_]) -
              getCoord(next_index)).squaredNorm ();
          if (prev2s < next2f)
            min_dist = 1;
          else
            min_dist = 2;
        }
        /* straightforward cases */
        else if (prev_ffn && !next_sfn && !prev_sfn && !next_ffn)
          min_dist = 0;
        else if (!prev_ffn && !next_sfn && prev_sfn && !next_ffn)
          min_dist = 1;
        else if (!prev_ffn && !next_sfn && !prev_sfn && next_ffn)
          min_dist = 2;
        else if (!prev_ffn && next_sfn && !prev_sfn && !next_ffn)
          min_dist = 3;
        /* messed up cases */
        else if (prev_ffn)
        {
          double prev2f = (getCoord(ffn_[current_index_]) -
              getCoord(prev_index)).squaredNorm ();
          if (prev_sfn)
          {
            double prev2s = (getCoord(sfn_[current_index_]) -
                getCoord(prev_index)).squaredNorm ();
            if (prev2s < prev2f)
              min_dist = 1;
            else
              min_dist = 0;
          }
          else if (next_ffn)
          {
            double next2f = (getCoord(ffn_[current_index_]) -
                getCoord(next_index)).squaredNorm ();
            if (next2f < prev2f)
              min_dist = 2;
            else
              min_dist = 0;
          }
        }
        else if (next_sfn)
        {
          double next2s = (getCoord(sfn_[current_index_]) -
              getCoord(next_index)).squaredNorm ();
          if (prev_sfn)
          {
            double prev2s = (getCoord(sfn_[current_index_]) -
                getCoord(prev_index)).squaredNorm ();
            if (prev2s < next2s)
              min_dist = 1;
            else
              min_dist = 3;
          }
          else if (next_ffn)
          {
            double next2f = (getCoord(ffn_[current_index_]) -
                getCoord(next_index)).squaredNorm ();
            if (next2f < next2s)
              min_dist = 2;
            else
              min_dist = 3;
          }
        }
        switch (min_dist)
        {
          case 0://prev2f:
          {
            addTriangle (current_index_, ffn_[current_index_], prev_index);

            /* updating prev_index */
            if (ffn_[prev_index] == current_index_)
            {
              ffn_[prev_index] = ffn_[current_index_];
            }
            else if (sfn_[prev_index] == current_index_)
            {
              sfn_[prev_index] = ffn_[current_index_];
            }
            else if (ffn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              ffn_[prev_index] = ffn_[current_index_];
            }
            else if (sfn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              sfn_[prev_index] = ffn_[current_index_];
            }
            else if (prev_index == R_)
            {
              new2boundary_ = ffn_[current_index_];
            }

            /* updating ffn */
            if (ffn_[ffn_[current_index_]] == current_index_)
            {
              ffn_[ffn_[current_index_]] = prev_index;
            }
            else if (sfn_[ffn_[current_index_]] == current_index_)
            {
              sfn_[ffn_[current_index_]] = prev_index;
            }

            /* updating current */
            ffn_[current_index_] = next_index;

            break;
          }
          case 1://prev2s:
          {
            addTriangle (current_index_, sfn_[current_index_], prev_index);

            /* updating prev_index */
            if (ffn_[prev_index] == current_index_)
            {
              ffn_[prev_index] = sfn_[current_index_];
            }
            else if (sfn_[prev_index] == current_index_)
            {
              sfn_[prev_index] = sfn_[current_index_];
            }
            else if (ffn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              ffn_[prev_index] = sfn_[current_index_];
            }
            else if (sfn_[prev_index] == R_)
            {
              changed_1st_fn_ = true;
              sfn_[prev_index] = sfn_[current_index_];
            }
            else if (prev_index == R_)
            {
              new2boundary_ = sfn_[current_index_];
            }

            /* updating sfn */
            if (ffn_[sfn_[current_index_]] == current_index_)
            {
              ffn_[sfn_[current_index_]] = prev_index;
            }
            else if (sfn_[sfn_[current_index_]] == current_index_)
            {
              sfn_[sfn_[current_index_]] = prev_index;
            }

            /* updating current */
            sfn_[current_index_] = next_index;

            break;
          }
          case 2://next2f:
          {
            addTriangle (current_index_, ffn_[current_index_], next_index);
            PointId neighbor_update = next_index;

            /* updating next_index */
            if (!stateSet(next_index))
            {
              state_[next_index] = GP3Type::FRINGE;
              ffn_[next_index] = current_index_;
              sfn_[next_index] = ffn_[current_index_];
            }
            else
            {
              if (ffn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                ffn_[next_index] = ffn_[current_index_];
              }
              else if (sfn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                sfn_[next_index] = ffn_[current_index_];
              }
              else if (next_index == R_)
              {
                new2boundary_ = ffn_[current_index_];
                if (next_next_index == new2boundary_)
                  already_connected_ = true;
              }
              else if (ffn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                ffn_[next_index] = ffn_[current_index_];
              }
              else if (sfn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                sfn_[next_index] = ffn_[current_index_];
              }
              else
              {
                tmp_ = getCoord(ffn_[next_index]) - proj_qp_;
                uvn_next_ffn_[0] = tmp_.dot(u_);
                uvn_next_ffn_[1] = tmp_.dot(v_);
                tmp_ = getCoord(sfn_[next_index]) - proj_qp_;
                uvn_next_sfn_[0] = tmp_.dot(u_);
                uvn_next_sfn_[1] = tmp_.dot(v_);

                bool ffn_next_ffn =
                  isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_ffn_) &&
                  isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_ffn_);
                bool sfn_next_ffn =
                  isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_ffn_) &&
                  isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_ffn_);

                int connect2ffn = -1;
                if (ffn_next_ffn && sfn_next_ffn)
                {
                  double fn2f = (getCoord(ffn_[current_index_]) -
                      getCoord(ffn_[next_index])).squaredNorm ();
                  double sn2f = (getCoord(ffn_[current_index_]) -
                      getCoord(sfn_[next_index])).squaredNorm ();
                  if (fn2f < sn2f)
                      connect2ffn = 0;
                  else
                      connect2ffn = 1;
                }
                else if (ffn_next_ffn)
                    connect2ffn = 0;
                else if (sfn_next_ffn)
                    connect2ffn = 1;

                switch (connect2ffn)
                {
                  case 0: // ffn[next]
                  {
                    addTriangle (next_index, ffn_[current_index_],
                        ffn_[next_index]);
                    neighbor_update = ffn_[next_index];

                    /* ffn[next_index] */
                    if ((ffn_[ffn_[next_index]] == ffn_[current_index_]) ||
                        (sfn_[ffn_[next_index]] == ffn_[current_index_]))
                    {
                      state_[ffn_[next_index]] = GP3Type::COMPLETED;
                    }
                    else if (ffn_[ffn_[next_index]] == next_index)
                    {
                      ffn_[ffn_[next_index]] = ffn_[current_index_];
                    }
                    else if (sfn_[ffn_[next_index]] == next_index)
                    {
                      sfn_[ffn_[next_index]] = ffn_[current_index_];
                    }

                    ffn_[next_index] = current_index_;

                    break;
                  }
                  case 1: // sfn[next]
                  {
                    addTriangle (next_index, ffn_[current_index_],
                        sfn_[next_index]);
                    neighbor_update = sfn_[next_index];

                    /* sfn[next_index] */
                    if ((ffn_[sfn_[next_index]] = ffn_[current_index_]) ||
                        (sfn_[sfn_[next_index]] == ffn_[current_index_]))
                    {
                      state_[sfn_[next_index]] = GP3Type::COMPLETED;
                    }
                    else if (ffn_[sfn_[next_index]] == next_index)
                    {
                      ffn_[sfn_[next_index]] = ffn_[current_index_];
                    }
                    else if (sfn_[sfn_[next_index]] == next_index)
                    {
                      sfn_[sfn_[next_index]] = ffn_[current_index_];
                    }

                    sfn_[next_index] = current_index_;

                    break;
                  }
                  default:;
                }
              }
            }

            /* updating ffn */
            if ((ffn_[ffn_[current_index_]] == neighbor_update) ||
                (sfn_[ffn_[current_index_]] == neighbor_update))
            {
              state_[ffn_[current_index_]] = GP3Type::COMPLETED;
            }
            else if (ffn_[ffn_[current_index_]] == current_index_)
            {
              ffn_[ffn_[current_index_]] = neighbor_update;
            }
            else if (sfn_[ffn_[current_index_]] == current_index_)
            {
              sfn_[ffn_[current_index_]] = neighbor_update;
            }

            /* updating current */
            ffn_[current_index_] = prev_index;

            break;
          }
          case 3://next2s:
          {
            addTriangle (current_index_, sfn_[current_index_], next_index);
            PointId neighbor_update = next_index;

            /* updating next_index */
            if (!stateSet(next_index))
            {
              state_[next_index] = GP3Type::FRINGE;
              ffn_[next_index] = current_index_;
              sfn_[next_index] = sfn_[current_index_];
            }
            else
            {
              if (ffn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                ffn_[next_index] = sfn_[current_index_];
              }
              else if (sfn_[next_index] == R_)
              {
                changed_2nd_fn_ = true;
                sfn_[next_index] = sfn_[current_index_];
              }
              else if (next_index == R_)
              {
                new2boundary_ = sfn_[current_index_];
                if (next_next_index == new2boundary_)
                  already_connected_ = true;
              }
              else if (ffn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                ffn_[next_index] = sfn_[current_index_];
              }
              else if (sfn_[next_index] == next_next_index)
              {
                already_connected_ = true;
                sfn_[next_index] = sfn_[current_index_];
              }
              else
              {
                tmp_ = getCoord(ffn_[next_index]) - proj_qp_;
                uvn_next_ffn_[0] = tmp_.dot(u_);
                uvn_next_ffn_[1] = tmp_.dot(v_);
                tmp_ = getCoord(sfn_[next_index]) - proj_qp_;
                uvn_next_sfn_[0] = tmp_.dot(u_);
                uvn_next_sfn_[1] = tmp_.dot(v_);

                bool ffn_next_sfn =
                    isVisible(uvn_next_ffn_, uvn_next, uvn_current, uvn_sfn_) &&
                    isVisible(uvn_next_ffn_, uvn_next, uvn_next_sfn_, uvn_sfn_);
                bool sfn_next_sfn =
                    isVisible(uvn_next_sfn_, uvn_next, uvn_current, uvn_sfn_) &&
                    isVisible(uvn_next_sfn_, uvn_next, uvn_next_ffn_, uvn_sfn_);

                int connect2sfn = -1;
                if (ffn_next_sfn && sfn_next_sfn)
                {
                  double fn2s = (getCoord(sfn_[current_index_]) -
                      getCoord(ffn_[next_index])).squaredNorm ();
                  double sn2s = (getCoord(sfn_[current_index_]) -
                      getCoord(sfn_[next_index])).squaredNorm ();
                  if (fn2s < sn2s)
                      connect2sfn = 0;
                  else
                      connect2sfn = 1;
                }
                else if (ffn_next_sfn)
                    connect2sfn = 0;
                else if (sfn_next_sfn)
                    connect2sfn = 1;

                switch (connect2sfn)
                {
                  case 0: // ffn[next]
                  {
                    addTriangle (next_index, sfn_[current_index_],
                        ffn_[next_index]);
                    neighbor_update = ffn_[next_index];

                    /* ffn[next_index] */
                    if ((ffn_[ffn_[next_index]] == sfn_[current_index_]) ||
                        (sfn_[ffn_[next_index]] == sfn_[current_index_]))
                    {
                      state_[ffn_[next_index]] = GP3Type::COMPLETED;
                    }
                    else if (ffn_[ffn_[next_index]] == next_index)
                    {
                      ffn_[ffn_[next_index]] = sfn_[current_index_];
                    }
                    else if (sfn_[ffn_[next_index]] == next_index)
                    {
                      sfn_[ffn_[next_index]] = sfn_[current_index_];
                    }

                    ffn_[next_index] = current_index_;

                    break;
                  }
                  case 1: // sfn[next]
                  {
                    addTriangle (next_index, sfn_[current_index_],
                        sfn_[next_index]);
                    neighbor_update = sfn_[next_index];

                    /* sfn[next_index] */
                    if ((ffn_[sfn_[next_index]] == sfn_[current_index_]) ||
                        (sfn_[sfn_[next_index]] == sfn_[current_index_]))
                    {
                      state_[sfn_[next_index]] = GP3Type::COMPLETED;
                    }
                    else if (ffn_[sfn_[next_index]] == next_index)
                    {
                      ffn_[sfn_[next_index]] = sfn_[current_index_];
                    }
                    else if (sfn_[sfn_[next_index]] == next_index)
                    {
                      sfn_[sfn_[next_index]] = sfn_[current_index_];
                    }

                    sfn_[next_index] = current_index_;

                    break;
                  }
                  default:;
                }
              }
            }

            /* updating sfn */
            if ((ffn_[sfn_[current_index_]] == neighbor_update) ||
                (sfn_[sfn_[current_index_]] == neighbor_update))
            {
              state_[sfn_[current_index_]] = GP3Type::COMPLETED;
            }
            else if (ffn_[sfn_[current_index_]] == current_index_)
            {
              ffn_[sfn_[current_index_]] = neighbor_update;
            }
            else if (sfn_[sfn_[current_index_]] == current_index_)
            {
              sfn_[sfn_[current_index_]] = neighbor_update;
            }

            sfn_[current_index_] = prev_index;

            break;
          }
          default:;
        }
      }
    }
  }
}

/**
std::vector<std::vector<size_t> >
GreedyProjection::getTriangleList (const pcl::PolygonMesh &input)
{
  std::vector<std::vector<size_t> > triangleList (input.cloud.width * input.cloud.height);

  for (size_t i=0; i < input.polygons.size (); ++i)
    for (size_t j=0; j < input.polygons[i].vertices.size (); ++j)
      triangleList[input.polygons[i].vertices[j]].push_back (i);
  return (triangleList);
}
**/

} // namespace pdal
