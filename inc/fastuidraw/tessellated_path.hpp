/*!
 * \file tessellated_path.hpp
 * \brief file tessellated_path.hpp
 *
 * Copyright 2018 by Intel.
 *
 * Contact: kevin.rogovin@intel.com
 *
 * This Source Code Form is subject to the
 * terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with
 * this file, You can obtain one at
 * http://mozilla.org/MPL/2.0/.
 *
 * \author Kevin Rogovin <kevin.rogovin@intel.com>
 *
 */


#pragma once


#include <fastuidraw/util/fastuidraw_memory.hpp>
#include <fastuidraw/util/rect.hpp>
#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/c_array.hpp>
#include <fastuidraw/util/reference_counted.hpp>
#include <fastuidraw/path_enums.hpp>

namespace fastuidraw  {

///@cond
class Path;
class StrokedPath;
class FilledPath;
///@endcond

/*!\addtogroup Paths
 * @{
 */

/*!
 * \brief
 * An TessellatedPath represents the tessellation of a Path
 * into line segments and arcs.
 *
 * A single contour of a TessellatedPath is constructed from
 * a single \ref PathContour of the source \ref Path. Each
 * edge of a contour of a TessellatedPath is contructed from
 * a single \ref PathContour::interpolator_base of the source \ref
 * PathContour. The ordering of the contours of a
 * TessellatedPath is the same ordering as the source
 * \ref PathContour objects of the source \ref Path. Also,
 * the ordering of edges within a contour is the same ordering
 * as the \ref PathContour::interpolator_base objects of
 * the source \ref PathContour. In particular, for each contour
 * of a TessellatedPath, if an edge is closed, the closing edge
 * is the last edge.
 */
class TessellatedPath:
    public reference_counted<TessellatedPath>::non_concurrent
{
public:
  /*!
   * \brief
   * Enumeration to identify the type of a \ref segment
   */
  enum segment_type_t
    {
      /*!
       * Indicates that the segment is an arc segment,
       * i.e. it connects two point via an arc of a
       * circle
       */
      arc_segment,

      /*!
       * Indicates that the segment is a line segment
       * i.e. it connects two point via a line.
       */
      line_segment,
    };

  /*!
   * \brief
   * A TessellationParams stores how finely to tessellate
   * the curves of a path.
   */
  class TessellationParams
  {
  public:
    /*!
     * Ctor, initializes values.
     */
    TessellationParams(void):
      m_max_distance(-1.0f),
      m_max_recursion(5)
    {}

    /*!
     * Provided as a conveniance. Equivalent to
     * \code
     * m_max_distance = tp;
     * \endcode
     * \param p value to which to assign to \ref m_max_distance
     */
    TessellationParams&
    max_distance(float p)
    {
      m_max_distance = p;
      return *this;
    }

    /*!
     * Set the value of \ref m_max_recursion.
     * \param v value to which to assign to \ref m_max_recursion
     */
    TessellationParams&
    max_recursion(unsigned int v)
    {
      m_max_recursion = v;
      return *this;
    }

    /*!
     * Maximum distance to attempt between the actual curve and the
     * tessellation. A value less than or equal to zero indicates to
     * accept any distance value between the tessellation and the
     * curve. Default value is -1.0 (i.e. accept any distance value).
     */
    float m_max_distance;

    /*!
     * Maximum number of times to perform recursion to tessellate an edge.
     * Default value is 5.
     */
    unsigned int m_max_recursion;
  };

  /*!
   * \brief
   * Represents segment of a tessellated or arc-tessellated path.
   */
  class segment
  {
  public:
    /*!
     * Specifies the segment type.
     */
    enum segment_type_t m_type;

    /*!
     * Gives the start point on the path of the segment.
     */
    vec2 m_start_pt;

    /*!
     * Gives the end point on the path of the segment.
     */
    vec2 m_end_pt;

    /*!
     * Only valid if \ref m_type is \ref arc_segment; gives
     * the center of the arc.
     */
    vec2 m_center;

    /*!
     * Only valid if \ref m_type is \ref arc_segment; gives
     * the angle range of the arc.
     */
    range_type<float> m_arc_angle;

    /*!
     * Only valid if \ref m_type is \ref arc_segment; gives
     * the radius of the arc.
     */
    float m_radius;

    /*!
     * Gives the length of the segment.
     */
    float m_length;

    /*!
     * Gives the distance of the start of the segment from
     * the start of the edge (i.e PathContour::interpolator_base).
     */
    float m_distance_from_edge_start;

    /*!
     * Gives the distance of the start of segment to the
     * start of the -contour-.
     */
    float m_distance_from_contour_start;

    /*!
     * Gives the length of the edge (i.e.
     * PathContour::interpolator_base) on which the
     * segment lies. This value is the same for all
     * segments along a fixed edge.
     */
    float m_edge_length;

    /*!
     * Gives the length of the contour on which this
     * segment lies. This value is the same for all
     * segments along a fixed contour.
     */
    float m_contour_length;

    /*!
     * Gives the unit-vector of the path entering the segment.
     */
    vec2 m_enter_segment_unit_vector;

    /*!
     * Gives the unit-vector of the path leaving the segment.
     */
    vec2 m_leaving_segment_unit_vector;

    /*!
     * If true, indicates that the arc is tangent with its
     * predecessor. This happens when TessellatedPath breaks
     * a \ref segment into smaller pieces to make its angle
     * smaller or to make it monotonic.
     */
    bool m_tangent_with_predecessor;
  };

  /*!
   * \brief
   * A wrapper over a dynamic array of \ref segment objects;
   * segment values added to SegmentStorage must be added
   * in order of time along the domain of a \ref
   * PathContour::interpolator_base
   */
  class SegmentStorage:fastuidraw::noncopyable
  {
  public:
    /*!
     * Add a \ref segment to the SegmentStorage that is a
     * line segment between two points.
     * \param start the starting point of the segment
     * \param end the ending point of the segment
     */
    void
    add_line_segment(vec2 start, vec2 end);

    /*!
     * Add a \ref segment to the SegmentStorage that is an
     * arc segment. If necessary, An arc-segment will be broken
     * into multiple segments to that each segment is monotonic
     * in the x and y coordiantes and each segment's arc-angle
     * is no more than FASTUIDRAW_PI / 4.0 radians (45 degrees).
     * \param start gives the start point of the arc on the path
     * \param end gives the end point of the arc on the path
     * \param center is the center of the circle that defines the arc
     * \param radius is the radius of the circle that defines the arc
     * \param arc_angle is the arc-angle range that defines the arc
     */
    void
    add_arc_segment(vec2 start, vec2 end,
                    vec2 center, float radius,
                    range_type<float> arc_angle);

  private:
    SegmentStorage(void) {}
    ~SegmentStorage() {}

    friend class TessellatedPath;
    void *m_d;
  };

  /*!
   * A Refiner is stateful object that creates new TessellatedPath
   * objects from a starting TessellatedPath where the tessellation
   * is made finer.
   */
  class Refiner:
    public reference_counted<Refiner>::non_concurrent
  {
  public:
    virtual
    ~Refiner();

    /*!
     * Update the TessellatedPath returned by tessellated_path() by
     * refining the current value returned by tessellated_path().
     * \param max_distance new maximum distance to aim for
     * \param additional_recursion amount by which to additionally recurse
     *                             when tessellating.
     */
    void
    refine_tessellation(float max_distance,
                        unsigned int additional_recursion);

    /*!
     * Returns the current TessellatedPath of this Refiner.
     */
    reference_counted_ptr<TessellatedPath>
    tessellated_path(void) const;

  private:
    friend class TessellatedPath;
    Refiner(TessellatedPath *p, const Path &path);

    void *m_d;
  };

  /*!
   * Ctor. Construct a TessellatedPath from a Path
   * \param input source path to tessellate
   * \param P parameters on how to tessellate the source Path
   * \param ref if non-NULL, construct a Refiner object and return
   *            the value via upading the value of ref.
   */
  TessellatedPath(const Path &input, TessellationParams P,
                  reference_counted_ptr<Refiner> *ref = nullptr);

  ~TessellatedPath();

  /*!
   * Returns the tessellation parameters used to construct
   * this TessellatedPath.
   */
  const TessellationParams&
  tessellation_parameters(void) const;

  /*!
   * Returns true if and only if there is a \ref segment
   * in segment_data() for which segment::m_type is
   * \ref arc_segment.
   */
  bool
  has_arcs(void) const;

  /*!
   * Returns the maximum across all edges of all contours
   * of the distance between the tessellation and the actual
   * path.
   */
  float
  max_distance(void) const;

  /*!
   * Returns the maximum recursion employed by any edge
   */
  unsigned int
  max_recursion(void) const;

  /*!
   * Returns all the segment data
   */
  c_array<const segment>
  segment_data(void) const;

  /*!
   * Returns the number of contours
   */
  unsigned int
  number_contours(void) const;

  /*!
   * Returns true if the named contour was closed
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   */
  bool
  contour_closed(unsigned int contour) const;

  /*!
   * Returns the range into segment_data() for the named
   * contour.
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   */
  range_type<unsigned int>
  contour_range(unsigned int contour) const;

  /*!
   * Returns the segment data of the named contour.
   * Provided as a conveniance equivalent to
   * \code
   * segment_data().sub_array(contour_range(contour))
   * \endcode
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   */
  c_array<const segment>
  contour_segment_data(unsigned int contour) const;

  /*!
   * Returns the number of edges for the named contour
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   */
  unsigned int
  number_edges(unsigned int contour) const;

  /*!
   * Returns the range into segment_data(void)
   * for the named edge of the named contour.
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   * \param edge which edge of the contour to query, must
   *             have that 0 <= edge < number_edges(contour)
   */
  range_type<unsigned int>
  edge_range(unsigned int contour, unsigned int edge) const;

  /*!
   * Returns the segment data of the named edge of the
   * named contour, provided as a conveniance, equivalent
   * to
   * \code
   * segment_data().sub_array(edge_range(contour, edge))
   * \endcode
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   * \param edge which edge of the contour to query, must
   *             have that 0 <= edge < number_edges(contour)
   */
  c_array<const segment>
  edge_segment_data(unsigned int contour, unsigned int edge) const;

  /*!
   * Returns the edge type of the named edge of the named contour
   * of the source Path.
   * \param contour which path contour to query, must have
   *                that 0 <= contour < number_contours()
   * \param edge which edge of the contour to query, must
   *             have that 0 <= edge < number_edges(contour)
   */
  enum PathEnums::edge_type_t
  edge_type(unsigned int contour, unsigned int edge) const;

  /*!
   * Returns the bounding box of the tessellation.
   */
  const Rect&
  bounding_box(void) const;

  /*!
   * Returns this \ref TessellatedPath where any arcs are
   * realized as segments. If this \ref TessellatedPath has
   * no arcs, returns this object. If a non-positive value
   * is passed, returns a linearization where arc-segments
   * are tessellated into very few line segments.
   * \param thresh threshhold at which to linearize
   *               arc-segments.
   */
  const TessellatedPath*
  linearization(float thresh) const;

  /*!
   * Provided as a conveniance, returns the starting point tessellation.
   * Equivalent to
   * \code
   * linearization(-1.0f)
   * \endcode
   */
  const TessellatedPath*
  linearization(void) const;

  /*!
   * Returns this \ref TessellatedPath stroked. The \ref
   * StrokedPath object is constructed lazily.
   */
  const reference_counted_ptr<const StrokedPath>&
  stroked(void) const;

  /*!
   * Returns this \ref TessellatedPath filled. If this
   * \ref TessellatedPath has arcs will return
   * the fill associated to the linearization() of
   * this \ref TessellatedPath. If a non-positive value
   * is passed, returns the fill of the linearization
   * where arc-segments are tessellated into very few
   * line segments.
   * \param thresh threshhold at which to linearize
   *               arc-segments.
   */
  const reference_counted_ptr<const FilledPath>&
  filled(float thresh) const;

  /*!
   * Provided as a conveniance, returns the starting point tessellation.
   * Equivalent to
   * \code
   * filled(-1.0f)
   * \endcode
   */
  const reference_counted_ptr<const FilledPath>&
  filled(void) const;

private:
  TessellatedPath(Refiner *p, float threshhold,
                  unsigned int additional_recursion_count);

  TessellatedPath(const TessellatedPath &with_arcs,
                  float thresh);

  void *m_d;
};

/*! @} */

}
