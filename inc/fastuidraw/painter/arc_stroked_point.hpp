/*!
 * \file arc_stroked_point.hpp
 * \brief file arc_stroked_point.hpp
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

#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/util.hpp>
#include <fastuidraw/painter/painter_attribute.hpp>

namespace fastuidraw  {

/*!\addtogroup Paths
 * @{
 */

/*!
 * \brief
 * An ArcStrokedPoint holds the data for a point of stroking
 * where the segments can be arcs of a circle. The upshot
 * is that a fragment shader will determine per-pixel coverage.
 * In addition, the data is so that changing the stroking width
 * or miter limit does not change the stroking data.
 */
class ArcStrokedPoint
{
public:
  /*!
   * \brief
   * Enumeration type to specify how to compute the location
   * of an ArcStrokedPoint.
   */
  enum offset_type_t
    {
      /*!
       * A point of an arc. Indicates the point is part of
       * an arc.
       */
      offset_arc_point,

      /*!
       * The point is part of a line-segment
       */
      offset_line_segment,

      /*!
       * Represents a point at the start or end of an edge.
       * When performing dashed stroking with caps, these
       * points can be expanded into quads to cover a cap
       * induced by stroking at the start or end of an
       * edge.
       */
      offset_arc_point_dashed_capper,

      /*!
       * Number of offset types, not an actual offset type(!).
       */
      number_offset_types,
    };

  /*!
   * \brief
   * Enumeration encoding of bits of \ref m_packed_data
   * common to all offset types.
   */
  enum packed_data_bit_layout_common_t
    {
      /*!
       * Bit0 for holding the offset_type() value
       * of the point.
       */
      offset_type_bit0 = 0,

      /*!
       * number of bits needed to hold the
       * offset_type() value of the point.
       */
      offset_type_num_bits = 2,

      /*!
       * Bit indicates that the point in on the stroking
       * boundary. Points with this bit down have that
       * the vertex position after stroking is the same
       * as \ref m_position.
       */
      boundary_bit = offset_type_bit0 + offset_type_num_bits,

      /*!
       * Bit indicates that point is on the end of a
       * segment.
       */
      end_segment_bit,

      /*!
       * Bit indicates that when doing dashed stroking that
       * the value the distance value is the same for all
       * points of a triangle, i.e. the dashed coverage
       * computation can be done purely from the vertex
       * shader
       */
      distance_constant_on_primitive_bit,

      /*!
       * Bit indicates that the primitive formed is for a join
       */
      join_bit,

      /*!
       * Bit0 for holding the depth() value
       * of the point
       */
      depth_bit0,

      /*!
       * number of bits needed to hold the
       * depth() value of the point.
       */
      depth_num_bits = 20,

      /*!
       * Number of bits used on common packed data
       */
      number_common_bits = depth_bit0 + depth_num_bits,
    };

  /*!
   * Enumeration of the bits of \ref m_packed_data
   * for those with offset type \ref offset_arc_point
   */
  enum packed_data_bit_stroking_boundary_t
    {
      /*!
       * Bit indicates that the point is a beyond stroking
       * boundary bit. These points go beyond the stroking
       * boundary to make sure that the triangles emitted
       * contain the stroked arc.
       */
      beyond_boundary_bit = number_common_bits,

      /*!
       * If bit is up, then point is on the inside
       * stroking boundary, otherwise the point is
       * on the outside stroking boundary.
       */
      inner_stroking_bit,

      /*!
       * When this bit is up and the stroking radius exceeds
       * the arc-radius, the point should be placed at the
       * center of the arc.
       */
      move_to_arc_center_bit,
    };

  /*!
   * Enumeration of the bits of \ref m_packed_data
   * for those with offset type \ref offset_arc_point_dashed_capper
   */
  enum packed_data_bit_arc_point_dashed_capper
    {
      /*!
       * if up, the point is to be moved in the direction
       * of \ref m_data to cover an induced cap.
       */
      extend_bit = number_common_bits,
    };

  /*!
   * \brief
   * Enumeration holding bit masks generated from
   * values in \ref packed_data_bit_layout_common_t.
   */
  enum packed_data_bit_masks_t
    {
      /*!
       * Mask generated for \ref offset_type_bit0 and
       * \ref offset_type_num_bits
       */
      offset_type_mask = FASTUIDRAW_MASK(offset_type_bit0, offset_type_num_bits),

      /*!
       * Mask generated for \ref boundary_bit
       */
      boundary_mask = FASTUIDRAW_MASK(boundary_bit, 1),

      /*!
       * Mask generated for \ref boundary_bit
       */
      beyond_boundary_mask = FASTUIDRAW_MASK(beyond_boundary_bit, 1),

      /*!
       * Mask generated for \ref inner_stroking_bit
       */
      inner_stroking_mask = FASTUIDRAW_MASK(inner_stroking_bit, 1),

      /*!
       * Mask generated for \ref move_to_arc_center_bit
       */
      move_to_arc_center_mask = FASTUIDRAW_MASK(move_to_arc_center_bit, 1),

      /*!
       * Mask generated for \ref end_segment_bit
       */
      end_segment_mask = FASTUIDRAW_MASK(end_segment_bit, 1),

      /*!
       * Mask generated for \ref distance_constant_on_primitive_bit
       */
      distance_constant_on_primitive_mask = FASTUIDRAW_MASK(distance_constant_on_primitive_bit, 1),

      /*!
       * Mask generated for \ref join_bit
       */
      join_mask = FASTUIDRAW_MASK(join_bit, 1),

      /*!
       * Mask generated for \ref extend_bit
       */
      extend_mask = FASTUIDRAW_MASK(extend_bit, 1),

      /*!
       * Mask generated for \ref depth_bit0 and \ref depth_num_bits
       */
      depth_mask = FASTUIDRAW_MASK(depth_bit0, depth_num_bits),
    };

  /*!
   * Give the position of the point on the path.
   */
  vec2 m_position;

  /*!
   * Gives the unit vector in which to push the point.
   * For those points that are arc's the location of
   * the center is always given by
   *   \ref m_position - \ref radius() * \ref m_offset_direction
   */
  vec2 m_offset_direction;

  /*!
   * Meaning of \ref m_data depends on \ref offset_type(). If
   * \ref offset_type() is \ref offset_line_segment, then
   * \ref m_data holds the vector from this point to other
   * point of the line segment. Otherwise, \ref m_data[0]
   * holds the radius of the arc and \ref m_data[1] holds
   * the angle difference from the end of the arc to the
   * start of the arc.
   */
  vec2 m_data;

  /*!
   * Gives the distance of the point from the start
   * of the -edge- on which the point resides.
   */
  float m_distance_from_edge_start;

  /*!
   * Gives the distance of the point from the start
   * of the -contour- on which the point resides.
   */
  float m_distance_from_contour_start;

  /*!
   * Gives the length of the edge on which the
   * point lies. This value is the same for all
   * points along a fixed edge.
   */
  float m_edge_length;

  /*!
   * Gives the length of the contour on which the
   * point lies. This value is the same for all
   * points along a fixed contour.
   */
  float m_contour_length;

  /*!
   * Bit field with data packed.
   */
  uint32_t m_packed_data;

  /*!
   * Provides the point type from a value of \ref m_packed_data.
   * The return value is one of the enumerations of
   * \ref offset_type_t.
   * \param packed_data_value value suitable for \ref m_packed_data
   */
  static
  enum offset_type_t
  offset_type(uint32_t packed_data_value)
  {
    uint32_t v;
    v = unpack_bits(offset_type_bit0, offset_type_num_bits, packed_data_value);
    return static_cast<enum offset_type_t>(v);
  }

  /*!
   * Provides the point type for the point. The return value
   * is one of the enumerations of \ref offset_type_t.
   */
  enum offset_type_t
  offset_type(void) const
  {
    return offset_type(m_packed_data);
  }

  /*!
   * If a point from an arc-segment, gives the radius
   * of the arc, equivalent to \ref m_data[0].
   */
  float
  radius(void) const
  {
    return m_data[0];
  }

  /*!
   * If a point from an arc-segment, gives the radius
   * of the arc, equivalent to \ref m_data[0].
   */
  float&
  radius(void)
  {
    return m_data[0];
  }

  /*!
   * If a point from an arc-segment, gives the angle
   * of the arc, equivalent to \ref m_data[1].
   */
  float
  arc_angle(void) const
  {
    return m_data[1];
  }

  /*!
   * If a point from an arc-segment, gives the angle
   * of the arc, equivalent to \ref m_data[1].
   */
  float&
  arc_angle(void)
  {
    return m_data[1];
  }

  /*!
   * When stroking the data, the depth test to only
   * pass when the depth value is -strictly- larger
   * so that a fixed pixel is not stroked twice by
   * a single path. The value returned by depth() is
   * a relative z-value for a vertex. The points
   * drawn first have the largest z-values.
   */
  uint32_t
  depth(void) const
  {
    return unpack_bits(depth_bit0, depth_num_bits, m_packed_data);
  }

  /*!
   * Set the value returned by depth().
   */
  void
  depth(const uint32_t v)
  {
    m_packed_data &= ~depth_mask;
    m_packed_data |= pack_bits(depth_bit0, depth_num_bits, v);
  }

  /*!
   * Pack the data of this \ref ArcStrokedPoint into a \ref
   * PainterAttribute. The packing is as follows:
   * - PainterAttribute::m_attrib0 .xy -> \ref m_position (float)
   * - PainterAttribute::m_attrib0 .zw -> \ref m_offset_direction (float)
   * - PainterAttribute::m_attrib1 .x  -> \ref m_distance_from_edge_start (float)
   * - PainterAttribute::m_attrib1 .y  -> \ref m_distance_from_contour_start (float)
   * - PainterAttribute::m_attrib1 .zw -> \ref m_data (float)
   * - PainterAttribute::m_attrib2 .x  -> \ref m_packed_data (uint)
   * - PainterAttribute::m_attrib2 .y  -> \ref m_edge_length (float)
   * - PainterAttribute::m_attrib2 .z  -> \ref m_contour_length (float)
   * - PainterAttribute::m_attrib2 .w  (free)
   *
   * \param dst PainterAttribute to which to pack
   */
  void
  pack_point(PainterAttribute *dst) const;

  /*!
   * Unpack an \ref ArcStrokedPoint from a \ref PainterAttribute.
   * \param dst point to which to unpack data
   * \param src PainterAttribute from which to unpack data
   */
  static
  void
  unpack_point(ArcStrokedPoint *dst, const PainterAttribute &src);
};


/*! @} */

}
