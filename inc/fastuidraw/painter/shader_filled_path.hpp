/*!
 * \file filled_path.hpp
 * \brief file filled_path.hpp
 *
 * Copyright 2019 by Intel.
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
#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/c_array.hpp>
#include <fastuidraw/util/rect.hpp>
#include <fastuidraw/util/matrix.hpp>
#include <fastuidraw/util/reference_counted.hpp>
#include <fastuidraw/text/glyph_atlas.hpp>
#include <fastuidraw/text/glyph_render_data.hpp>
#include <fastuidraw/painter/painter_attribute_data.hpp>
#include <fastuidraw/painter/painter_enums.hpp>

namespace fastuidraw {

/*!\addtogroup Paths
 * @{
 */

/*!
 * A ShaderFilledPath represents a path that is drawn as a rectangle
 * where the fragment shader acting on the rectangle performs the
 * coverage computation of each pixel. Generally speaking, one
 * should only fill paths via a ShaderFilledPath when a filled
 * path when drawn has very high density of edges.
 */
class ShaderFilledPath:
  public reference_counted<ShaderFilledPath>::non_concurrent
{
public:
  /*!
   *
   */
  class Builder:fastuidraw::noncopyable
  {
  public:
    Builder(void);
    ~Builder();

    /*!
     * Start a contour. Before starting a new contour
     * the previous contour must be closed by calling
     * line_to() or quadratic_to() connecting to the
     * start point of the previous contour.
     * \param pt start point of the new contour
     */
    void
    move_to(vec2 pt);

    /*!
     * Add a line segment connecting the end point of the
     * last curve or line segment of the current contour to
     * a given point.
     * \param pt end point of the new line segment
     */
    void
    line_to(vec2 pt);

    /*!
     * Add a quadratic curveconnecting the end point of the
     * last curve or line segment of the current contour
     * \param ct control point of the quadratic curve
     * \param pt end point of the quadratic curve
     */
    void
    quadratic_to(vec2 ct, vec2 pt);

    /*!
     * Add a cubic curve connecting the end point of the
     * last curve or line segment of the current contour
     * \param ct0 control point of the cubic curve
     * \param ct1 control point of the cubic curve
     * \param pt end point of the cubic curve
     */
    void
    cubic_to(vec2 ct0, vec2 ct1, vec2 pt);

  private:
    friend class ShaderFilledPath;
    void *m_d;
  };

  explicit
  ShaderFilledPath(const Builder &B);

  ~ShaderFilledPath();

  /*!
   * Returns the \ref PainterAttribute and \ref PainterIndex data
   * to draw the filled path. The attribute data is packed so that
   * it is to be shaded by a \ref PainterGlyphShader.
   * \param glyph_atlas \ref GlyphAtlas to place GPU data
   * \param fill_rule full rule with which to fill the path
   * \param out_attribs location to which to write attributes
   * \param out_indices location to which to write indices
   */
  void
  render_data(const reference_counted_ptr<GlyphAtlas> &glyph_atlas,
              enum PainterEnums::fill_rule_t fill_rule,
              c_array<const PainterAttribute> *out_attribs,
              c_array<const PainterIndex> *out_indices) const;

  /*!
   * Returns the \ref glyph_type of the ShaderFilledPath.
   * Any \ref PainterGlyphShader that accepts the returned
   * \ref glyph_type can then also be used to draw the
   * ShaderFilledPath.
   */
  enum glyph_type
  render_type(void) const;

private:
  void *m_d;
};

/*! @} */

} //namespace f
