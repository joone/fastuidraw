/*!
 * \file image_atlas.hpp
 * \brief file image_atlas.hpp
 *
 * Copyright 2016 by Intel.
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

#include <fastuidraw/util/reference_counted.hpp>
#include <fastuidraw/util/util.hpp>
#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/c_array.hpp>

namespace fastuidraw
{

///@cond
class Image;
class ImageSourceBase;
///@endcond

/*!\addtogroup PainterBackend
 * @{
 */

  /*!
   * \brief
   * Represents the interface for a backing store for color data of images.
   *
   * For example in GL, this can be a GL_TEXTURE_2D_ARRAY. An implementation
   * of the class does NOT need to be thread safe because the user of the
   * backing store (ImageAtlas) performs calls to the backing store behind
   * its own mutex.
   */
  class AtlasColorBackingStoreBase:
    public reference_counted<AtlasColorBackingStoreBase>::default_base
  {
  public:
    /*!
     * Ctor.
     * \param whl provides the dimensions of the AtlasColorBackingStoreBase
     * \param presizable if true the object can be resized to be larger
     */
    AtlasColorBackingStoreBase(ivec3 whl, bool presizable);

    /*!
     * Ctor.
     * \param w width of the backing store
     * \param h height of the backing store
     * \param num_layers number of layers of the backing store
     * \param presizable if true the object can be resized to be larger
     */
    AtlasColorBackingStoreBase(int w, int h, int num_layers, bool presizable);

    virtual
    ~AtlasColorBackingStoreBase();

    /*!
     * To be implemented by a derived class to set color data into the backing store.
     * \param mimap_level what mipmap level
     * \param dst_xy x and y coordinates of location to place data in the atlas
     * \param dst_l layer of position to place data in the atlas
     * \param src_xy x and y coordinates from which to take data from the ImageSourceBase
     * \param size width and height of region to copy into the backing store.
     * \param data ImageSourceBase from which to src data
     */
    virtual
    void
    set_data(int mimap_level, ivec2 dst_xy, int dst_l, ivec2 src_xy,
             unsigned int size, const ImageSourceBase &data) = 0;

    /*!
     * To be implemented by a derived class to set color data into the backing store.
     * \param mimap_level what mipmap level
     * \param dst_xy x and y coordinates of location to place data in the atlas
     * \param dst_l layer of position to place data in the atlas
     * \param size width and height of region to copy into the backing store.
     * \param color_value value with which to fill EVERY texel of the
     */
    virtual
    void
    set_data(int mimap_level, ivec2 dst_xy, int dst_l, unsigned int size, u8vec4 color_value) = 0;

    /*!
     * To be implemented by a derived class
     * to flush set_data() to the backing
     * store.
     */
    virtual
    void
    flush(void) = 0;

    /*!
     * Returns the dimensions of the backing store
     * (as passed in the ctor).
     */
    ivec3
    dimensions(void) const;

    /*!
     * Returns true if and only if this object can be
     * resized to a larger size.
     */
    bool
    resizeable(void) const;

    /*!
     * Resize the object by increasing the number of layers.
     * The routine resizeable() must return true, if not
     * the function FASTUIDRAWasserts.
     */
    void
    resize(int new_num_layers);

  protected:
    /*!
     * To be implemented by a derived class to resize the
     * object. The resize changes ONLY the number of layers
     * of the object and only increases the value as well.
     * When called, the return value of dimensions() is
     * the size before the resize completes.
     * \param new_num_layers new number of layers to which
     *                       to resize the underlying store.
     */
    virtual
    void
    resize_implement(int new_num_layers) = 0;

  private:
    void *m_d;
  };

  /*!
   * \brief
   * Represents the interface for the backing store for index data of images.
   *
   * For example in GL, this can be a GL_TEXTURE_2D_ARRAY. An implementation
   * of the class does NOT need to be thread safe because the user of the
   * backing store (ImageAtlas) performs calls to the backing store behind
   * its own mutex.
   */
  class AtlasIndexBackingStoreBase:
    public reference_counted<AtlasIndexBackingStoreBase>::default_base
  {
  public:
    /*!
     * Ctor.
     * \param whl dimensions of the backing store
     * \param presizable if true the object can be resized to be larger
     */
    AtlasIndexBackingStoreBase(ivec3 whl, bool presizable);

    /*!
     * Ctor.
     * \param w width of the backing store
     * \param h height of the backing store
     * \param l number layers of the backing store
     * \param presizable if true the object can be resized to be larger
     */
    AtlasIndexBackingStoreBase(int w, int h, int l, bool presizable);

    virtual
    ~AtlasIndexBackingStoreBase();

    /*!
     * To be implemented by a derived class to fill index data into the
     * backing store. The index data passed references into a
     * AtlasColorBackingStore.
     * \param x horizontal position
     * \param y vertical position
     * \param l layer position
     * \param w width of data
     * \param h height of data
     * \param data list of tiles as returned by ImageAtlas::add_color_tile()
     * \param slack amount of pixels duplicated on each boundary, in particular
     *              the actual color data "starts" slack pixels into the color
     *              tile and "ends" slack pixels before. The purpose of the
     *              slack is to allow to sample outside of the tile, for example
     *              to do bilinear filtering (requires slack of 1) or cubic
     *              filtering (require slack of 2). The slack value is to be
     *              used by an implementation to help compute the actaul texel
     *              values to store in the underlying texture.
     * \param c AtlasColorBackingStoreBase into which to index
     * \param color_tile_size size of tiles on C
     */
    virtual
    void
    set_data(int x, int y, int l,
             int w, int h,
             c_array<const ivec3> data,
             int slack,
             const AtlasColorBackingStoreBase *c,
             int color_tile_size) = 0;

    /*!
     * To be implemented by a derived class to fill index data into the
     * backing store. The index data passed references back into this
     * AtlasIndexBackingStore.
     * \param x horizontal position
     * \param y vertical position
     * \param l layer position
     * \param w width of data
     * \param h height of data
     * \param data list of tiles as returned by ImageAtlas::add_index_tile()
     */
    virtual
    void
    set_data(int x, int y, int l, int w, int h,
             c_array<const ivec3> data) = 0;

    /*!
     * To be implemented by a derived class
     * to flush set_data() to the backing
     * store.
     */
    virtual
    void
    flush(void) = 0;

    /*!
     * Returns the dimensions of the backing store
     * (as passed in the ctor).
     */
    ivec3
    dimensions(void) const;

    /*!
     * Returns true if and only if this object can be
     * resized to a larger size.
     */
    bool
    resizeable(void) const;

    /*!
     * Resize the object by increasing the number of layers.
     * The routine resizeable() must return true, if not
     * the function FASTUIDRAWasserts.
     */
    void
    resize(int new_num_layers);

  protected:
    /*!
     * To be implemented by a derived class to resize the
     * object. The resize changes ONLY the number of layers
     * of the object and only increases the value as well.
     * When called, the return value of dimensions() is
     * the size before the resize completes.
     * \param new_num_layers new number of layers to which
     *                       to resize the underlying store.
     */
    virtual
    void
    resize_implement(int new_num_layers) = 0;

  private:
    void *m_d;
  };
/*! @} */

/*!\addtogroup Imaging
 * @{
 */

  /*!
   * \brief
   * An ImageAtlas is a common location to place images of an application.
   *
   * Ideally, all images are placed into a single ImageAtlas (changes of
   * ImageAtlas force draw-call breaks). Methods of ImageAtlas are
   * thread safe, locked behind a mutex of the ImageAtlas.
   */
  class ImageAtlas:
    public reference_counted<ImageAtlas>::default_base
  {
  public:
    /*!
     * Class representing an action to execute when a bindless
     * image is deleted. The action is gauranteed to be executed
     * AFTER the 3D API is no longer using the resources that
     * back the image.
     */
    class ResourceReleaseAction:
      public reference_counted<ResourceReleaseAction>::default_base
    {
    public:
      /*!
       * To be implemented by a derived class to perform resource
       * release actions.
       */
      virtual
      void
      action(void) = 0;
    };

    /*!
     * Ctor.
     * \param pcolor_tile_size size of each color tile
     * \param pindex_tile_size size of each index tile
     * \param pcolor_store color data backing store for atlas, the width and
     *                     height of the backing store must be divisible by
     *                     pcolor_tile_size.
     * \param pindex_store index backing store for atlas, the width and
     *                     height of the backing store must be divisible by
     *                     pindex_tile_size.
     */
    ImageAtlas(int pcolor_tile_size, int pindex_tile_size,
               reference_counted_ptr<AtlasColorBackingStoreBase> pcolor_store,
               reference_counted_ptr<AtlasIndexBackingStoreBase> pindex_store);

    virtual
    ~ImageAtlas();

    /*!
     * Returns the size (in texels) used for the index tiles.
     */
    int
    index_tile_size(void) const;

    /*!
     * Returns the size (in texels) used for the color tiles.
     */
    int
    color_tile_size(void) const;

    /*!
     * Calls AtlasIndexBackingStoreBase::flush() on
     * the index backing store (see index_store())
     * and AtlasColorBackingStoreBase::flush() on
     * the color backing store (see color_store()).
     */
    void
    flush(void) const;

    /*!
     * Returns a handle to the backing store for the image data.
     */
    const reference_counted_ptr<const AtlasColorBackingStoreBase>&
    color_store(void) const;

    /*!
     * Returns a handle to the backing store for the index data.
     */
    const reference_counted_ptr<const AtlasIndexBackingStoreBase>&
    index_store(void) const;

    /*!
     * Returns true if and only if the backing stores of the atlas
     * can be increased in size.
     */
    bool
    resizeable(void) const;

    /*!
     * Increments an internal counter. If this internal
     * counter is greater than zero, then the reurning
     * of tiles to the free store for later use is
     * -delayed- until the counter reaches zero again
     * (see unlock_resources()). The use case is for
     * buffered painting where the GPU calls are delayed
     * for later (to batch commands) and an Image may go
     * out of scope before the GPU commands are sent to
     * the GPU. By delaying the return of an Image's
     * tiles to the freestore, the image data is valid
     * still for rendering.
     */
    void
    lock_resources(void);

    /*!
     * Decrements an internal counter. If this internal
     * counter reaches zero, those tiles from Image's
     * that were deleted while the counter was non-zero,
     * are then returned to the tile free store. See
     * lock_resources() for more details.
     */
    void
    unlock_resources(void);

    /*!
     * Returns the number of index color tiles that are available
     * in the atlas without resizing the AtlasIndexBackingStoreBase
     * of the ImageAtlas.
     */
    int
    number_free_index_tiles(void) const;

    /*!
     * Adds an index tile that indexes into color data
     * \param data array of tiles as returned by add_color_tile()
     * \param slack amount of pixels duplicated on each boundary, in particular
     *              the actual color data "starts" slack pixels into the color
     *              tile and "ends" slack pixels before. The purpose of the
     *              slack is to allow to sample outside of the tile, for example
     *              to do bilinear filtering (requires slack of 1) or cubic
     *              filtering (require slack of 2).
     */
    ivec3
    add_index_tile(c_array<const ivec3> data, int slack);

    /*!
     * Adds an index tile that indexes into the index data. This is needed
     * for large images where more than one level of index look up is
     * needed to access the image
     * \param data array of tiles as returned by add_index_tile()
     */
    ivec3
    add_index_tile_index_data(c_array<const ivec3> data);

    /*!
     * Mark a tile as free in the atlas
     * \param tile tile to free as returned by add_index_tile().
     */
    void
    delete_index_tile(ivec3 tile);

    /*!
     * Adds a tile to the atlas returning the location
     * (in pixels) of the tile in the backing store
     * of the atlas.
     * \param src_xy location from ImageSourceBase to take data
     * \param image_data image data to which to set the tile
     */
    ivec3
    add_color_tile(ivec2 src_xy, const ImageSourceBase &image_data);

    /*!
     * Adds a tile of a constant color to the atlas returning
     * the location (in pixels) of the tile in the backing store
     * of the atlas.
     * \param color_data color value to which to set all pixels of
     *                   the tile
     */
    ivec3
    add_color_tile(u8vec4 color_data);

    /*!
     * Mark a tile as free in the atlas
     * \param tile tile to free as returned by add_color_tile().
     */
    void
    delete_color_tile(ivec3 tile);

    /*!
     * Returns the number of free color tiles that are available
     * in the atlas without resizing the AtlasColorBackingStoreBase
     * of the ImageAtlas.
     */
    int
    number_free_color_tiles(void) const;

    /*!
     * Resize the color and image backing stores so that
     * the given number of color and index tiles can also
     * be added to the atlas without needing to free any
     * tiles.
     * \param num_color_tiles number of color tiles
     * \param num_index_tiles number of index tiles
     */
    void
    resize_to_fit(int num_color_tiles, int num_index_tiles);

    /*!
     * Queue a ResourceReleaseAction to be executed when resources are
     * not locked down, see lock_resources() and unlock_resources().
     */
    void
    queue_resource_release_action(const reference_counted_ptr<ResourceReleaseAction> &action);

  private:
    void *m_d;
  };

/*! @} */

} //namespace
