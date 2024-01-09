#ifndef PCL_NANOFLANN_HPP
#define PCL_NANOFLANN_HPP
// #include <pcl/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nanoflann.hpp>
template <typename PointT>
struct PCLAdaptor {
    using coord_t = float;
    using Derived = pcl::PointCloud<PointT>;
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PCLAdaptor<PointT>>, PCLAdaptor<PointT>, 3>
        KDTree;
    const Derived& obj;

    PCLAdaptor(const Derived& obj_) : obj(obj_) {}
    inline const Derived& derived() const { return obj; }

    // Returns the dim 'th component of the idx' th point in the class :
    // Since this is inlined and the "dim" argument is typically an immediate
    // value, the
    //  "if/else's" are actually solved at compile time.
    inline coord_t kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return derived()[idx].x;
        else if (dim == 1)
            return derived()[idx].y;
        else
            return derived()[idx].z;
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return derived().size(); }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const {
        return false;
    }
};

#endif