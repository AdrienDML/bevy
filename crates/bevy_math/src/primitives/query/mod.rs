//! This module defines various queries that can be performed on primitives.

use crate::{Vec2, Vec3};
mod dim2;
mod dim3;

/// Allows you to query the distance to a 3d primitive.
/// For primitives with no interior such as Lines arcs and such the signed distance and the
/// distance are the same.
pub trait Distance3d {
    /// Returns the distance to the shape with:
    /// - a distance of zero inside and on the boundary.
    /// - a distance greater than zero outside.
    fn distance(&self, point: Vec3) -> f32;

    /// Returns the distance to the shape with:
    /// - a distance less than zero inside the primitive.
    /// - a distance equal to zero on the boundary.
    /// - a distance greater than zero outside.
    fn signed_distance(&self, point: Vec3) -> f32;
}

/// Allows you to query the distance to a 2d primitive.
/// For primitives with no interior such as Lines arcs and such the signed distance and the
/// distance are the same.
pub trait Distance2d {
    /// Returns the distance to the shape with:
    /// - a distance of zero inside and on the boundary.
    /// - a distance greater than zero outside.
    fn distance(&self, point: Vec2) -> f32;

    /// Returns the distance to the shape with:
    /// - a distance less than zero inside the primitive.
    /// - a distance equal to zero on the boundary.
    /// - a distance greater than zero outside.
    fn signed_distance(&self, point: Vec2) -> f32;
}

/// Allows you to query the distance to a 2d primitive.
/// For primitives with no interior such as Lines, Arcs and such the closest point and the
/// closest point on boundary are the same.
pub trait ClosestPoint2d {
    /// Returns the closest point to the provided on the primitive considering the primitive is
    /// 'filled' meaning if the point is inside the primitive it will return the same point as the
    /// query point.
    fn closest_point(&self, point: Vec2) -> Vec2;

    /// Returns the closest point to the provided on the primitive considering the primitive is
    /// 'holow' meaning the distance to the closest point on the boundary of the primitive.
    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2;
}

/// Allows you to query the distance to a 3d primitive.
/// For primitives with no interior such as Lines, Arcs and such the closest point and the
/// closest point on boundary are the same.
pub trait ClosestPoint3d {
    /// Returns the closest point to the provided on the primitive considering the primitive is
    /// 'filled' meaning if the point is inside the primitive it will return the same point as the
    /// query point.
    fn closest_point(&self, point: Vec3) -> Vec3;

    /// Returns the closest point to the provided on the primitive considering the primitive is
    /// 'holow' meaning the distance to the closest point on the boundary of the primitive.
    fn closest_point_on_boundary(&self, point: Vec3) -> Vec3;
}
