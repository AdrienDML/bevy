use std::{f32::consts::FRAC_PI_2, f64::consts::FRAC_PI_2};

use glam::Vec2Swizzles;
use rand_distr::num_traits::Float;

use crate::{
    primitives::{
        Annulus, Arc2d, Capsule2d, Circle, CircularSector, CircularSegment, Line2d, Plane2d,
        Rectangle, Rhombus, Segment2d, Triangle2d,
    },
    Dir2, NormedVectorSpace, ShapeSample, Vec2, VectorSpace,
};

use super::{ClosestPoint2d, Distance2d};

// - [x] Circle
// - [x] Line2d
// - [x] Arc2d
// - [x] CircularSector
// - [x] Annulus
// - [x] Rectangle
// - [x] Plane2d
// - [ ] CircularSegment
// - [ ] Ellipse
// - [ ] Rhombus
// - [ ] Segment2d
// - [ ] Polyline2d
// - [ ] BoxedPolyline2d
// - [ ] Triangle2d
// - [ ] Polygon
// - [ ] BoxedPolygon
// - [ ] RegularPolygon
// - [x] Capsule2d

impl Distance2d for Circle {
    fn distance(&self, point: Vec2) -> f32 {
        let l = point.length_squared();
        if l > self.radius.powi(2) {
            l.sqrt() - self.radius
        } else {
            0.0
        }
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        point.length() - self.radius
    }
}

impl ClosestPoint2d for Circle {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        let l = point.length_squared();
        if l > self.radius.powi(2) {
            point / l.sqrt() * self.radius
        } else {
            point
        }
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        point.normalize() * self.radius
    }
}

impl Distance2d for Line2d {
    fn distance(&self, point: Vec2) -> f32 {
        point.perp_dot(*self.direction).abs()
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        point.perp_dot(*self.direction).abs()
    }
}

impl ClosestPoint2d for Line2d {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        point.project_onto(*self.direction)
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        point.project_onto(*self.direction)
    }
}

impl Distance2d for Arc2d {
    fn distance(&self, point: Vec2) -> f32 {
        let Arc2d { radius, half_angle } = *self;
        let half_angle = FRAC_PI_2 - half_angle;
        let p = Vec2 {
            x: point.x.abs(),
            ..point
        };
        let angle = point.y.atan2(point.x);
        if angle < half_angle {
            (point.length() - radius).abs()
        } else {
            point.distance(self.right_endpoint())
        }
    }

    // As Arc2d doesn't have an interior the signed distance and the distance are the same.
    fn signed_distance(&self, point: Vec2) -> f32 {
        self.distance(point)
    }
}

impl ClosestPoint2d for Arc2d {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        self.closest_point_on_boundary(point)
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        let Arc2d { radius, half_angle } = *self;
        let x_sign = point.x.signum();
        let point = Vec2 {
            x: point.x * x_sign,
            ..point
        };
        let half_angle = FRAC_PI_2 - half_angle;
        let angle = point.y.atan2(point.x).clamp(0.0, half_angle);
        radius * Vec2::from_angle(angle) * Vec2::new(x_sign, 1.0)
    }
}

impl Distance2d for CircularSector {
    fn distance(&self, point: Vec2) -> f32 {
        let point = Vec2 {
            x: point.x.abs(),
            ..point
        };
        let l = point.length_squared();
        let angle = point.y.atan2(point.x);
        let CircularSector {
            arc: Arc2d { radius, half_angle },
        } = *self;
        if angle > half_angle {
            (Vec2::from_angle(FRAC_PI_2 - half_angle) * l.sqrt().min(radius)).distance(point)
        } else if l > radius.powi(2) {
            l.sqrt() - radius
        } else {
            0.0
        }
    }

    // Function picked from inigo Quillez
    fn signed_distance(&self, point: Vec2) -> f32 {
        let CircularSector {
            arc: Arc2d { radius, half_angle },
        } = *self;
        let point = Vec2 {
            x: point.x.abs(),
            ..point
        };
        let l = point.length() - radius;
        let sc = {
            let half_angle = FRAC_PI_2 - half_angle;
            let (s, c) = half_angle.sin_cos();
            Vec2::new(s, c)
        };
        let m = point.distance(sc * point.dot(sc).clamp(0.0, radius));
        l.max(m * sc.perp_dot(point).signum())
    }
}

impl ClosestPoint2d for CircularSector {
    fn closest_point(&self, mut point: Vec2) -> Vec2 {
        let CircularSector {
            arc: Arc2d {
                radius: r,
                half_angle: ha,
            },
        } = *self;
        let mut l = point.length();
        let sx = Vec2::new(point.x.signum(), 1.0);
        point.x = point.x.abs();
        let a = point.x.atan2(point.y); // angle relative to +Y.
        if l > r {
            l = r;
        }
        if a > ha {
            Vec2::from(ha.sin_cos()) * ((a - ha).cos() * l) * sx
        } else {
            Vec2::from(a.sin_cos()) * l * sx
        }
    }

    fn closest_point_on_boundary(&self, mut point: Vec2) -> Vec2 {
        let CircularSector {
            arc: Arc2d {
                radius: r,
                half_angle: ha,
            },
        } = *self;

        let sx = Vec2::new(point.x.signum(), 1.0);
        point.x = point.x.abs();
        let l = point.length();
        let a = point.x.atan2(point.y); // angle relative to +Y.
        let dr = l - r;
        let dl = if (a - ha).cos() < 0.0 {
            -0.5 * r
        } else {
            l * (a - ha).sin()
        };

        if dr > dl {
            Vec2::new(a.sin(), a.cos()) * r * sx
        } else {
            let d = Vec2::from_angle(FRAC_PI_2 - ha);
            d * d.dot(point).clamp(0.0, r) * sx
        }
    }
}

impl Distance2d for Rectangle {
    fn distance(&self, point: Vec2) -> f32 {
        let d = point.abs() - self.half_size;
        match (d.x < 0.0, d.y < 0.0) {
            (true, true) => 0.0,
            (false, true) => d.x,
            (true, false) => d.y,
            (false, false) => d.length(),
        }
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        let d = point.abs() - self.half_size;
        match (d.x < 0.0, d.y < 0.0) {
            (true, true) => d.max_element(),
            (false, true) => d.x,
            (true, false) => d.y,
            (false, false) => d.length(),
        }
    }
}

impl ClosestPoint2d for Rectangle {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        point.clamp(-self.half_size, self.half_size)
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        let s = point.signum();
        let p = point.abs();
        let d = p - self.half_size;
        let p = match (d.x < 0.0, d.y < 0.0) {
            (true, true) => {
                if d.x > d.y {
                    Vec2 {
                        x: self.half_size.x,
                        ..p
                    }
                } else {
                    Vec2 {
                        y: self.half_size.y,
                        ..p
                    }
                }
            }
            (false, true) => Vec2 {
                y: self.half_size.y,
                ..p
            },
            (true, false) => Vec2 {
                x: self.half_size.x,
                ..p
            },
            (false, false) => self.half_size,
        };
        p * s
    }
}

impl Distance2d for Plane2d {
    fn distance(&self, point: Vec2) -> f32 {
        self.normal.dot(point).abs()
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        self.normal.dot(point)
    }
}

impl ClosestPoint2d for Plane2d {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        let sd = self.signed_distance(point);
        if sd < 0.0 {
            point
        } else {
            point - sd * self.normal
        }
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        point.reject_from_normalized(*self.normal)
    }
}

impl Distance2d for Segment2d {
    fn distance(&self, point: Vec2) -> f32 {
        let p1 = point - self.point1();
        let p2 = point - self.point2();
        let h = (p1.dot(p2) / p2.length_squared()).clamp(0.0, 1.0);
        return p1.distance(p2 * h);
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        self.distance(point)
    }
}

impl ClosestPoint2d for Segment2d {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        let Segment2d {
            direction,
            half_length,
        } = *self;
        let t = direction.dot(point);
        t.clamp(-half_length, half_length) * direction
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        self.closest_point(point)
    }
}

impl Distance2d for Capsule2d {
    fn distance(&self, point: Vec2) -> f32 {
        self.signed_distance(point).max(0.0)
    }

    fn signed_distance(&self, mut point: Vec2) -> f32 {
        point = point.abs();
        if point.y < self.half_length {
            point.x - self.radius
        } else {
            point.y -= self.half_length;
            point.length() - self.radius
        }
    }
}

impl ClosestPoint2d for Capsule2d {
    fn closest_point(&self, mut point: Vec2) -> Vec2 {
        let s = point.signum();
        point = point.abs();
        if point.y < self.half_length {
            point.x = point.x.min(self.radius);
        } else {
            point.y -= self.half_length;
            let new_l = {
                let l = point.length();
                if l < self.radius {
                    1.0
                } else {
                    self.radius / l
                }
            };
            point = point * new_l;
        }
        point * s
    }

    fn closest_point_on_boundary(&self, mut point: Vec2) -> Vec2 {
        let s = point.signum();
        point = point.abs();
        if point.y < self.half_length {
            point.x = self.radius;
            point * s
        } else {
            point.y -= self.half_length;
            (point.normalize() * self.radius + self.half_length) * s
        }
    }
}

impl Distance2d for Annulus {
    fn distance(&self, point: Vec2) -> f32 {
        self.inner_circle.signed_distance(point).max(0.0)
            + self.outer_circle.signed_distance(point).max(0.0)
    }

    fn signed_distance(&self, point: Vec2) -> f32 {
        (point.length() - (self.inner_circle.radius + self.outer_circle.radius) * 0.5).abs()
            - self.thickness() * 0.5
    }
}

impl ClosestPoint2d for Annulus {
    fn closest_point(&self, point: Vec2) -> Vec2 {
        // TODO Pick a random direction if point is ZERO.
        let (d, mut l) = Dir2::new_and_length(point).unwrap();
        l = l.clamp(self.inner_circle.radius, self.outer_circle.radius);
        *d * l
    }

    fn closest_point_on_boundary(&self, point: Vec2) -> Vec2 {
        // TODO Pick a random direction if point is ZERO.
        let (d, l) = Dir2::new_and_length(point).unwrap();
        if l < (self.inner_circle.radius + self.outer_circle.radius) * 0.5 {
            d * self.inner_circle.radius
        } else {
            d * self.outer_circle.radius
        }
    }
}

