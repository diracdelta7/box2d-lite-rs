use crate::collision::FeaturePair;
use crate::collision::arbiter::Contact;
use crate::dynamics::{BodyHandle, World};
use crate::math::{Mat22, Vec2, sign_nonzero};

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3#[repr(u8)]

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Axis {
    FaceAX = 0,
    FaceAY = 1,
    FaceBX = 2,
    FaceBY = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum EdgeNumber {
    NoEdge = 0,
    Edge1 = 1,
    Edge2 = 2,
    Edge3 = 3,
    Edge4 = 4,
}

impl Default for EdgeNumber {
    fn default() -> Self {
        EdgeNumber::NoEdge
    }
}

impl EdgeNumber {
    #[inline]
    pub const fn from_u8(v: u8) -> Self {
        match v {
            0 => EdgeNumber::NoEdge,
            1 => EdgeNumber::Edge1,
            2 => EdgeNumber::Edge2,
            3 => EdgeNumber::Edge3,
            4 => EdgeNumber::Edge4,
            _ => EdgeNumber::NoEdge, // Defensive coding.
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct ClipVertex {
    v: Vec2,
    fp: FeaturePair,
}

pub fn flip(fp: &mut FeaturePair) {
    std::mem::swap(&mut fp.in_edge1, &mut fp.in_edge2);
    std::mem::swap(&mut fp.out_edge1, &mut fp.out_edge2);
}

pub fn clip_segment_to_line(
    v_out: &mut [ClipVertex; 2],
    v_in: &[ClipVertex; 2],
    normal: Vec2,
    offset: f32,
    clip_edge: EdgeNumber,
) -> usize {
    let mut num_out: usize = 0;

    let distance0 = normal.dot(v_in[0].v) - offset;
    let distance1 = normal.dot(v_in[1].v) - offset;

    if distance0 <= 0.0 {
        v_out[num_out] = v_in[0];
        num_out += 1;
    }
    if distance1 <= 0.0 {
        v_out[num_out] = v_in[1];
        num_out += 1;
    }

    if distance0 * distance1 < 0.0 {
        let interp = distance0 / (distance0 - distance1);
        v_out[num_out].v = v_in[0].v + interp * (v_in[1].v - v_in[0].v);
        if distance0 > 0.0 {
            v_out[num_out].fp = v_in[0].fp;
            v_out[num_out].fp.in_edge1 = clip_edge;
            v_out[num_out].fp.in_edge2 = EdgeNumber::NoEdge;
        } else {
            v_out[num_out].fp = v_in[1].fp;
            v_out[num_out].fp.out_edge1 = clip_edge;
            v_out[num_out].fp.out_edge2 = EdgeNumber::NoEdge;
        }

        num_out += 1;
    }

    num_out
}

pub fn compute_incident_edge(
    c: &mut [ClipVertex; 2],
    h: Vec2,
    pos: Vec2,
    rot: Mat22,
    normal: Vec2,
) {
    // The normal is from the reference box. Convert it
    // to the incident boxe's frame and flip sign.
    let rot_t = rot.transpose();
    let n = -(rot_t * normal);
    let n_abs = n.abs();

    if n_abs.x > n_abs.y {
        if sign_nonzero(n.x) > 0.0 {
            c[0].v.set(h.x, -h.y);
            c[0].fp.in_edge2 = EdgeNumber::Edge3;
            c[0].fp.out_edge2 = EdgeNumber::Edge4;

            c[1].v.set(h.x, h.y);
            c[1].fp.in_edge2 = EdgeNumber::Edge4;
            c[1].fp.out_edge2 = EdgeNumber::Edge1;
        } else {
            c[0].v.set(-h.x, h.y);
            c[0].fp.in_edge2 = EdgeNumber::Edge1;
            c[0].fp.out_edge2 = EdgeNumber::Edge2;

            c[1].v.set(-h.x, -h.y);
            c[1].fp.in_edge2 = EdgeNumber::Edge2;
            c[1].fp.out_edge2 = EdgeNumber::Edge3;
        }
    } else {
        if sign_nonzero(n.y) > 0.0 {
            c[0].v.set(h.x, h.y);
            c[0].fp.in_edge2 = EdgeNumber::Edge4;
            c[0].fp.out_edge2 = EdgeNumber::Edge1;

            c[1].v.set(-h.x, h.y);
            c[1].fp.in_edge2 = EdgeNumber::Edge1;
            c[1].fp.out_edge2 = EdgeNumber::Edge2;
        } else {
            c[0].v.set(-h.x, -h.y);
            c[0].fp.in_edge2 = EdgeNumber::Edge2;
            c[0].fp.out_edge2 = EdgeNumber::Edge3;

            c[1].v.set(h.x, -h.y);
            c[1].fp.in_edge2 = EdgeNumber::Edge3;
            c[1].fp.out_edge2 = EdgeNumber::Edge4;
        }
    }

    c[0].v = pos + rot * c[0].v;
    c[1].v = pos + rot * c[1].v;
}

// The normal points from A to B
pub fn collide(
    contacts: &mut [Contact; 2],
    body_a: BodyHandle,
    body_b: BodyHandle,
    world: &World,
) -> usize {
    let body_a = world.body(body_a);
    let body_b = world.body(body_b);

    // Setup
    let ha = 0.5 * body_a.width;
    let hb = 0.5 * body_b.width;

    let pos_a = body_a.position;
    let pos_b = body_b.position;

    let rot_a = Mat22::from_angle(body_a.rotation);
    let rot_b = Mat22::from_angle(body_b.rotation);

    let rot_at = rot_a.transpose();
    let rot_bt = rot_b.transpose();

    let dp = pos_a - pos_b;
    let da = rot_at * dp;
    let db = rot_bt * dp;

    let c = rot_at * rot_b;
    let c_abs = c.abs();
    let ct_abs = c.transpose().abs();

    // Box A faces
    let face_a = da.abs() - ha - c_abs * hb;
    if face_a.x > 0.0 || face_a.y > 0.0 {
        return 0;
    }

    // Box B faces
    let face_b = db.abs() - ct_abs * ha - hb;
    if face_b.x > 0.0 || face_b.y > 0.0 {
        return 0;
    }

    // Find best axis

    // Box A faces
    let mut axis = Axis::FaceAX;
    let mut separation = face_a.x;
    let mut normal = if da.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };

    let relative_tol: f32 = 0.95;
    let absolute_tol: f32 = 0.01;

    if face_a.y > relative_tol * separation + absolute_tol * ha.y {
        axis = Axis::FaceAY;
        separation = face_a.y;
        normal = if da.y > 0.0 { rot_a.col2 } else { -rot_a.col2 };
    }

    // Box B faces
    if face_b.x > relative_tol * separation + absolute_tol * hb.x {
        axis = Axis::FaceBX;
        separation = face_b.x;
        normal = if db.x > 0.0 { rot_b.col1 } else { -rot_b.col1 };
    }

    if face_b.y > relative_tol * separation + absolute_tol * hb.y {
        axis = Axis::FaceBY;
        // separation = face_b.y;
        normal = if db.y > 0.0 { rot_b.col2 } else { -rot_b.col2 };
    }

    // Setup clipping plane data based on the separating axis
    let front_normal: Vec2;
    let side_normal: Vec2;
    let mut incident_edge: [ClipVertex; 2] = [ClipVertex::default(); 2];
    let front: f32;
    let neg_side: f32;
    let pos_side: f32;
    let neg_edge: EdgeNumber;
    let pos_edge: EdgeNumber;

    // Compute the clipping lines and the line segment to be clipped.
    match axis {
        Axis::FaceAX => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + ha.x;
            side_normal = rot_a.col2;
            let side = pos_a.dot(side_normal);
            neg_side = -side + ha.y;
            pos_side = side + ha.y;
            neg_edge = EdgeNumber::Edge3;
            pos_edge = EdgeNumber::Edge1;
            compute_incident_edge(&mut incident_edge, hb, pos_b, rot_b, front_normal);
        }

        Axis::FaceAY => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + ha.y;
            side_normal = rot_a.col1;
            let side = pos_a.dot(side_normal);
            neg_side = -side + ha.x;
            pos_side = side + ha.x;
            neg_edge = EdgeNumber::Edge2;
            pos_edge = EdgeNumber::Edge4;
            compute_incident_edge(&mut incident_edge, hb, pos_b, rot_b, front_normal);
        }
        Axis::FaceBX => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + hb.x;
            side_normal = rot_b.col2;
            let side = pos_b.dot(side_normal);
            neg_side = -side + hb.y;
            pos_side = side + hb.y;
            neg_edge = EdgeNumber::Edge3;
            pos_edge = EdgeNumber::Edge1;
            compute_incident_edge(&mut incident_edge, ha, pos_a, rot_a, front_normal);
        }
        Axis::FaceBY => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + hb.y;
            side_normal = rot_b.col1;
            let side = pos_b.dot(side_normal);
            neg_side = -side + hb.x;
            pos_side = side + hb.x;
            neg_edge = EdgeNumber::Edge2;
            pos_edge = EdgeNumber::Edge4;
            compute_incident_edge(&mut incident_edge, ha, pos_a, rot_a, front_normal);
        }
    }

    // clip other face with 5 box planes (1 face plane, 4 edge planes)

    let mut clip_points1 = [ClipVertex::default(); 2];
    let mut clip_points2 = [ClipVertex::default(); 2];

    // Clip to box side 1
    let mut np = clip_segment_to_line(
        &mut clip_points1,
        &incident_edge,
        -side_normal,
        neg_side,
        neg_edge,
    );

    if np < 2 {
        return 0;
    }

    // Clip to negative box side 1
    np = clip_segment_to_line(
        &mut clip_points2,
        &clip_points1,
        side_normal,
        pos_side,
        pos_edge,
    );

    if np < 2 {
        return 0;
    }

    // Now clipPoints2 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.
    let mut num_contacts: usize = 0;
    for i in 0..2 {
        let sep = front_normal.dot(clip_points2[i].v) - front;

        if sep <= 0.0 {
            let contact = &mut contacts[num_contacts];
            contact.separation = sep;
            contact.normal = normal;
            // slide contact point onto reference face (easy to cull)
            contact.position = clip_points2[i].v - sep * front_normal;
            contact.feature = clip_points2[i].fp;
            if axis == Axis::FaceBX || axis == Axis::FaceBY {
                flip(&mut contacts[num_contacts].feature);
            }
            num_contacts += 1;
        }
    }

    num_contacts
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn edge_number_from_u8_is_defensive() {
        assert_eq!(EdgeNumber::from_u8(0), EdgeNumber::NoEdge);
        assert_eq!(EdgeNumber::from_u8(1), EdgeNumber::Edge1);
        assert_eq!(EdgeNumber::from_u8(2), EdgeNumber::Edge2);
        assert_eq!(EdgeNumber::from_u8(3), EdgeNumber::Edge3);
        assert_eq!(EdgeNumber::from_u8(4), EdgeNumber::Edge4);
        assert_eq!(EdgeNumber::from_u8(200), EdgeNumber::NoEdge);
    }

    #[test]
    fn flip_swaps_edges() {
        let mut fp = FeaturePair::new(
            EdgeNumber::Edge1,
            EdgeNumber::Edge2,
            EdgeNumber::Edge3,
            EdgeNumber::Edge4,
        );
        flip(&mut fp);
        assert_eq!(fp.in_edge1, EdgeNumber::Edge3);
        assert_eq!(fp.out_edge1, EdgeNumber::Edge4);
        assert_eq!(fp.in_edge2, EdgeNumber::Edge1);
        assert_eq!(fp.out_edge2, EdgeNumber::Edge2);
    }

    #[test]
    fn clip_segment_to_line_clips_one_point() {
        // Line: x = 0.5, keep points with x <= 0.5
        let normal = Vec2::new(1.0, 0.0);
        let offset = 0.5;
        let fp0 = FeaturePair::default();
        let fp1 = FeaturePair::default();
        let v_in = [
            ClipVertex {
                v: Vec2::new(0.0, 0.0),
                fp: fp0,
            },
            ClipVertex {
                v: Vec2::new(1.0, 0.0),
                fp: fp1,
            },
        ];
        let mut v_out = [ClipVertex::default(); 2];
        let n = clip_segment_to_line(&mut v_out, &v_in, normal, offset, EdgeNumber::Edge1);

        // One is inside, and one intersection point.
        assert_eq!(n, 2);
        assert_relative_eq!(v_out[0].v.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(v_out[1].v.x, 0.5, epsilon = 1e-6);
        assert_relative_eq!(v_out[1].v.y, 0.0, epsilon = 1e-6);
    }
}
