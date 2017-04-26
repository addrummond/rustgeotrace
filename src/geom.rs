use std::collections::HashSet;
use nalgebra::Vector2 as Vector2_;
use nalgebra::Point2 as Point2_;
use nalgebra;
use std::f64;
use std::f64::consts;

//
// Basic types.
//

pub type Scalar = f64;
pub type Vector2 = Vector2_<Scalar>;
pub type Point2 = Point2_<Scalar>;

const EPSILON: Scalar = 0.0001;

//
// Basic geometric primitive.
//

#[derive(Copy, Clone, Debug)]
pub struct Segment {
    // p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y)
    pub p1: Point2,
    pub p2: Point2
}

//
// Utilities for constructing sequences of segments that approximate various
// bits of geometry. Note that these functions do not need to be particularly
// efficient as they are just used for constructing geometry, and are not
// called during ray tracing.
//

fn ang(v: &Vector2) -> Scalar {
    v.data[1].atan2(v.data[0])
}

pub fn arc_to_segments(center: Point2, start: Point2, end: Point2, n_segs: usize, from: i32, to: i32)
-> Vec<Segment> {
    assert!(n_segs > 0 && to <= n_segs as i32);

    let l1 = start - center;
    let l2 = end - center;

    let rad = nalgebra::distance(&center, &start);

    let a1 = ang(&l1);
    let a2 = ang(&l2);
    let mut angle = a2 - a1;
    if angle < 0.0
        { angle = 2.0*consts::PI + angle; }
    angle = 2.0*consts::PI - angle;

    let nsf = n_segs as Scalar;
    let ai = angle / nsf;
    let mut segments: Vec<Segment> = Vec::new();
    let mut current_point = Point2::new(a1.cos() * rad, a1.sin() * rad);
    let f = if from < 0 { 1 } else { from as usize };
    let t = if to < 0 { n_segs + 1 } else { to as usize };
    for i in f..(t+1) {
        let nf = i as Scalar;
        let a = a1 - (ai * nf);
        let y = a.sin() * rad;
        let x = a.cos() * rad;
        let to = Point2::new(x, y);
        segments.push(Segment { p1: current_point, p2: to });
        current_point = to;
    }

    segments
}

//
// QTrees
//

#[derive(Debug, Copy, Clone)]
pub struct Ray {
    // Origin at p1, pointing in direction of p2.
    pub p1: Point2,
    pub p2: Point2
}

pub fn seg(x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar) -> Segment {
    let p1: Point2;
    let p2: Point2;

    if x1 < x2 || (x1 == x2 && y1 < y2) {
        p1 = Point2::new(x1, y1);
        p2 = Point2::new(x2, y2);
    }
    else {
        p2 = Point2::new(x1, y1);
        p1 = Point2::new(x2, y2);
    }

    Segment {
        p1: p1,
        p2: p2
    }
}

pub fn ray(x1: Scalar, y1: Scalar, x2: Scalar, y2: Scalar) -> Ray {
    Ray {
        p1: Point2::new(x1, y1),
        p2: Point2::new(x2, y2)
    }
}

const QTREE_BIN_SIZE : usize = 8;

#[derive(Debug)]
pub struct QTreeChildInfo<'a,SI>
where SI: 'a + Copy {
    pub center: Point2,
    pub children: [Box<QTreeNode<'a,SI>>; 4] // Clockwise from NW
}

#[derive(Debug)]
pub struct QTreeNode<'a, SI>
where SI: 'a + Copy {
    pub segments: Vec<(&'a Segment, SI)>,
    pub child_info: Option<QTreeChildInfo<'a,SI>>
}

#[derive(Debug)]
pub struct QTree<'a, SI>
where SI: 'a + Copy {
    root: Box<QTreeNode<'a, SI>>
}

fn get_point_quad(p: Point2, c: Point2) -> i32 {
    if p.coords[0] <= c.coords[0] {
        if p.coords[1] <= c.coords[1]
            { 3 }
        else
            { 0 }
    }
    else {
        if p.coords[1] <= c.coords[1]
            { 2 }
        else
            { 1 }
    }
}

fn get_segment_quad(segment: &Segment, c: Point2) -> i32 {
    let q1 = get_point_quad(segment.p1, c);
    let q2 = get_point_quad(segment.p2, c);
    if q1 != q2 {
        -1
    }
    else {
        q1
    }
}

pub struct QTreeInOrderIterator<'a, SI>
where SI: 'a + Copy {
    stack: Vec<(usize, &'a QTreeNode<'a,SI>)>,
}

impl<'a, SI> Iterator for QTreeInOrderIterator<'a, SI>
where SI: 'a + Copy
{
    type Item = (usize, &'a QTreeNode<'a,SI>);

    fn next(&mut self) -> Option<Self::Item> {
        match self.stack.pop() {
            None => { None },
            Some((depth, t)) => {
                if let Some(ref x) = t.child_info {
                    self.stack.extend(x.children.iter().map(|x| (depth+1, &**x)));
                }
                Some((depth, t))
            }
        }
    }
}

fn ray_intersects_segment(ray: &Ray, segment: &Segment) -> Option<Point2> {
    //println!("TESTINTER RAY {} {} {} {}", segment.p1.coords[0],segment.p1.coords[1],segment.p2.coords[0],segment.p2.coords[1]);

    let ray_slope_num = ray.p2.coords[1] - ray.p1.coords[1];
    let seg_slope_num = segment.p2.coords[1] - segment.p1.coords[1];
    let ray_slope_denom = ray.p2.coords[0] - ray.p1.coords[0];
    let seg_slope_denom = segment.p2.coords[0] - segment.p1.coords[0];

    if ray_slope_num == 0.0 && seg_slope_num == 0.0 {
        // Parallel horizontal lines.
        // Does not count as an intersection for purposes of ray tracing,
        // even if the lines overlap.
        return None;
    }
    else if ray_slope_denom == 0.0 && seg_slope_denom == 0.0 {
        // Parallel vertical lines.
        return None;
    }

    let ray_slope = ray_slope_num / ray_slope_denom;
    let seg_slope = seg_slope_num / seg_slope_denom;

    if ray_slope == seg_slope {
        // Parallel lines.
        return None;
    }

    // Calculate intersection point;
    let x;
    let y;
    if ray_slope_denom == 0.0 {
        // We can quickly determine that there's no intersection if the ray is vertical
        // and the segment doesn't overlap the relevant y coordinate.
        if ! (ray.p1.coords[0] >= segment.p1.coords[0] && ray.p1.coords[0] <= segment.p2.coords[0]) ||
             (ray.p1.coords[0] <= segment.p1.coords[0] && ray.p1.coords[0] >= segment.p2.coords[0]) {
            return None;
        }
        x = ray.p1.coords[0];
        let seg_k = segment.p1.coords[1] - (seg_slope * segment.p1.coords[0]);
        y = (seg_slope * x) + seg_k;
    }
    else if seg_slope_denom == 0.0 {
        x = segment.p1.coords[0];
        let ray_k = ray.p1.coords[1] - (ray_slope * ray.p1.coords[0]);
        y = (ray_slope * x) + ray_k;
    }
    else {
        let ray_k = ray.p1.coords[1] - (ray_slope * ray.p1.coords[0]);
        let seg_k = segment.p1.coords[1] - (seg_slope * segment.p1.coords[0]);

        x = (seg_k - ray_k) / (ray_slope - seg_slope);
        y = (seg_k*ray_slope - ray_k*seg_slope) / (ray_slope - seg_slope);
    }

    //println!("CALC: {} {}", x, y);

    // Is the intersection point on the ray?
    if ray_slope_num > 0.0 && y < ray.p1.coords[1]
        { return None; }
    else if ray_slope_num < 0.0 && y > ray.p1.coords[1]
        { return None; }
    if ray_slope_denom > 0.0 && x < ray.p1.coords[0]
        { return None; }
    else if ray_slope_denom < 0.0 && x > ray.p1.coords[0]
        { return None; }
    
    // It's on the ray. Is it on the segment?
    // Because of the ordering of segment points, we know that
    // the x value of the first point <= the x value of the second point.
    if x >= segment.p1.coords[0] - EPSILON && x <= segment.p2.coords[0] + EPSILON &&
       ((y >= segment.p1.coords[1] - EPSILON && y <= segment.p2.coords[1] + EPSILON) ||
        (y <= segment.p1.coords[1] + EPSILON && y >= segment.p2.coords[1] - EPSILON)) {
        return Some(Point2::new(x, y));
    }
    else {
        //println!("ULTIFAIL {} {} {} {}", segment.p1.coords[0], segment.p1.coords[1], segment.p2.coords[0], segment.p2.coords[1]);
        return None;
    }
}

pub struct CollimatedBeamRayIterator {
    p1: Point2,
    x: Scalar,
    y: Scalar,
    normal: Vector2,
    i: usize,
    n_rays: usize
}

impl CollimatedBeamRayIterator {
    pub fn new(p1: Point2, p2: Point2, shiny_side_is_left: bool, n_rays: usize)
    -> CollimatedBeamRayIterator {
        let x = p2.coords[0] - p1.coords[0];
        let y = p2.coords[1] - p1.coords[1];

        // The left normal (looking "along" the line)
        let mut nx = -y;
        let mut ny = x;

        if !shiny_side_is_left {
            nx = -nx;
            ny = -ny;
        }

        let n_rays_s = n_rays as Scalar;

        CollimatedBeamRayIterator {
            p1: p1,
            x: x / n_rays_s,
            y: y / n_rays_s,
            normal: Vector2::new(nx, ny),
            i: 0,
            n_rays: n_rays
        }
    }
}

impl Iterator for CollimatedBeamRayIterator {
    type Item = (Point2, Point2);

    fn next(&mut self) -> Option<Self::Item> {
        if self.i >= self.n_rays
            { return None; }
        
        let ii = self.i as Scalar;

        let mut origin = self.p1;
        origin.coords[0] += ii * self.x;
        origin.coords[1] += ii * self.y;

        let dest = origin + self.normal;

        self.i += 1;

        Some((origin, dest))
    }
}

pub struct QTreeRayTraceIterator<'a, 'b, SI>
where SI: 'a + Copy {
    ray: &'b Ray,
    ray_m: Scalar,
    ray_k: Scalar,
    stack: Vec<(bool, &'a QTreeNode<'a,SI>)>
}

impl<'a,'b, SI> Iterator for QTreeRayTraceIterator<'a, 'b, SI>
where SI: 'a + Copy {
    type Item = &'a Vec<(&'a Segment, SI)>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(&(already, r)) = self.stack.last() {
            if !already {
                let i = self.stack.len() - 1;
                self.stack[i].0 = true;
                return Some(&r.segments);
            }

            self.stack.pop();

            if let Some(ref child_info) = r.child_info {
                // The ray starts from p1, so at least the quad
                // that q1 is in should be added to the mask.
                let ref center = child_info.center;
                let mut quad_mask = 1 << get_point_quad(self.ray.p1, *center);

                if self.ray.p1.coords[1] != self.ray.p2.coords[1] {
                    //println!("FIRST TEST {} {} {}", child_info.center.coords[1], k, m);
                    let x_intercept = (child_info.center.coords[1] - self.ray_k) / self.ray_m;
                    let s1 = self.ray.p2.coords[1] - self.ray.p1.coords[1] >= 0.0;
                    let s2 = child_info.center.coords[1] - self.ray.p1.coords[1] >= 0.0;
                    //println!("CRUCIAL {} {} {}", s1, s2, x_intercept);
                    if s1 == s2 {
                        if x_intercept > child_info.center.coords[0] {
                            quad_mask |= 0b0110;
                        }
                        else {
                            //println!("OH!");
                            quad_mask |= 0b1001;
                        }
                    }
                }

                if self.ray.p1.coords[0] != self.ray.p2.coords[0] {
                    let y_intercept = (self.ray_m * child_info.center.coords[0]) + self.ray_k;
                    let s1 = self.ray.p2.coords[0] - self.ray.p1.coords[0] >= 0.0;
                    let s2 = child_info.center.coords[0] - self.ray.p1.coords[0] >= 0.0;
                    //println!("SECOND TEST {} {} {}", y_intercept, s1, s2);
                    if s1 == s2 {
                        if y_intercept > child_info.center.coords[1] {
                            quad_mask |= 0b0011;
                        }
                        else {
                            quad_mask |= 0b1100;
                        }
                    }
                }

                for i in 0..4 {
                    if quad_mask & (1 << i) != 0 {
                        //println!("PUSHING [{}] {}", quad_mask, i);
                        self.stack.push((false,&(child_info.children[i])));
                    }
                }
            }
        }

        None
    }
}

// -1 left, 0 on line, 1 right (looking from first point to second point).
pub fn point_side_of_line_segment(lp1: Point2, lp2: Point2, p: Point2) -> i32 {
    let ax = lp1.coords[0];
    let ay = lp1.coords[1];
    let bx = lp2.coords[0];
    let by = lp2.coords[1];
    let cx = p.coords[0];
    let cy = p.coords[1];

    let determinant = (bx - ax)*(cy - ay) - (by - ay)*(cx - ax);

    if determinant > 0.0
        { -1 }
    else if determinant < 0.0
        { 1 }
    else
        { 0 }
}

fn choose_new_center<'a, I>(segments: I, new_segment: &Segment) -> Point2
where I: Iterator<Item=&'a Segment> {
    let mut min_x = f64::NEG_INFINITY as Scalar;
    let mut min_y = f64::NEG_INFINITY as Scalar;
    let mut max_x = f64::INFINITY as Scalar;
    let mut max_y = f64::INFINITY as Scalar;

    let mut looped = false;
    for s in segments {
        looped = true;
        min_x = f64::min(s.p1.coords[0], min_x);
        max_x = f64::max(s.p1.coords[0], max_x);
        min_y = f64::min(s.p1.coords[1], min_y);
        max_y = f64::max(s.p1.coords[1], max_y);
    }
    if !looped
        { return Point2::new(0.0,0.0); }

    if new_segment.p2.coords[0] < min_x || new_segment.p1.coords[1] < min_y {
        new_segment.p2
    }
    else if new_segment.p1.coords[0] > max_x || new_segment.p1.coords[1] > max_y {
        new_segment.p1
    }
    else {
        let x = (min_x + max_x) / 2.0;
        let y = (min_y + max_y) / 2.0;
        Point2::new(x, y)
    }
}

impl<'a, SI> QTree<'a, SI>
where SI: 'a + Copy {
    pub fn make_empty_qtree() -> QTree<'a,SI>
    {
        let root = QTreeNode {
            segments: vec! [],
            child_info: None
        };
        return QTree {
            root: Box::new(root),
        };
    }

    pub fn in_order_iter(&self) -> QTreeInOrderIterator<SI> {
        QTreeInOrderIterator { stack: vec![(0, &*self.root)] }
    }

    pub fn ray_trace_iter<'b>(&'a self, ray: &'b Ray) -> QTreeRayTraceIterator<'a,'b,SI> {
        let m = (ray.p2.coords[1] - ray.p1.coords[1]) /
                (ray.p2.coords[0] - ray.p1.coords[0]);
        let k = ray.p2.coords[1] - (m * ray.p1.coords[0]);

        QTreeRayTraceIterator {
            ray: ray,
            ray_m: m,
            ray_k: k,
            stack: vec![(false, &*self.root)]
        }
    }

    fn split_node(r: &mut QTreeNode<'a, SI>, new_segment: &Segment) {
        if r.segments.len() <= QTREE_BIN_SIZE
            { return; }

        // Not possible to initialize an array in Rust using a loop in safe code.
        fn mk<'a, SI: Copy>() -> Box<QTreeNode<'a, SI>> { Box::new(QTreeNode { segments: vec![ ], child_info: None }) }
        let mut new_children: [Box<QTreeNode<'a,SI>>; 4] = [
            mk(), mk(), mk(), mk()
        ];

        let new_center = choose_new_center(r.segments.iter().map(|&(s,_)| s), new_segment);
        
        // Move segments downstairs if they're contained in only
        // one quad.
        let mut to_delete: HashSet<usize> = HashSet::new();
        let mut segi = 0;
        for &(seg, info) in &r.segments {
            let quad = get_segment_quad(seg, new_center);
            if quad != -1 {
                new_children[quad as usize].segments.push((seg, info));
                to_delete.insert(segi);
            }
            segi += 1;
        }

        if to_delete.len() > 0 {
            r.segments =
                r.segments
                .iter()
                .enumerate()
                .filter(|&(i,_)| !to_delete.contains(&i))
                .map(|(_,x)| *x)
                .collect();
        }

        r.child_info = Some(QTreeChildInfo {
            children: new_children,
            center: new_center
        });
    }

    pub fn insert_segment(&mut self, s: &'a Segment, info: SI)
    {
        let mut r: &mut QTreeNode<'a, SI> = &mut*self.root;

        loop {
            // The issue here is that pattern matching on the 'child_info' option
            // would cause 'r' to be borrowed within the scope of the 'if',
            // so that we could not subequently assign to 'r' within the 'if'.
            if r.child_info.is_some() {
                let quad = get_segment_quad(s, r.child_info.as_mut().unwrap().center);
                if quad == -1 {
                    r.segments.push((s, info));
                    break;
                }
                else {
                    // 1) Move out of r using the {r} trick.
                    // 2) Get reference to child_info within the struct referenced
                    //    by the temporary reference. This does not cause 'r' to
                    //    be borrowed.
                    // 3) Update value of r.
                    let child_info = {r}.child_info.as_mut().unwrap();
                    r = child_info.children.as_mut()[quad as usize].as_mut();
                }
            }
            else {
                r.segments.push((s, info));
                break;
            }
        }

        QTree::split_node(r, s);
    }

    pub fn insert_segments<F>(&mut self, segments: &'a Vec<Segment>, get_info: F) 
    where F: Fn(usize) -> SI
    {
        // Our aim here is to find a good order for segment insertion.
        // Generally, alternately going from the outside in and the inside out
        // works well. So we find the center point and then inversely order segments
        // by |p1|^2 + |p2|^2, where |p| is the distance of a point
        // p from the center.

        if segments.len() == 0
            { return; }

        let mut avg_x: Scalar = 0.0;
        let mut avg_y: Scalar = 0.0;
        for s in segments {
            avg_x += s.p1.coords[0] + s.p2.coords[0];
            avg_y += s.p1.coords[1] + s.p2.coords[1];
        }
        avg_x /= segments.len() as Scalar;
        avg_y /= segments.len() as Scalar;
        
        let mut sls: Vec<(Scalar, &'a Segment, usize)> = Vec::new();
        let mut ind = 0;
        for s in segments {
            let x1d = s.p1.coords[0] - avg_x;
            let x2d = s.p2.coords[0] - avg_x;
            let y1d = s.p1.coords[1] - avg_y;
            let y2d = s.p2.coords[1] - avg_y;

            sls.push((
                (x1d*x1d + x2d*x2d + y1d*y1d + y2d*y2d),
                &s,
                ind
            ));

            ind += 1;
        }

        sls.sort_by(|&(d1,_,_), &(d2,_,_)| d2.partial_cmp(&d1).unwrap());

        let mut si = 0;
        let mut ei = sls.len()-1;
        while si < ei {
            self.insert_segment(sls[si].1, get_info(sls[si].2));
            self.insert_segment(sls[ei].1, get_info(sls[ei].2));
            si += 1;
            ei -= 1;
        }
        if si == ei {
            self.insert_segment(sls[si].1, get_info(sls[si].2));
        }
    }

    pub fn get_segments_possibly_touched_by_ray(&'a self, ray: &Ray) -> Vec<(&'a Segment, SI)> {
        return self.ray_trace_iter(ray).flat_map(|x| x.iter()).map(|x| *x).collect();
    }

    pub fn get_segments_touched_by_ray(&'a self, ray: &Ray)
    -> Option <(Vec<(&'a Segment, SI)>, Point2, Scalar)> {
        let segs = self.get_segments_possibly_touched_by_ray(ray);

        let mut intersects: Vec<(&'a Segment, SI, Point2, Scalar)> = Vec::new();
        for (s,si) in segs {
            if let Some(pt) = ray_intersects_segment(ray, s) {
                let xd = pt.coords[0] - ray.p1.coords[0];
                let yd = pt.coords[1] - ray.p1.coords[1];
                let d = xd*xd + yd*yd;
                intersects.push((s, si, pt, d));
            }
        }

        if intersects.len() == 0
            { return None; }
        
        // Find the intersects closest to the start of the ray.
        intersects.sort_by(|&(_,_,_,d1),&(_,_,_,d2)| {
            d1.partial_cmp(&d2).unwrap()
        });

        // Skip any initial zero intercepts.
        let it = intersects.iter().skip_while(|&&(_,_,_,d)| d - EPSILON < 0.0);
        
        let mut last_d: Scalar = 0.0;
        let mut last_pt: Point2 = Point2::new(0.0, 0.0);
        let mut rs: Vec<(&'a Segment, SI)> = Vec::new();
        for &(s, si, pt, d) in it {
            if last_d == 0.0 || d == last_d {
                last_d = d;
                last_pt = pt;
                rs.push((s, si));
            }
            else {
                break;
            }
        }

        if rs.len() == 0 {
            None
        }
        else {
            let pt = last_pt;
            let d = last_d;
            //println!("INTERSECT {} {} FROM {} {} at {}", pt.coords[0], pt.coords[1], ray.p1.coords[0], ray.p1.coords[1], d);
            Some((rs, pt, d))
        }
    }
}

