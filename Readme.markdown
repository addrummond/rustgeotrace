This is a program for doing basic geometric ray tracing in two dimensions.
I developed it to help with the design of diffusers and light pipes.

The program is written in [Rust](https://www.rust-lang.org/en-US/). It compiles with stable Rust (rustc v. 1.16.0).

Tracing procedure and termination
=====

Tracing ends naturally if a tracing cycle ends with no new rays
being introduced by reflection or refraction. Otherwise,
it ends once either (i) the total number of rays traced exceeds
a specified value or (ii) the “depth” of the trace exceeds
a specified value. The depth is equal to the number of ancestor
rays of the ray currently being traced.

The program outputs a CSV
file containing a record of every instance in which one of a
specified set of segments was hit by a ray. It can also
produce SVG files giving
a simple graphical record of the trace.


Geometry files
=====

The input to the tracer is one or more a geometry files. A Geometry file has
one declaration per line. Declarations may be either `material`
declarations, `segment`, `box`, `circle` or `arc` declarations,
or `ray` or `beam` declarations. Blank lines are ignored.
Comments can be introduced using `#` in the usual way.


Defining materials and geometry
=====

Geometry is constructed out of directed segments.
Each segment is tagged with the material
to its left (looking along the segment)
and the material to its right.
A segment can be added to a geometry file using
the following syntax:

    line air/mirror -10 10 10 10

This specifies a segment from (-10, 10) to (10, 10)
with `air` to its left and `mirror` to its right.
The properties of the materials `air` and `mirror`
are specified using material declarations such as the following:

    material mirror drf=0 srf=1 rff=0

A ray that strikes a segment may undergo either diffuse
reflection, specular reflection, or refraction.
The above definition indicates that the probability
of specular reflection (`srf`) is 1 and that the probabilities
of diffuse reflection and refraction (`drf` and `rff`)
are zero. In general, the probabilities are normalized
with respect to the sum of the values for `drf`, `srf`
and `rff`. The default value for each or `drf`, `srf` and `rff`
is 0, so the preceding definition could be written as follows:

    material mirror srf=1

The refractive properties of a material are determined by
a series of coefficients to be plugged in to the general
form of [Cauchy's equation](https://en.wikipedia.org/wiki/Cauchy%27s_equation).
For example, air could be defined [as follows](http://scienceworld.wolfram.com/physics/CauchysFormula.html),
assuming negligible reflection:

    material air drf=0 srf=0 rff=1 c1=1.000287566 c2=1.3412e-18 c3=3.777e-32

By default, materials do not absorb any of the intensity of a ray when
the ray is reflected by diffuse or specular reflection. The property
'abs' can be used to specify the fraction of the intensity that is
absorbed:

    material mirror srf=1 abs=0.1

It is also possible to give different absorption values for diffuse and specular
reflection as follows:

    material mirror srf=0.9 drf=0.1 drf_abs=0.1 srf_abs=0.2

The `drf_abs` and `srf_abs` options are incompatible with the `abs` option.

As rays travel through a material their intensity is
attenuated in proportion to the square of the distance traveled.
The relevant attenuation constant is specified using `att`.
By default, materials do not attenuate.

Although all geometry is ultimately modeled using segments, there are utilities
for constructing circles, arcs and boxes. A box is specified by giving
two diagonally opposite corners:

    box air/mirror -5 5 5 -5

The first segment is horizontal and leaves from the first corner. So for example,
in the case of the box defined above, the first segment goes from (-5, 5) to
(5, 5), the second from (5, 5) to (5, -5), and so on.

A circle is specified by giving its center point, a point on the circumference,
and the number of segments to use to approximate the circle:

    circle (10) air/mirror 0 0 0 20      # Circle with center (0, 0) and radius 20 approximated using 10 segments.

An arc is specified by giving the center point of a circle,
a point on its circumference, and a point defining a line L from
the center. The arc defined is the clockwise arc from the point on the circumference
to the intersection of L with the circumference. E.g.:

    arc (10) air/mirror 0 0 0 20 20 0    # 90 degree arc in top right quadrant approximated using 10 segments.


Defining rays
=====

A ray is defined by two points. It originates at the first point and goes off
in the direction of the second. A ray has an intensity
(which by convention is > 0 and ≤ 1) and a wavelength specified in m.
By default each ray is traced once.
The following example defines a ray to be traced 10 times
from (-10, 10) in the direction of (20, 10) with intensity 1.0 and
wavelength 500nm:

    ray i=1.0 l=5e-7 t=10 | -10 10 20 10

There is a 'beam' utility for defining a series of evenly spaced parallel
rays. A beam of this sort is defined by specifying a line segment,
a direction (left or right) and a number of rays. The rays have their
origin on the line segment and are perpendicular to it, going off
to either the left or the right (from the point of view of someone
looking from the first point of the segment towards the second).
For example:

    beam i=1.0 l=5e-7 t=10 n=20 |- 0 0 0 10
    beam i=1.0 l=5e-7 t=10 n=20 -| 0 0 0 10

Each of the preceding statements defines 20 rays which have
evenly-spaced origins along the line segment (0, 0) to (0, 10).
The rays defined by the first statement go off to the right,
while the rays defined by the second statement go off to the left.


Naming segments
=====

If you wish to record instances where rays hit a particular segment,
you must name the segment:

    line air/mirror -10 10 10 10 named foo
    box air/mirror -5 5 5 -5 named bar

In the case of the `circle`, `arc` and `box` utilities, the
resulting segments are named by appending `_1`, `_2`, etc.
to the given name. So for example, the 'box' declaration
above will name its segments `bar_1`, `bar_2`, `bar_3`, and
`bar_4`.


Command line arguments
=====

The program must be passed the names of zero or more geometry files. If no geometry file names are given as arguments, the program reads a single geometry file from stdin. Multiple geometry files are combined where consistent with each other. The program exits with an error if there are conflicts between them.

The following flags may also be used:

-g NAME (optional)
-----
Specifies an output file name `NAME` for SVG diagrams of the trace.
Separate diagrams are produced for each thread named `NAME_1`, `NAME_2` etc,
or `N_1.EXTENSION` if `NAME` is of the form `N.EXTENSION`.

-h NAME (optional)
-----
Specifies the output file name `NAME` for the record of instances where a named segment was hit by a ray.
The file is in CSV format with the first line giving column headings. If the `-h` option is not given, the record of hits is printed to stdout.

-d MAXDEPTH (optional)
-----

If specified, then the trace stops once it reaches a depth of `MAXDEPTH`.
By default, or if `MAXDEPTH` is set to 0, there is no limit to the depth of the trace.

-r MAXRAYS (optional)
-----

If specified then the trace stops once `MAXRAYS` rays have been traced.
By default, or if `MAXRAYS` is set to 0, there is no limit to the number of rays traced.

-s RANDSEED (optional)
-----

Specifies a random seed. This should be a value that fits in a 32-bit unsigned integer. The computation is deterministic for a given seed.

-t N_THREADS (optional)
-----

Specifies the number of threads to use when performing the trace. By default, the program attempts to determine the number of cores available and runs one thread per core.

