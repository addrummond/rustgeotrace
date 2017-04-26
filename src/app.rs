use getopts;
use getopts::Options;
use geom as g;
use geom_import as gi;
use trace as t;
use std::io;
use std::io::{Write, BufWriter, Seek};
use std::fs::File;
use std::error;
use std::fmt;
use std::num;
use std::fs;
use std::cmp;
use num_cpus;
use rand::{Rng};
use simplesvg;
use rayon::prelude::*;
use regex;
use tempfile;
use render;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;

#[derive(Clone)]
pub struct RunParams {
    pub max_depth: usize,              // 0 if no maximum
    pub max_rays: usize,               // 0 if no maximum
    pub svg_filename: Option<String>,  // not written if not specified
    pub hit_filename: Option<String>,  // stdout if not specified
    pub geom_filenames: Vec<String>,   // stdin if empty
    pub random_seed: usize,
    pub n_threads: usize
}

#[derive(Debug)]
pub enum CommandLineError {
    Custom(String),
    Getopts(getopts::Fail),
    UIntParse(num::ParseIntError)
}
impl From<getopts::Fail> for CommandLineError {
    fn from(err: getopts::Fail) -> CommandLineError {
        CommandLineError::Getopts(err)
    }
}
// Using <usize as str::FromStr>::Err instead of the concrete type
// gives rise to complicated errors and appears not to be worth the bother.
impl From<num::ParseIntError> for CommandLineError {
    fn from(err: num::ParseIntError) -> CommandLineError {
        CommandLineError::UIntParse(err)
    }
}
impl error::Error for CommandLineError {
    fn description(&self) -> &str {
        match *self {
            CommandLineError::Custom(ref s) => { s.as_str() },
            CommandLineError::Getopts(ref e) => { e.description() },
            CommandLineError::UIntParse(ref e) => { e.description() }
        }
    }
    fn cause(&self) -> Option<&error::Error> {
        match *self {
            CommandLineError::Custom(_) => { None },
            CommandLineError::Getopts(ref e) => { Some(e) },
            CommandLineError::UIntParse(ref e) => { Some(e) }
        }
    }
}
impl fmt::Display for CommandLineError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            CommandLineError::Custom(ref s) => { write!(f, "{}", s) },
            CommandLineError::Getopts(ref e) => { e.fmt(f) },
            CommandLineError::UIntParse(ref e) => { e.fmt(f) }
        }
    }
}

pub fn parse_command_line(args: &Vec<String>) -> Result<RunParams, CommandLineError> {
    let mut opts = Options::new();
    opts.optopt("g", "graphics", "set svg file output name", "NAME");
    opts.optopt("h", "hitfile", "set hit file output name", "NAME");
    opts.optopt("d", "maxdepth", "set the maximum recursion depth for tracing", "DEPTH");
    opts.optopt("r", "maxrays", "set the maximum number of rays to trace", "N");
    opts.optopt("s", "randseed", "set the seed used for generating random numbers", "N");
    opts.optopt("t", "threads", "set the number of threads to use; defaults to number of cores", "N");
    let matches = opts.parse(&args[1..])?;

    let mut max_depth: usize = 0;
    let mut max_rays: usize = 0;
    let mut svg_filename: Option<String> = None;
    let mut hit_filename: Option<String> = None;
    let mut random_seed: usize = 1;
    let mut n_threads: usize = num_cpus::get();

    fn parse_uint(v: &str, err: &str) -> Result<usize, CommandLineError> {
        v.parse::<usize>().map_err(|_| {
            CommandLineError::Custom(err.to_string())
        })
    }

    if let Some(v) = matches.opt_str("d") {
        max_depth = parse_uint(v.as_str(), "Argument to 'd' option must be an integer >= 0")?
    }
    if let Some(v) = matches.opt_str("r") {
        max_rays = parse_uint(v.as_str(), "Argument to 'r' option must be an integer >= 0")?
    }
    if let Some(v) = matches.opt_str("g") {
        svg_filename = Some(v)
    }
    if let Some(v) = matches.opt_str("h") {
        hit_filename = Some(v)
    }
    if let Some(v) = matches.opt_str("s") {
        random_seed = parse_uint(v.as_str(), "Argument to 's' option must be an integer >= 0")?
    }
    if let Some(v) = matches.opt_str("t") {
        n_threads = parse_uint(v.as_str(), "Argument to 't' option must be an integer > 0")?;
        if n_threads == 0
            { return Err(CommandLineError::Custom("Argument to 't' option must be an integer > 0".to_string())); }
    }

    debug_assert!(n_threads > 0);

    Ok(RunParams {
        max_depth: max_depth,
        max_rays: max_rays,
        svg_filename: svg_filename,
        hit_filename: hit_filename,
        geom_filenames: matches.free.clone(),
        random_seed: random_seed,
        n_threads: n_threads
    })
}

fn beams_to_rays(beams: &Vec<gi::Beam>) -> Vec<(g::Ray, t::LightProperties, usize)> {
    let mut rays: Vec<(g::Ray, t::LightProperties, usize)> = Vec::new();
    for b in beams {
        match *b {
            gi::Beam::Collimated { from, to, shiny_side_is_left, n_rays, light_properties, n_traces } => {
                let it = g::CollimatedBeamRayIterator::new(from, to, shiny_side_is_left, n_rays);
                for (p1, p2) in it {
                    let new_ray = g::Ray {
                        p1: p1,
                        p2: p2
                    };
                    rays.push((new_ray, light_properties, n_traces));
                }
            },
            gi::Beam::Ray { from, to, light_properties, n_traces } => {
                rays.push((g::Ray { p1: from, p2: to }, light_properties, n_traces));
            }
        }
    }
    rays
}

fn box_error_send<'a,E>(e: E) -> Box<error::Error + Send + 'a>
where E: error::Error + Send + 'a {
    Box::new(e)
}

fn add_n_to_filename(filename: &str, n: usize) -> String {
    let re = regex::Regex::new(r"((?:\.[^.\\/]+$)|(?:$))").unwrap();
    let repl = format!("_{}$1", n);
    re.replace_all(filename, repl.as_str()).to_string()
}

fn output_svg(svg: &simplesvg::Svg, outname: &str) -> Result<(), Box<error::Error + Send>> {
    let f = File::create(outname).map_err(box_error_send)?;
    let mut f = BufWriter::new(f);
    f.write_all(svg.to_string().as_bytes()).map_err(box_error_send)            
}

fn get_hit_writer(name: &Option<String>) -> Result<Box<Write>, Box<error::Error>> {
    Ok(match *name {
        None => { Box::new(io::stdout()) },
        Some(ref name) => {
            let f = File::create(name)?;
            let f = BufWriter::new(f);
            Box::new(f)
        }
    })
}

pub fn do_run(params: &RunParams) -> Result<(), Box<error::Error>> {
    assert!(params.n_threads > 0);

    let mut geoms: Vec<gi::ImportedGeometry> = Vec::new();

    if params.geom_filenames.len() == 0 {
        geoms.push(gi::parse_geometry_file(Box::new(io::stdin()))??);
    }
    for filename in &params.geom_filenames {
        let file = Box::new(fs::File::open(filename)?);
        geoms.push(gi::parse_geometry_file(file)??);
    }

    assert!(geoms.len() > 0);
    let geom;
    if geoms.len() == 1 {
        geom = &geoms[0];
    }
    else {
        let (a, b) = geoms.split_at_mut(1);
        gi::combine_imported_geometry(&mut a[0], b)?;
        geom = &a[0];
    }

    let mut starting_rays = beams_to_rays(&geom.beams);

    let mut qtree: g::QTree<t::RayTraceSegmentInfo> = g::QTree::make_empty_qtree();
    qtree.insert_segments(&geom.segments, |i| i);

    let tracing_props = t::TracingProperties {
        random_seed: [params.random_seed],
        intensity_threshold: 0.01
    };

    let rt_init = t::RayTraceInitArgs {
        tracing_properties: &tracing_props,
        qtree: &qtree,
        segment_names: &geom.segment_names,
        materials: &geom.materials,
        left_material_properties: &geom.left_material_properties,
        right_material_properties: &geom.right_material_properties
    };
    let mut st = t::RayTraceState::initial(&rt_init);

    // Shuffle starting rays so that different threads get a roughly equally
    // "persistent" set of rays.
    st.get_rng().shuffle(starting_rays.as_mut_slice());

    let mut thread_local_data
        :Vec<(usize, Vec<(g::Ray, t::LightProperties, usize)>, t::RayTraceState, RunParams)>
        = (0..params.n_threads).map(|i| (i, Vec::new(), st.clone(), params.clone())).collect();

    let mut current_thread = 0;
    for &(ray, light_props, n_traces) in &starting_rays {
        if n_traces == 1 {
            thread_local_data[current_thread].1.push((ray, light_props, n_traces));
            current_thread += 1;
            if current_thread >= thread_local_data.len()
                { current_thread = 0; }
        }
        else {
            let n_per_thread = cmp::max(1, ((n_traces as f64) / (params.n_threads as f64)) as usize);
            let mut traces_remaining = n_traces;
            for &mut (_, ref mut v, _, _) in &mut thread_local_data {
                v.push((
                    ray,
                    light_props,
                    n_per_thread
                ));

                if traces_remaining < n_per_thread
                    { break; }
                traces_remaining -= n_per_thread;
                if traces_remaining == 0
                    { break; }
            }
        }
    }

    // Remove any threads that have nothing to do. These should always be at the end of the list.
    let mut first_nonempty = thread_local_data.len();
    for &(_, ref v, _, _) in thread_local_data.iter().rev() {
        if v.len() != 0
            { break; }
        first_nonempty -= 1;
    }
    thread_local_data.truncate(first_nonempty);

    let result: Result<Vec<File>, Box<error::Error + Send>> =
    thread_local_data
    .par_iter_mut()
    .map(|&mut (ref i, ref mut starting_rays, ref mut st, ref params)| -> Result<Vec<File>,Box<error::Error + Send>> {
        let mut hit_tempfile = tempfile::tempfile().map_err(box_error_send)?;

        let mut rayb = t::RayBuffer {
            old_rays: starting_rays.clone(),
            new_rays: Vec::new()
        };
        let mut figs: Vec<simplesvg::Fig> = Vec::new();
        let mut count = 0;
        loop {
            let t = render::get_display_transform(
                &geom.segments,
                &(rayb.old_rays),
                render::DisplayTransformArgs {
                    width: WIDTH,
                    height: HEIGHT,
                    border: 40,
                    offset_x: 0.0,
                    offset_y: (count * HEIGHT) as g::Scalar,
                    scale_factor: 1.0,
                    keep_aspect_ratio: true
                }
            );

            count += 1;

            if params.svg_filename.is_some() {
                figs.push(render::render_segments(&geom.segments, &t, [0.0, 1.0, 0.0]));
                figs.push(render::render_rays(rayb.get_rays(), &t, [1.0, 0.0, 0.0]));
            }

            let result = t::ray_trace_step(st, &mut rayb, |e: &t::Event| -> Result<(),Box<error::Error + Send>> { 
                match *e {
                    t::Event::Hit { ref segment_index, ref segment_name, ref point } => {
                        write!(hit_tempfile, "hit,{},{},{},{},{}\n", *i, segment_index, segment_name, point.coords[0], point.coords[1]).map_err(box_error_send)?;
                        Ok(())
                    }
                }
            })?;

            // Is it time to stop tracing?
            if params.max_rays != 0 && result.ray_count >= params.max_rays
                { break; }
            if params.max_depth != 0 && result.recursion_level >= params.max_depth
                { break; }
            if rayb.get_n_rays() == 0
                { break; }
        }

        if let Some(ref svg_filename) = params.svg_filename {
            let svg = simplesvg::Svg(figs, WIDTH, (count*HEIGHT));
            let svg_filename = add_n_to_filename(svg_filename.as_str(), *i);
            output_svg(&svg, svg_filename.as_str())?;
        }

        Ok(vec![hit_tempfile])
    })
    .reduce_with(|r1, r2| {
        if r1.is_err()
            { r1 }
        else if r2.is_err()
            { r2 }
        else {
            Ok(r1.unwrap().into_iter().chain(r2.unwrap().into_iter()).collect())
        }
    }).unwrap();

    match result {
        Err(e) => { Err(e) },
        Ok(mut tempfiles) => {
            // Concatenate all the hits files.
            let hit_writer = get_hit_writer(&params.hit_filename)?;
            let mut w = io::BufWriter::new(hit_writer);

            write!(w, "event_type,thread,segment_index,segment_name,x,y\n")?;

            for t in &mut tempfiles {
                t.seek(io::SeekFrom::Start(0))?;
                io::copy(t, &mut w)?;
            }

            Ok(())
        }
    }
}