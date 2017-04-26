//! This module provides utilities for implementing recursive
//! descent parsers. It offers a sort of halfway house between
//! using a true parser combinator library and writing a recursive
//! descent parser from scratch. It provides a standard type for
//! parsers, `Parser<T>`, a few basic parsers such as parsers for
//! identifiers, numeric constants, etc., and some higher order parsers.
//!
//! The utility functions are designed to make it difficult to
//! accidentally write backtracking parsers. Recursive descent parsers
//! are efficient only if they have limited backtracking.
//!
//! IMHO the parser combinator approach is a bit clunky in
//! Rust, and of little benefit in an imperative language that already has
//! state and sequencing as primitive concepts and built-in syntactic support
//! for the Option monad.

use std::str;
use std::error;
use std::fmt;

/// This type should be used by all parsers to signal an error.
#[derive(Debug)]
pub struct ParseError {
    pub line: usize,
    pub col: usize,
    pub err: String
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Line {} col {}: {}", self.line, self.col, self.err)
    }
}

impl error::Error for ParseError {
    fn description(&self) -> &str {
        "Parse error"
    }
    fn cause(&self) -> Option<&error::Error> {
        Some(self)
    }
}

/// All parsers should return values of this type.
pub type ParseResult<T> = Result<T, ParseError>;
pub trait Parser<T>: FnMut (&mut ParseState) -> ParseResult<T> { }
impl <T,U> Parser<T> for U where U: FnMut (&mut ParseState) -> ParseResult<T> { }

/// This function gets the next unicode code point given a slice of bytes.
/// Once the 'next_code_point' function moves to stable, it should be possible
/// to implement the function below a little more simply and efficiently.
fn get_next_utf8_codepoint_as_char(arr: &[u8]) -> Option<(char,usize)> {
    // As non-ASCII chars will be rare in practice, try decoding
    // just one byte first, then two, then three, etc.
    for i in 1..5 { // Max length of UTF-8 codepoint is 4 bytes.
        if i > arr.len()
            { return None }
        let r = str::from_utf8(&arr[0..i]);
        if let Ok(s) = r {
            if let Some(c) = s.chars().next()
                { return Some((c, i)); }
        }
    }
    
    None
}

#[derive(Copy, Clone, PartialEq)]
/// Represents the position of the parser in the input stream.
pub struct ParsePosition {
    i: usize,
    line: usize,
    col: usize,
    peek: usize,
    i_at_last_peek: isize
}

/// Packages together the input byte slice with a position.
pub struct ParseState<'a> {
    input: &'a [u8],
    pos: ParsePosition
}

impl<'a> ParseState<'a> {
    /// Given a byte slice, returns the `ParseState` required to
    /// start parsing at the beginning.
    pub fn new(input: &'a [u8]) -> ParseState {
        ParseState {
            input: input,
            pos: ParsePosition {
                i: 0,
                line: 1,
                col: 0,
                peek: 0,
                i_at_last_peek: -1
            }
        }
    }
}

/// Get the current position.
pub fn get_position(st: &mut ParseState) -> ParsePosition {
    st.pos
}

/// Restore a saved position.
pub fn restore_position(st: &mut ParseState, pos: &ParsePosition) {
    st.pos = *pos;
}

/// Returns true iff the end of the input stream has been reached.
pub fn at_eof(st: &mut ParseState) -> bool {
    st.pos.i >= st.input.len()
}

/// Decode the next UTF8 code point from the input stream without advancing
/// the position of the parser.
pub fn peek_char(st: &mut ParseState) -> ParseResult<Option<char>> {
    if st.pos.i >= st.input.len()
        { return Ok(None); }
    
    let sl = &st.input[st.pos.i ..];
    match get_next_utf8_codepoint_as_char(sl) {
        None => { parse_error_string(st, format!("UTF-8 decode error at byte {}", st.pos.i)) },
        Some((c, n)) => {
            st.pos.peek = n;
            st.pos.i_at_last_peek = st.pos.i as isize;
            Ok(Some(c))
        }
    }
}

fn update_line_col(st: &mut ParseState, c: char) {
    if c == '\n' {
        st.pos.line += 1;
        st.pos.col = 0;
    }
    else {
        st.pos.col += 1;
    }
}

/// Update the position of the parser following a call to `peek_char`.
/// This should be passed the char returned by `peek_char`.
/// In debug builds, an assertion will fail if the parser position
/// has already advanced following the last call to `peek_char`, or
/// if `peek_char` was never called.
pub fn skip_peeked(st: &mut ParseState, c: char) {
    // If this isn't true, then more was read since the last peek
    // and we can't skip the last peek, or no peek ever occurred.
    debug_assert!((st.pos.i_at_last_peek as usize) == st.pos.i);

    st.pos.i += st.pos.peek;
    update_line_col(st, c);
}

/// Decode the next UTF8 code point from the input stream and update
/// the parser's position accordingly.
pub fn next_char(st: &mut ParseState) -> ParseResult<Option<char>> {
    if st.pos.i >= st.input.len()
        { return Ok(None); }
    
    let sl = &st.input[st.pos.i ..];
    match get_next_utf8_codepoint_as_char(sl) {
        None => {
            parse_error_string(st, format!("UTF-8 decode error at byte {}", st.pos.i))
        },
        Some((c, n)) => {
            st.pos.i += n;
            update_line_col(st, c);
            Ok(Some(c))
        }
    }
}

/// Skip n code points of the input stream.
pub fn skip_nchars(st: &mut ParseState, mut n: usize) ->
ParseResult<()> {
    assert!(n >= 1);

    n += 1;
    while n > 0 {
        if let None = next_char(st)?
            { return parse_error(st, "Unexpected EOF"); }
        n -= 1;
    }

    Ok(())
}

/// Skip code points until one is encountered that does not match the
/// supplied predicate. Returns the number of code points skipped.
pub fn skip_while<F>(st: &mut ParseState, mut filter: F) ->
ParseResult<usize>
where F: FnMut(char) -> bool {
    let mut n: usize = 0;
    while let Some(c) = peek_char(st)? {
        if filter(c) {
            n += 1;
            skip_peeked(st, c);
        }
        else {
            break;
        }
    }

    Ok(n)
}

/// Read code points until one is encountered that does not match the
/// supplied predicate. Return a vector of the code points read.
pub fn take_while<F>(st: &mut ParseState, mut filter: F) ->
ParseResult<Vec<char>>
where F: FnMut(char) -> bool {
    let mut r: Vec<char> = Vec::new();

    while let Some(c) = peek_char(st)? {
        if filter(c) {
            skip_peeked(st, c);
            r.push(c);
        }
        else {
            break;
        }
    }

    Ok(r)
}

/// Utility function for constructing parse errors.
pub fn parse_error_string<X>(st: &ParseState, error_msg: String) -> ParseResult<X> {
    Err(ParseError {
        line: st.pos.line,
        col: st.pos.col,
        err: error_msg
    })
}

/// Utility function for constructing parse errors.
pub fn parse_error<X>(st: &ParseState, error_msg: &str) -> ParseResult<X> {
    parse_error_string(st, error_msg.to_string())
}

/// A parser that expects to find a given string, and fails if it does not
/// find it.
pub fn expect_str(st: &mut ParseState, expected: &str) -> ParseResult<()> {
    let mut it = expected.chars();
    let mut error = false;

    while let Some(c) = peek_char(st)? {
        match it.next() {
            None => { break; },
            Some(cc) => {
                if c != cc {
                    error = true;
                    break;
                }
                skip_peeked(st, c);
            }
        }
    }

    match it.next() {
        None => {
            if error {
                parse_error_string(st, format!("Expected '{}'", expected))
            }
            else {
                Ok(())
            }
        },
        Some(_) => {
            parse_error_string(st, format!("Expected '{}'", expected))
        }
    }
}

/// A parser that skips whitespace chars, including single-line comments
/// initiated by the supplied comment char, and returns the last char
/// skipped, if any, together with the number of chars skipped.
/// The `include_nl` parameters determines whether or not newlines ('\n')
/// count as whitespace. To skip whitespace without parsing comments
/// use `skip_while` with a suitable predicate.
pub fn skip_space_wc(st: &mut ParseState, comment_char: char, include_nl: bool) ->
ParseResult<(Option<char>, usize)> {
    let mut cc: Option<char> = None;
    let mut in_comment = false;
    let mut count = 0;

    while let Some(c) = peek_char(st)? {
        count += 1;

        if c == '\n' {
            cc = Some(c);
            in_comment = false;
            if !include_nl
                { return Ok((cc, count)); }
            skip_peeked(st, c);
        }
        else if char::is_whitespace(c) {
            cc = Some(c);
            skip_peeked(st, c);
        }
        else if c == comment_char {
            cc = Some(c);
            in_comment = true;
            skip_peeked(st, c);
        }
        else if in_comment {
            cc = Some(c);
            skip_peeked(st, c);
        }
        else {
            count -= 1;
            return Ok((cc, count));
        }
    }

    Ok((cc, count))
}

/// Skips non-newline whitespace, including single-line comments
/// initiated by the supplied comment char,
/// and returns the last char skipped, if any. To skip
/// whitespace without parsing comments use `skip_while` with a suitable predicate.
pub fn skip_space(st: &mut ParseState, comment_char: char) -> ParseResult<Option<char>> {
    Ok(skip_space_wc(st, comment_char, false)?.0)
}

/// Skips non-newline whitespace, including single-line comments
/// initiated by the supplied comment char, and returns
/// the last char skipped. Fails if no whitespace characters
/// are encountered. To skip whitespace without parsing comments use
/// `skip_while` with a suitable predicate.
pub fn skip_at_least_one_space(st: &mut ParseState, comment_char: char) -> ParseResult<char> {
    let (r, c) = skip_space_wc(st, comment_char, false)?;
    if c > 0
        { Ok(r.unwrap()) } // 'unwrap' guaranteed not to panic given that c > 0
    else
        { parse_error(st, "Expected whitespace, found '{}'") }
}

/// Skips whitespace, including newlines and single-line comments
/// initiated by the supplied comment char, and returns
/// the last char skipped. Fails if no whitespace characters
/// are encountered. To skip whitespace without parsing comments use
/// `skip_while` with a suitable predicate.
pub fn skip_space_inc_nl(st: &mut ParseState, comment_char: char) -> ParseResult<Option<char>> {
    Ok(skip_space_wc(st, comment_char, true)?.0)
}

/// Parses an identifier. I.e. an alpha char or '_' followed by 0 or more
/// instances of an alphanumeric char or '_'.
pub fn identifier(st: &mut ParseState) -> ParseResult<String> {
    let mut current_str: Vec<char> = Vec::new();
    while let Some(c) = peek_char(st)? {
        if char::is_alphanumeric(c) || c == '_' {
            current_str.push(c);
            skip_peeked(st, c);
        }
        else {
            break;
        }
    }

    if current_str.len() == 0 {
        parse_error(st, "Expected identifier")
    }
    else {
        Ok(current_str.into_iter().collect())
    }
}

/// Repeatedly applies `parser` and `sep` in sequence.
/// The parse ends successfully when either `parser` or `sep` fails
/// without advancing the position of the parser. This may happen
/// immediately, in which case the parse succeeds and yields an
/// empty vector. If either `sep` or `parser` fails after advancing
/// the position of the input, the the parse fails with the parse error
/// returned by `sep` or `parser`. This behavior ensures that backtracking
/// parsers cannot be accidentally constructed.
pub fn sep_by<R1,R2,F1,F2>(st: &mut ParseState, mut sep: F1, mut parser: F2) -> ParseResult<(Vec<R2>, ParseError)>
where F1: Parser<R1>,
      F2: Parser<R2> {

    let mut rs: Vec<R2> = Vec::new();
    let mut pos = get_position(st);

    loop {
        match parser(st) {
            Ok(r) => {
                rs.push(r);
                pos = get_position(st);
            }
            Err(e) => {
                if pos != get_position(st)
                    { return Err(e); }
                else
                    { return Ok((rs, e)); }
            }
        }

        if let Err(e) = sep(st) {
            if pos != get_position(st)
                { return Err(e); }
            else
                { return Ok((rs, e)); }
        }

        pos = get_position(st);
    }
}

/// `space_separated(st, parser)` is equivalent to `sep_by(st, skip_at_least_one_space, parser)`.
pub fn space_separated<R,F>(st: &mut ParseState, comment_char: char, parser: F) -> ParseResult<(Vec<R>, ParseError)>
where F: Parser<R> {
    sep_by(st, |st: &mut ParseState| { skip_at_least_one_space(st, comment_char) }, parser)
}

/// Parses a floating point constant as an `f64`.
pub fn numeric_constant(st: &mut ParseState) -> ParseResult<f64> {
    let mut n = 0;
    let chars = take_while(st, |c| {
        n += 1;
        char::is_digit(c, 10) ||
        c == 'e' || c == '+' || c == '-' || c == '.'
    })?;

    if chars.len() == 0 {
        parse_error(st, "Expecting numeric constant")
    }
    else {
        let s: String = chars.into_iter().collect();
        match s.as_str().parse::<f64>() {
            Err(_) => {
                parse_error_string(st, format!("Error in syntax of numeric constant '{}'", s))
            },
            Ok(v) => {
                Ok(v)
            }
        }
    }
}