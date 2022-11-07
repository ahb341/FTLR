// the point of this file is to be a sort of DSL for constructing keycaps.
// when you create a method chain you are just changing the parameters
// key.scad uses, it doesn't generate anything itself until the end. This
// lets it remain easy to use key.scad like before (except without key profiles)
// without having to rely on this file. Unfortunately that means setting tons of
// special variables, but that's a limitation of SCAD we have to work around

include <./includes.scad>


// example key
translate_u(0, 0) dsa_row(3) legend("Pair", size=4)     alps() key();
translate_u(0, 1) dsa_row(3) legend("Mode3", size = 3)  alps() key();
translate_u(1, 0) dsa_row(3) legend("Input", size = 3)  alps() key();
translate_u(1, 1) dsa_row(3)  alps() key();
translate_u(2, 0) dsa_row(3)  alps() key();
translate_u(2, 1) dsa_row(3)  alps() key();

// example row
/* for (x = [0:1:4]) {
  translate_u(0,-x) dcs_row(x) key();
} */

// example layout
/* preonic_default("dcs") key(); */
