$fn = 30;


tp_h = 1.6;
tp_w = 65.0;
tp_d = 49.0;
tp_r = 2.5;

wall_t = 1.6;
base_t = 1.2;

box_h = 9;
box_w = tp_w + 2*wall_t;
box_d = tp_d + 2*wall_t;
box_r = tp_r + wall_t;
box_inner_l = 2*wall_t;
box_inner_w = box_w - 2*box_inner_l;
box_inner_d = box_d - 2*box_inner_l;
box_inner_r = tp_r - wall_t;

module rounded_cube(x, y, z, r) {
    hull() {
        translate([r, r, 0])
        cylinder(r=r, h=z, center=false);
        translate([x-r, r, 0])
        cylinder(r=r, h=z, center=false);
        translate([x-r, y-r, 0])
        cylinder(r=r, h=z, center=false);
        translate([r, y-r, 0])
        cylinder(r=r, h=z, center=false);
    }
}


difference() {
    rounded_cube(box_w, box_d, box_h, box_r);
    translate([box_inner_l, box_inner_l, base_t])
        rounded_cube(box_inner_w, box_inner_d, box_h, box_inner_r);
    translate([wall_t, wall_t, box_h-tp_h])
        rounded_cube(tp_w, tp_d, tp_h+0.1, tp_r);
    translate([box_w - 8, box_d/2-5, -0.1])
        cube([2, 10, base_t+0.2]);
}