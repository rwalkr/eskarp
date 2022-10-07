$fn=30;

module adj_stand_top() {
    bolt_d = 4;
    bolt_h = 7;
    w = 12;
    d = 80;
    h = 2;
    h2 = 5 + 2.5;
    cube([w, d, h]);
    translate([w/2, 10, h]) {
        nut_cutout();
    }
    translate([w/2, d-10, h]) {
        nut_cutout();
    }
    module nut_cutout() {
        difference() {
            cylinder(h = h2, d = bolt_h+4);
            cylinder(h = h2+.1, d = bolt_d);
            translate([0, 0, h2-3])
                cylinder(h = 3+.1, d = bolt_h);
        }
    }
}

module adj_stand_base() {
    bolt_h = 7;
    w = 20;
    d = 80;
    h = 2;
    h2 = 3;
    cube([w, d, h]);
    translate([w/2, 10, h]) {
        socket();
    }
    translate([w/2, d-10, h]) {
        socket();
    }
    module socket() {
        difference() {
            cylinder(h = h2, d = bolt_h+3.2);
            cylinder(h = h2+.1, d = bolt_h);
        }
    }
}

adj_stand_top();
translate([15, 0, 0])
    adj_stand_base();