$fn=72;

difference() {
    linear_extrude(height=5) import(file = "HorizontalPlate.dxf");

    rotate([0, 90, 0]){
        translate([-2.5, 5.5, -10]){
            cylinder(h=150, r=1.75);
        }
        translate([-2.5, 36.5, -10]){
            cylinder(h=150, r=1.75);
        }        
    }
}




