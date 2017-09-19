rMain=18.4;
hMain=6;
dHub = 20;
hHub = 14;
wheelHolesDistance=13;
$fn=180;


difference() {
    union(){
        cylinder(h=hMain, r=rMain, center=true);
        rotate([180, 0, 60]){
            translate([0, 0, (hHub + hMain)/2]){
                hub(hHub, dHub, 5.5, 5.1, 1, [6, 3], 3.2, -2, 0, 4, 4);
            }
        }
	}
	
	cylinder(h=hMain, r=2.75, center=true);
	wheelScrews(hMain+0.1, 2);
}





module wheelScrews(height, recess) {
	translate([wheelHolesDistance,0,0]){
		wheelScrew(height, recess);
	}
	rotate([0,0,120]){
		translate([wheelHolesDistance,0,0]){
			wheelScrew(height, recess);
		}
	}
	rotate([0,0,-120]){
		translate([wheelHolesDistance,0,0]){
			wheelScrew(height, recess);
		}
	}
}

module wheelScrew(height, recess){
	cylinder(r=1.7, h=height, center=true);
	translate([0,0,-height/2 - 100]){
        rotate([0, 0, 30]){
            cylinder(r=3.3, h=2*(recess + 100), center=true, $fn=6);
        }
	}		
}





// The hub (the part that holds the wheel onto the motor
module hub( height, 
            diameter, 
            boreDiameter, // The diameter of the motor shaft
            shaftFlatDiameter, // The diameter of the motor shaft at the flat, or shaftDiameter for no flat.
            nuts, // The number of set screws/nuts to render, spaced evenly around the shaft 
            nutSize, // Size [indiameter, thickness] of set screw nut. The depth is set automatically.
            setScrewDiameter, // The diameter of the set screw. 3 is the default for an M3 screw.
            setScrewNutOffset=0,	// The distance to offset the nut from the center of the material. -/+ = in/out
            hubZOffset=0, 
            baseFilletRadius=0, // The radius of the fillet (rounded part) between the hub and wheel.
            topFilletRadius=0, // The radius of the fillet (rounded part) at the top of the hub.
            chamferOnly=false, // Set to true to use chamfers (straight 45-degree angles) instead of fillets.
            hexbore=false // Make the bore a hex shaft vs a circle
) 
{

	hubWidth=(diameter-boreDiameter)/2;

	union() {	
		difference() {

			// Main hub shape
			union() {
				difference() {
					union() {
						cylinder( h=height, r=diameter/2, center=true );
			
						// First chamfer the base...
						rotate_extrude() 
							translate([diameter/2,-(height/2)-hubZOffset,0])
								polygon(points=[[0,0],[0,baseFilletRadius],[baseFilletRadius,0]]);
					}
			
					// Chamfer the top...
					rotate_extrude() 
						translate([diameter/2,height/2,0])				
							polygon(points=[[0.5,0.5],[-topFilletRadius-0.5,0.5],[0.5, -topFilletRadius-0.5]]);
			
					// Carve the bottom fillet from the chamfer
					if ( !chamferOnly ) { 
						rotate_extrude() {
							translate([(diameter/2)+baseFilletRadius,
								-(height-(2*baseFilletRadius))/2-hubZOffset,0]) {
								circle(r=baseFilletRadius);
							}
						}
					}
				}

				// Add the fillet back on top of the top chamfer 
				if (!chamferOnly) {
					rotate_extrude() {
						translate([
							(diameter/2)-topFilletRadius,
							(height-(2*topFilletRadius))/2,
							0])				
							circle(r=topFilletRadius);
					}
				}
			}
	
			// Remove the bore
			difference() {
				if (hexbore) {
					cylinder(r=boreDiameter/2/ cos(180/6),h=height+1,$fn=6, center=true);
            } else {
					difference(){
				   	cylinder( h=height+1, r=boreDiameter/2, center=true );
        				translate([(boreDiameter-shaftFlatDiameter+1)/2 + (boreDiameter/2) 
							- (boreDiameter - shaftFlatDiameter),0,0]) 
							cube( [boreDiameter-shaftFlatDiameter+1,boreDiameter,height+2], center=true );
					}
            } 
			}
			
	
			// Remove the captive nut
			for( i=[0:nuts-1] ) {
				if (hexbore) {
					rotate([ 0,0, (360/nuts)*i+30 ])
						translate([boreDiameter/2+(diameter-boreDiameter)/4 +setScrewNutOffset, 0, height/2 - (height+hubZOffset)/2]) {
							rotate([0,-90,0]) {
								captiveNut( nutSize, setScrewDiameter, depth=height/2+1, holeLengthTop=hubWidth/2+setScrewNutOffset
									+(boreDiameter-shaftFlatDiameter), holeLengthBottom=hubWidth+baseFilletRadius-setScrewNutOffset);
							}
						} 
				} else {
					rotate([ 0,0, (360/nuts)*i ])
						translate([boreDiameter/2+(diameter-boreDiameter)/4 +setScrewNutOffset,	0, height/2 - (height+hubZOffset)/2]) {
							rotate([0,-90,0]) { 
								captiveNut( nutSize, setScrewDiameter, depth=height/2+1, holeLengthTop=hubWidth/2+setScrewNutOffset
									+(boreDiameter-shaftFlatDiameter), holeLengthBottom=hubWidth+baseFilletRadius-setScrewNutOffset);
							}
						} 
				}
			}
		}
	}
}


// This is the captive nut module  
module captiveNut( nutSize, setScrewHoleDiameter=3, 
	depth=10, holeLengthTop=5, holeLengthBottom=5 )
{
	render()
	union() {
		nut( nutSize ); 
	
		if ( depth > 0 ) 
			translate([depth/2,0,0]) 
				cube( [depth, nutSize[0], nutSize[1]], center=true );
	
		translate([0,0,-(nutSize[1]/2)-holeLengthBottom]) 
			cylinder(r=setScrewHoleDiameter/2, h=nutSize[1]+holeLengthTop+holeLengthBottom, $fn=15);
	}
}

// nutSize = [inDiameter,thickness]
module nut( nutSize ) { 
	side = nutSize[0] * tan( 180/6 );
	if ( nutSize[0] * nutSize[1] != 0 ) {
		for ( i = [0 : 2] ) {
			rotate( i*120, [0, 0, 1]) 
				cube( [side, nutSize[0], nutSize[1]], center=true );
		}
	}
}