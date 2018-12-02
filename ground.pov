#include "colors.inc"
#include "textures.inc"

background { 
	color White
}

#declare CameraPos = <0, 15, 0>;
#declare POV = <50, 0, 50>;
camera {
	location CameraPos
	look_at POV
  up y
  right x * 1280 / 720
}

light_source {
	CameraPos 
	White
}

plane {
  y, 0
  texture {
    pigment { color rgb 0.9 }
    finish { ambient 0.9 }
  }
}

sphere {
  <48,0,51>
  .2
  pigment { color Black }
}
sphere {
  <12,0,28>
  .2
  pigment { color Black }
}
sphere {
  <33,0,11>
  .2
  pigment { color Black }
}
sphere {
  <22,0,62>
  .2
  pigment { color Black }
}

sphere {
  <18,0,25>
  .2
  pigment { color Red }
}
torus {
  1.02698, .1
  translate <18,0,25>
  pigment { color Red }
}
sphere {
  <35,0,30>
  .2
  pigment { color Red }
}
torus {
  1.601299, .1
  translate <35,0,30>
  pigment { color Red }
}
