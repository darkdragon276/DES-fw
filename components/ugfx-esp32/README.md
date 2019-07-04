# Vietnam font 


'U+0020-U+002F,U+0030-U+0039,U+003A-U+0040,U+0041-U+005A,U+005B-U+0060,U+0061-U+007A,U+007B-U+007E,U+00C0-U+00C3,U+00C8-U+00CA,U+00CC-U+00CD,U+00D0,U+00D2-U+00D5,U+00D9-U+00DA,U+00DD,U+00E0-U+00E3,U+00E8-U+00EA,U+00EC-U+00ED,U+00F2-U+00F5,U+00F9-U+00FA,U+00FD,U+0102-U+0103,U+0110-U+0111,U+0128-U+0129,U+0168-U+0169,U+01A0-U+01B0,U+1EA0-U+1EF9' 
//To support additional Vietnamese composite unicode characters, 
//append this range (total 337 glyphs)  
U+02C6-U+0323

RANGE 0x20-0x7E, C0-C3, C8-CA, CC-CD, D0, D2-D5, D9-DA, DD, E0-E3, E8-EA, EC-ED, F2-F5, F9-FA, FD, 0x102-103, 110-111, 128-129, 168-169, 1A0-1A3, 1B0, 1EA0-1EF9

RANGE_DEC= 32-126 192-195 200-202 204-205 208 210-213 217-218 221 224-227 232-234 236-237 242-245 249-250 253 258-259 272-273 296-297 360-361 416-419 432 7840-7929

https://int3ractive.com/2010/06/optimal-unicode-range-for-vietnamese.html
https://en.wikipedia.org/wiki/Vietnamese_Standard_Code_for_Information_Interchange

# Convert 
```
./tools/osx/mcufont import_ttf fonts/Roboto_Condensed/RobotoCondensed-Bold.ttf 24
./tools/osx/mcufont filter fonts/Roboto_Condensed/RobotoCondensed-Bold24.dat 32-126 192-195 200-202 204-205 208 210-213 217-218 221 224-227 232-234 236-237 242-245 249-250 253 258-259 272-273 296-297 360-361 416-419 432 7840-7929
./tools/osx/mcufont rlefont_export fonts/Roboto_Condensed/RobotoCondensed-Bold24.dat roboto_bold.h
```
